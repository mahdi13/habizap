#include "app.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "motion.h"
// #include "heartbeat.h"
#include "heartbeat/heartbeat.h"
#include "heartbeat/session.h"
#include "../trash/controller.h"

#ifdef CONFIG_HABIZAP_TRAINING_MODE_ON
#include "training.h"
#endif /* CONFIG_HABIZAP_TRAINING_MODE_ON */

#define I2C_MASTER_SCL_IO           23
#define I2C_MASTER_SDA_IO           22
#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          CONFIG_HABIZAP_I2C_MASTER_FREQUENCY

static const char *TAG = "APP";
static app_context_t g_ctx;

static void on_controller_cmd(const controller_cmd_t *cmd, void *user_ctx) {
    (void)user_ctx;
    switch (cmd->type) {
        case CTRL_CMD_PING:
            ESP_LOGI(TAG, "Controller CMD: PING");
            break;
        case CTRL_CMD_SET_CONFIG:
            ESP_LOGI(TAG, "Controller CMD: SET_CONFIG len=%u", (unsigned)cmd->len);
            break;
        case CTRL_CMD_START_STREAM:
            ESP_LOGI(TAG, "Controller CMD: START_STREAM");
            break;
        case CTRL_CMD_STOP_STREAM:
            ESP_LOGI(TAG, "Controller CMD: STOP_STREAM");
            break;
        case CTRL_CMD_USER_ACTION:
            ESP_LOGI(TAG, "Controller CMD: USER_ACTION len=%u", (unsigned)cmd->len);
            break;
        default:
            break;
    }
}

app_context_t *app_ctx(void) {
    return &g_ctx;
}

static void app_context_init() {
    ESP_LOGI(TAG, "App context initialized");
    memset(&g_ctx, 0, sizeof(g_ctx));
}

static bool init_i2c_bus(void) {
    ESP_LOGI(TAG, "Initializing I2C bus (new driver)");

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &g_ctx.i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "I2C bus ready on port %d (SDA=%d, SCL=%d)", I2C_MASTER_PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return true;
}

static void motion_retry_timer_cb(void *arg) {
    (void) arg;
    g_ctx.motion_retry_due = true;
}

static void start_motion_retry_timer(uint64_t period_us) {
    if (g_ctx.motion_retry_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = &motion_retry_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "motion_retry"
        };
        esp_timer_handle_t h = NULL;
        if (esp_timer_create(&args, &h) == ESP_OK) {
            g_ctx.motion_retry_timer = (void *) h;
        } else {
            ESP_LOGE(TAG, "Failed to create motion retry timer");
            return;
        }
    }
    esp_timer_handle_t h = (esp_timer_handle_t) g_ctx.motion_retry_timer;
    // If already running, stop to update period
    (void) esp_timer_stop(h);
    esp_err_t e = esp_timer_start_periodic(h, period_us);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start motion retry timer: %s", esp_err_to_name(e));
    } else {
        ESP_LOGI(TAG, "Motion retry timer started (every %llu ms)", (unsigned long long)(period_us / 1000ULL));
    }
}

static void stop_motion_retry_timer(void) {
    if (g_ctx.motion_retry_timer) {
        esp_timer_handle_t h = (esp_timer_handle_t) g_ctx.motion_retry_timer;
        esp_timer_stop(h);
    }
}

static int i2c_scan(i2c_master_bus_handle_t bus) {
    if (!bus) return 0;
    ESP_LOGI(TAG, "Scanning I2C bus for devices (0x03-0x77)...");
    int found = 0;
    for (uint8_t addr = 0x03; addr < 0x78; ++addr) {
        esp_err_t p = i2c_master_probe(bus, addr, 1000);
        if (p == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
            found++;
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "No I2C devices responded on the bus");
    }
    return found;
}

static bool init_motion(void) {
    ESP_LOGI(TAG, "Initializing motion (MPU6050) on I2C bus");
    if (!g_ctx.i2c_bus) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        g_ctx.motion_ready = false;
        return false;
    }
    esp_err_t err = motion_init(g_ctx.i2c_bus, I2C_MASTER_FREQ_HZ);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "motion_init failed: %s", esp_err_to_name(err));
        g_ctx.motion_ready = false;
        // schedule retries via timer if not already scheduled
        start_motion_retry_timer(10ULL * 1000ULL * 1000ULL); // 10 seconds
        return false;
    }
    g_ctx.motion_ready = true;
    g_ctx.motion_retry_due = false;
    stop_motion_retry_timer();
    ESP_LOGI(TAG, "motion_init OK");
    return true;
}

static bool init_heartbeat(void) {
    ESP_LOGI(TAG, "Initializing heartbeat (MAX30102) on I2C bus");
    if (!g_ctx.i2c_bus) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return false;
    }


    esp_err_t err = heartbeat_init(&g_ctx.hb_ctx, g_ctx.i2c_bus, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "heartbeat_init failed: %s", esp_err_to_name(err));
        g_ctx.heartbeat_ready = false;
        return false;
    }

    // Start measurements
    ESP_ERROR_CHECK(heartbeat_start(&g_ctx.hb_ctx));

    g_ctx.heartbeat_ready = true;
    ESP_LOGI(TAG, "heartbeat_init OK");
    return true;
}

void app_run(void) {
    ESP_LOGI(TAG, "App run: starting provisioning and subsystem init");

    app_context_init();

    // Initialize controller (communication module)
    ESP_ERROR_CHECK(controller_init());
    ESP_ERROR_CHECK(controller_register_command_handler(on_controller_cmd, NULL));
    ESP_ERROR_CHECK(controller_start());
    (void)controller_request_provisioning();

    // Log chip info to help debug board-specific issues (e.g., XIAO ESP32C6 vs ESP32-C3)
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: model=%d cores=%d features=0x%x revision=%d", (int)chip_info.model, chip_info.cores,
             chip_info.features, chip_info.revision);

    if (!init_i2c_bus()) {
        ESP_LOGE(TAG, "I2C bus init failed");
        return;
    }
    // i2c_bus_health_check();
    // int devices = i2c_scan(g_ctx.i2c_bus);
    // (void)devices;
    if (!init_motion()) {
        ESP_LOGW(TAG, "Motion init failed; will continue and retry later");
        ESP_LOGW(
            TAG,
            "If you're on XIAO ESP32C6: verify SDA=D4(GPIO4), SCL=D5(GPIO5), solid 3.3V/GND to sensor, and add 4.7k-10k pull-ups to 3.3V if needed. Some breakout Qwiic/Grove cables route 5V; ensure your sensor runs at 3.3V.")
        ;
    }
    if (!init_heartbeat()) {
        ESP_LOGW(TAG, "Heartbeat init failed; continuing without MAX30102");
    }

    if (!g_ctx.hb_ctx.started) {
    }

    // #ifdef CONFIG_HABIZAP_TRAINING_MODE_ON
    //     start_data_collection();
    // #endif /* CONFIG_HABIZAP_TRAINING_MODE_ON */

    heartbeat_session_params_t heartbeat_session_params = {
        .min_seconds = 10,
        .max_seconds = 10,
        .require_contact = true,
        .poll_interval_ms = 50,
    };
    heartbeat_sample_t session_sample_result;
    ESP_ERROR_CHECK(heartbeat_session_run(&g_ctx.hb_ctx, &heartbeat_session_params, &session_sample_result));
    //
    // Print detail result:
    ESP_LOGI(TAG, "Heartbeat session result: contact=%d win=%u(%.1fs) HR=%.1f bpm%s SpO2=%.1f%%%s RED=%u IR=%u T=%.2fC",
             (int)session_sample_result.contact,
             (unsigned)session_sample_result.samples_in_window,
             (double)session_sample_result.seconds_in_window,
             session_sample_result.hr_bpm, session_sample_result.hr_valid ? "" : " (invalid)",
             session_sample_result.spo2_pct, session_sample_result.spo2_valid ? "" : " (invalid)",
             (unsigned)session_sample_result.red, (unsigned)session_sample_result.ir, session_sample_result.temp_c);


    // Read sensor data in a loop
    while (1) {
        // Motion retry driven by timer to reduce log spam
        if (g_ctx.motion_retry_due) {
            g_ctx.motion_retry_due = false; // consume the trigger
            ESP_LOGI(TAG, "Retry tick: rescanning I2C bus and retrying inits as needed...");
            int found = i2c_scan(g_ctx.i2c_bus);
            (void) found;
            if (!g_ctx.motion_ready) {
                ESP_LOGI(TAG, "Retrying motion init (timer)...");
                (void) init_motion();
            }
            if (!g_ctx.heartbeat_ready) {
                ESP_LOGI(TAG, "Retrying heartbeat init (timer)...");
                (void) init_heartbeat();
            }
        }

        if (g_ctx.motion_ready) {
            motion_sample_t m = {0};
            esp_err_t err_m = motion_read(&m);
            if (err_m == ESP_OK) {
                ESP_LOGI(TAG, "Accel[g]: x=%.3f y=%.3f z=%.3f | Gyro[dps]: x=%.3f y=%.3f z=%.3f | Temp[C]=%.2f",
                         m.ax, m.ay, m.az, m.gx, m.gy, m.gz, m.temp_c);
            } else {
                ESP_LOGE(TAG, "motion_read failed: %s", esp_err_to_name(err_m));
            }
        }

        if (g_ctx.heartbeat_ready) {
            heartbeat_sample_t hb = {0};
            esp_err_t err_h = heartbeat_read(&g_ctx.hb_ctx, &hb);
            if (err_h == ESP_OK) {
                ESP_LOGI(TAG, "Heartbeat: contact=%d win=%u(%.1fs) HR=%.1f bpm%s SpO2=%.1f%%%s RED=%u IR=%u T=%.2fC",
                         (int)hb.contact,
                         (unsigned)hb.samples_in_window,
                         (double)hb.seconds_in_window,
                         hb.hr_bpm, hb.hr_valid ? "" : " (invalid)",
                         hb.spo2_pct, hb.spo2_valid ? "" : " (invalid)",
                         (unsigned)hb.red, (unsigned)hb.ir, hb.temp_c);
                // Uplink a compact heartbeat snapshot to the controller
                (void)controller_send_heartbeat(hb.hr_bpm, hb.hr_valid, hb.spo2_pct, hb.spo2_valid, hb.temp_c);
            } else {
                ESP_LOGE(TAG, "heartbeat_read failed: %s", esp_err_to_name(err_h));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Not reached
}
