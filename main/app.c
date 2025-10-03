#include "app.h"

#include "esp_log.h"

#include "driver/i2c.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_5
#define I2C_MASTER_SDA_IO           GPIO_NUM_4
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

static const char *TAG = "APP";
static app_context_t g_ctx;

app_context_t *app_ctx(void) {
    return &g_ctx;
}

static void app_context_init() {
    ESP_LOGI(TAG, "App context initialized");
    memset(&g_ctx, 0, sizeof(g_ctx));
}

static bool init_mpu() {
    ESP_LOGI(TAG, "Initializing MPU6050");

    // I2C configuration (use default pins for your board or configure accordingly)
    const i2c_port_t port = I2C_NUM_0;
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };

    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }
    // Create and configure MPU6050 using the library API used by the test
    mpu6050_handle_t mpu = mpu6050_create(port, MPU6050_I2C_ADDRESS);
    if (mpu == NULL) {
        ESP_LOGE(TAG, "mpu6050_create returned NULL");
        return false;
    }

    err = mpu6050_config(mpu, ACCE_FS_4G, GYRO_FS_500DPS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mpu6050_config failed: %s", esp_err_to_name(err));
        mpu6050_delete(mpu);
        return false;
    }

    err = mpu6050_wake_up(mpu);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mpu6050_wake_up failed: %s", esp_err_to_name(err));
        mpu6050_delete(mpu);
        return false;
    }

    // Verify WHO_AM_I
    uint8_t devid = 0;
    err = mpu6050_get_deviceid(mpu, &devid);
    if (err != ESP_OK || devid != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I failed (err=%s, id=0x%02X)", esp_err_to_name(err), devid);
        mpu6050_delete(mpu);
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 OK (WHO_AM_I=0x%02X)", devid);

    // Store handle in app context for later use
    // Ensure app_context_t has a field: mpu6050_handle_t mpu;
    g_ctx.mpu = mpu;

    return true;
}

// Read raw accel & gyro, convert to g and deg/s
bool read_mpu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    if (g_ctx.mpu == NULL) {
        ESP_LOGE(TAG, "MPU6050 handle is NULL. Call init_mpu() first.");
        return false;
    }

    esp_err_t err;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    err = mpu6050_get_acce(g_ctx.mpu, &acce);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mpu6050_get_acce failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mpu6050_get_gyro(g_ctx.mpu, &gyro);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mpu6050_get_gyro failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mpu6050_get_temp(g_ctx.mpu, &temp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mpu6050_get_temp failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Accel[g]: x=%.2f y=%.2f z=%.2f | Gyro[dps]: x=%.2f y=%.2f z=%.2f | Temp[C]=%.2f",
             acce.acce_x, acce.acce_y, acce.acce_z,
             gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
             temp.temp);

    // Set output values
    if (ax) *ax = acce.acce_x;
    if (ay) *ay = acce.acce_y;
    if (az) *az = acce.acce_z;
    if (gx) *gx = gyro.gyro_x;
    if (gy) *gy = gyro.gyro_y;
    if (gz) *gz = gyro.gyro_z;


    return true;
}


void app_run(void) {
    ESP_LOGI(TAG, "App run: starting provisioning and subsystem init");

    app_context_init();

    // Initialize configuration storage
    if (!init_mpu()) {
        ESP_LOGE(TAG, "MPU6050 init failed");
    }

    // Read sensor data in a loop
    while (1) {
        float ax, ay, az, gx, gy, gz;
        if (!read_mpu(&ax, &ay, &az, &gx, &gy, &gz)) {
            ESP_LOGE(TAG, "Failed to read MPU6050 data");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second between reads
    }


    ESP_LOGI(TAG, "App run complete");
}
