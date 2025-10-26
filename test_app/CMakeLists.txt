cmake_minimum_required(VERSION 3.22)

# Paths to components that will be built in the test_app.
# If the components have unit tests (in its test subdirectory) it will be run too.
set(EXTRA_COMPONENT_DIRS ../components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(habizap_test_app)
