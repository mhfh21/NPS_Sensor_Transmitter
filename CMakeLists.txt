# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)




include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test1)


set(EXTRA_COMPONENT_DIRS "components/mpu6050")
set(EXTRA_COMPONENT_DIRS "components/mirf")


idf_build_get_property(project_dir PROJECT_DIR)
idf_build_get_property(openocd_files OPENOCD_FILES)
set(OPENOCD_FILES "${OPENOCD_FILES} ${project_dir}/mpu6050_upload.cfg")
idf_build_set_property(PROJECT_DIR ${project_dir} FORCE)