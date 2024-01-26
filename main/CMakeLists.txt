idf_component_register(SRCS "test1.c"
                       INCLUDE_DIRS "." "../components/mpu6050"
                       INCLUDE_DIRS "." "../components/mirf"
                       REQUIRES mpu6050
                       REQUIRES mirf)
