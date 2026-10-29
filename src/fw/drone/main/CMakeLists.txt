idf_component_register(SRCS main.c flight_rt.c radio.c settings.c flight_math.c hover.c camera.c
                       INCLUDE_DIRS .
                       LDFRAGMENTS flight.lf
                       PRIV_REQUIRES esp_adc esp_driver_gpio driver esp_driver_ledc esp_driver_uart esp_driver_pcnt esp_timer esp_wifi app_update mbedtls nvs_flash)
# Keep our strings in our own rodata rather than the image-wide merged pool, so changing them moves nothing else.
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-merge-constants)
