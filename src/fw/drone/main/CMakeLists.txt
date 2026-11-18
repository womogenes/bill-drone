# The compute benchmark (make bench) adds the MPC prototype, timed beside the tick but not flown.
set(BENCH_SOURCES)
if(BENCH)
    set(BENCH_SOURCES mpc.c)
endif()
idf_component_register(SRCS main.c flight_rt.c radio.c settings.c flight_math.c hover.c ekf.c guard.c flip.c recover.c indi.c camera.c gps.c baro.c ${BENCH_SOURCES}
                       INCLUDE_DIRS .
                       LDFRAGMENTS flight.lf
                       PRIV_REQUIRES esp_adc esp_driver_gpio driver esp_driver_ledc esp_driver_uart esp_driver_pcnt esp_timer esp_wifi app_update mbedtls nvs_flash)
# Keep our strings in our own rodata rather than the image-wide merged pool, so changing them moves nothing else.
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-merge-constants)
if(BENCH)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE BENCH=1)
endif()
# Smaller code for the control tick's glue and the hold (flight.lf): its cost is flash cache misses, not instructions, and hover.c at -Os leaves room in IRAM for flight_math.c. Measured with make bench: -Os here cut the tick by about 15%; flip.c and ekf.c, which compute more, stay at -O2.
set_source_files_properties(hover.c flight_rt.c guard.c PROPERTIES COMPILE_OPTIONS -Os)
