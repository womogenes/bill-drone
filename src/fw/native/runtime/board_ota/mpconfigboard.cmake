include(boards/mpconfigboard_esp32_common.cmake)
configure_file(${MICROPY_BOARD_DIR}/sdkconfig.in ${CMAKE_BINARY_DIR}/sdkconfig.ota @ONLY)
list(APPEND SDKCONFIG_DEFAULTS
    ${MICROPY_BOARD_DIR}/../board/sdkconfig
    ${CMAKE_BINARY_DIR}/sdkconfig.ota
)
