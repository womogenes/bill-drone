// Antenna test: scans for every access point and prints each one's signal strength, to compare antennas (onboard against u.FL) at the same spot. The brownout reset is off (sdkconfig.defaults) and the transmit power starts low and steps up, so a board on a weak supply still runs and the power at which it would have browned out shows as a reset or a dip.
// Lines, at 115200 baud: "R <reason>" at boot (esp_reset_reason: 1 power-on, 3 software, 4 panic, 9 brownout), "P <dBm>" when the transmit power changes, "S <scan> <ssid> <channel> <rssi dBm>" per access point per scan, "D <scan> <ms>" when a scan is done.
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define MAX_RECORDS 40
// Quarter dBm: 2, 8, 14 and 19.5 dBm (the ESP32's top), ten scans each, then the top for good.
static const int8_t powers[] = {8, 32, 56, 78};

void app_main(void) {
    printf("R %d\n", (int)esp_reset_reason());
    esp_err_t status = nvs_flash_init();
    if (status == ESP_ERR_NVS_NO_FREE_PAGES || status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(status);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    static wifi_ap_record_t records[MAX_RECORDS];
    for (int scan = 0;; scan++) {
        int step = scan / 10 < (int)sizeof(powers) ? scan / 10 : (int)sizeof(powers) - 1;
        if (scan % 10 == 0 && scan / 10 < (int)sizeof(powers)) {
            esp_wifi_set_max_tx_power(powers[step]);
            int8_t now;
            esp_wifi_get_max_tx_power(&now);
            printf("P %.2f\n", now / 4.0);
        }
        int64_t start = esp_timer_get_time();
        wifi_scan_config_t config = {.show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE, .scan_time.active = {.min = 60, .max = 120}};
        if (esp_wifi_scan_start(&config, true) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        uint16_t count = MAX_RECORDS;
        esp_wifi_scan_get_ap_records(&count, records);
        for (int i = 0; i < count; i++)
            printf("S %d %s %d %d\n", scan, records[i].ssid[0] ? (char *)records[i].ssid : "-", records[i].primary, records[i].rssi);
        printf("D %d %lld\n", scan, (long long)((esp_timer_get_time() - start) / 1000));
    }
}
