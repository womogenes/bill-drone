// Thrust stand: one ESC on GPIO25 and up to two HX711 load cells (A: DT GPIO32, SCK GPIO33; B: DT GPIO39, SCK GPIO17), driven from the laptop by sw/stand.py over USB at 921600 baud or over Wi-Fi: it joins the network in the repository's .env (WIFI_SSID, WIFI_PASSWORD, built in by main/CMakeLists.txt) and takes up to CLIENTS TCP clients on port 3333 (a run and the dashboard's stand page together), each getting every line and each able to command, announcing "STAND <ip> 3333" by UDP broadcast to port 3334 every second so the laptop finds it. A client beyond CLIENTS replaces the oldest. New firmware also comes over Wi-Fi (sw/stand.py flash): its length as 4 bytes little-endian, then the image, on TCP port 3335; the stand answers "OK" and restarts into it, or "ERR <reason>", and refuses while the motor is commanded. The ESC pulse is the drone's (fw/drone/main/flight_rt.c esc_us): 400 Hz, 1000 us plus the throttle (1-1200), and 950 us for throttle 0 (stopped, below the calibrated bottom so the ESC arms and stays off), so a throttle here is a throttle there.
// Laptop to stand, one command per line: "T <throttle>" sets the throttle, "P <us>" sets a raw pulse for the ESC's range calibration (1000-2200 us), "Z <hz>" sets the ESC's PWM rate (50-400; some ESCs read only 50 Hz servo-style signals, so calibrating or testing those needs it), "C <mask>" chooses the cells read (bit 0 A, bit 1 B; A alone at start, since an unwired B on GPIO39, which has no pull-up, floats). T and P are held for WATCHDOG_US; without a fresh one the pulse falls back to 950 us (motor stopped).
// Stand to laptop, one line each: "R <reason>" once at boot, why the board last reset (esp_reset_reason: 1 power-on, 3 software, 4 panic, 9 brownout, ...), "F <cell> <us> <raw>" a load cell's reading and when its conversion finished (this board's clock, us), "E <us> <pulse>" the ESC pulse from that moment on, "H <us>" a heartbeat every 100 ms, "V <us> <mV>" the supply voltage every 10 ms (GPIO35 through the drone's 100k/10k divider, battery side of the ESC, with 100 nF at the pin; it floats when unwired), "W <ip>" on USB when it joins the network.
// The HX711 converts at 10 Hz unless its RATE pin is high (80 Hz); step timing wants 80 Hz.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "wifi_credentials.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define BAUD 921600
#define ESC_PIN 25
#define ESC_HZ 400
#define PULSE_STOP 950
#define PULSE_MIN 1000
#define PULSE_MAX 2200
#define THROTTLE_MAX 1200
#define WATCHDOG_US 300000
#define CELLS 2
#define LED GPIO_NUM_2
// Supply voltage on GPIO35 (ADC1 channel 7) through a 100k/10k divider, as on the drone (fw/drone/main/main.c).
#define SUPPLY ADC_CHANNEL_7
#define DIVIDER ((100000 + 10000) / 10000)
#define TCP_PORT 3333
#define BEACON_PORT 3334
#define UPDATE_PORT 3335
#define CLIENTS 4

static const int dout_pins[CELLS] = {32, 39}, sck_pins[CELLS] = {33, 17};
static portMUX_TYPE transfer = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE command = portMUX_INITIALIZER_UNLOCKED;
static int64_t commanded_us;
static volatile int cells = 1;
static SemaphoreHandle_t esc;
static int esc_hz = ESC_HZ, pulse_now = -1;
static SemaphoreHandle_t client_lock;
static int clients[CLIENTS] = {-1, -1, -1, -1};
static int oldest;
static char address[16];
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t calibration;

// To USB and to every Wi-Fi client. A line a client's socket can't take at once is dropped rather than stalling the sampling.
static void write_line(const char *line) {
    uart_write_bytes(UART_NUM_0, line, strlen(line));
    xSemaphoreTake(client_lock, portMAX_DELAY);
    for (int i = 0; i < CLIENTS; i++)
        if (clients[i] >= 0) send(clients[i], line, strlen(line), MSG_DONTWAIT);
    xSemaphoreGive(client_lock);
}

// The ESC pulse width (us) at esc_hz in 14-bit duty. force sets it even if unchanged (after a rate change).
static void apply_pulse(int pulse, bool force) {
    if (pulse < PULSE_STOP) pulse = PULSE_STOP;
    if (pulse > PULSE_MAX) pulse = PULSE_MAX;
    xSemaphoreTake(esc, portMAX_DELAY);
    if (pulse == pulse_now && !force) {
        xSemaphoreGive(esc);
        return;
    }
    pulse_now = pulse;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)((uint64_t)pulse * esc_hz * (1 << 14) / 1000000));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    char line[48];
    snprintf(line, sizeof(line), "E %lld %d %d\n", (long long)esp_timer_get_time(), pulse, esc_hz);
    write_line(line);
    xSemaphoreGive(esc);
}

static void set_pulse(int pulse) {
    apply_pulse(pulse, false);
}

// A finished conversion, if the cell has one (DOUT low): 24 bits, then a 25th pulse for channel A at gain 128. A clock-high pause over 60 us powers the HX711 down, so the transfer runs with interrupts off.
static bool read_cell(int cell, int32_t *value) {
    if (gpio_get_level(dout_pins[cell])) return false;
    int32_t raw = 0;
    portENTER_CRITICAL(&transfer);
    for (int i = 0; i < 24; i++) {
        gpio_set_level(sck_pins[cell], 1);
        esp_rom_delay_us(1);
        raw = raw << 1 | gpio_get_level(dout_pins[cell]);
        gpio_set_level(sck_pins[cell], 0);
        esp_rom_delay_us(1);
    }
    gpio_set_level(sck_pins[cell], 1);
    esp_rom_delay_us(1);
    gpio_set_level(sck_pins[cell], 0);
    portEXIT_CRITICAL(&transfer);
    *value = raw & 0x800000 ? raw - 0x1000000 : raw;
    return true;
}

static void mark_command(void) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&command);
    commanded_us = now;
    portEXIT_CRITICAL(&command);
}

// The supply in millivolts, averaged over four conversions.
static int supply_mv(void) {
    int sum = 0;
    for (int i = 0; i < 4; i++) {
        int raw, millivolts;
        adc_oneshot_read(adc, SUPPLY, &raw);
        adc_cali_raw_to_voltage(calibration, raw, &millivolts);
        sum += millivolts;
    }
    return sum / 4 * DIVIDER;
}

// Every tick (10 ms): each chosen cell's finished conversion, the supply voltage, the watchdog, the heartbeat.
static void sample_task(void *arg) {
    int64_t heartbeat_us = 0;
    while (true) {
        char volts[40];
        snprintf(volts, sizeof(volts), "V %lld %d\n", (long long)esp_timer_get_time(), supply_mv());
        write_line(volts);
        for (int cell = 0; cell < CELLS; cell++) {
            int32_t raw;
            int64_t now = esp_timer_get_time();
            if (cells >> cell & 1 && read_cell(cell, &raw)) {
                char line[48];
                snprintf(line, sizeof(line), "F %d %lld %ld\n", cell, (long long)now, (long)raw);
                write_line(line);
            }
        }
        int64_t now = esp_timer_get_time();
        // The command time is 64-bit and written on the other core: read under the lock so it can't tear.
        portENTER_CRITICAL(&command);
        bool fresh = now - commanded_us <= WATCHDOG_US;
        portEXIT_CRITICAL(&command);
        if (!fresh) set_pulse(PULSE_STOP);
        if (now - heartbeat_us >= 100000) {
            heartbeat_us = now;
            char line[32];
            snprintf(line, sizeof(line), "H %lld\n", (long long)now);
            write_line(line);
            gpio_set_level(LED, fresh);
        }
        vTaskDelay(1);
    }
}

static void handle_line(const char *line) {
    int value;
    if (sscanf(line, "T %d", &value) == 1 && value >= 0 && value <= THROTTLE_MAX) {
        mark_command();
        set_pulse(value ? PULSE_MIN + value : PULSE_STOP);
    } else if (sscanf(line, "P %d", &value) == 1 && value >= PULSE_MIN && value <= PULSE_MAX) {
        mark_command();
        set_pulse(value);
    } else if (sscanf(line, "Z %d", &value) == 1 && value >= 50 && value <= ESC_HZ) {
        esc_hz = value;
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, esc_hz);
        apply_pulse(pulse_now < 0 ? PULSE_STOP : pulse_now, true);
    } else if (sscanf(line, "C %d", &value) == 1 && value >= 0 && value < 1 << CELLS) {
        cells = value;
    }
}

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
        address[0] = 0;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        snprintf(address, sizeof(address), IPSTR, IP2STR(&((ip_event_got_ip_t *)data)->ip_info.ip));
        char line[24];
        snprintf(line, sizeof(line), "W %s\n", address);
        uart_write_bytes(UART_NUM_0, line, strlen(line));
    }
}

static void start_wifi(void) {
    esp_err_t status = nvs_flash_init();
    if (status == ESP_ERR_NVS_NO_FREE_PAGES || status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(status);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_set_hostname(esp_netif_create_default_wifi_sta(), "stand");
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL));
    wifi_config_t config = {0};
    strncpy((char *)config.sta.ssid, WIFI_SSID, sizeof(config.sta.ssid));
    strncpy((char *)config.sta.password, WIFI_PASSWORD, sizeof(config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Power save holds packets for up to a beacon interval (~100 ms), a third of the motor watchdog.
    esp_wifi_set_ps(WIFI_PS_NONE);
}

static void beacon_task(void *arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    struct sockaddr_in to = {.sin_family = AF_INET, .sin_port = htons(BEACON_PORT), .sin_addr.s_addr = htonl(INADDR_BROADCAST)};
    while (true) {
        if (address[0]) {
            char line[40];
            snprintf(line, sizeof(line), "STAND %s %d\n", address, TCP_PORT);
            sendto(sock, line, strlen(line), 0, (struct sockaddr *)&to, sizeof(to));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void reply(int sock, const char *text) {
    send(sock, text, strlen(text), 0);
}

// One update at a time: the image into the partition not running, then a restart into it. A failed update leaves the running firmware as it was.
static void update_task(void *arg) {
    int listener = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in at = {.sin_family = AF_INET, .sin_port = htons(UPDATE_PORT), .sin_addr.s_addr = htonl(INADDR_ANY)};
    bind(listener, (struct sockaddr *)&at, sizeof(at));
    listen(listener, 1);
    static char chunk[4096];
    while (true) {
        int sock = accept(listener, NULL, NULL);
        if (sock < 0) continue;
        struct timeval timeout = {.tv_sec = 10};
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        portENTER_CRITICAL(&command);
        bool running = esp_timer_get_time() - commanded_us <= 1000000;
        portEXIT_CRITICAL(&command);
        uint8_t header[4];
        size_t have = 0;
        int got;
        while (have < 4 && (got = recv(sock, header + have, 4 - have, 0)) > 0) have += got;
        uint32_t size = header[0] | header[1] << 8 | header[2] << 16 | (uint32_t)header[3] << 24;
        const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
        esp_ota_handle_t ota;
        if (running) {
            reply(sock, "ERR motor commanded in the last second\n");
        } else if (have < 4 || !target || size == 0 || size > target->size) {
            reply(sock, "ERR bad length or no update partition\n");
        } else if (esp_ota_begin(target, size, &ota) != ESP_OK) {
            reply(sock, "ERR begin\n");
        } else {
            uint32_t written = 0;
            esp_err_t status = ESP_OK;
            while (written < size && status == ESP_OK && (got = recv(sock, chunk, size - written < sizeof(chunk) ? size - written : sizeof(chunk), 0)) > 0) {
                status = esp_ota_write(ota, chunk, got);
                written += got;
            }
            if (written < size || status != ESP_OK) {
                esp_ota_abort(ota);
                reply(sock, "ERR short or failed write\n");
            } else if (esp_ota_end(ota) != ESP_OK || esp_ota_set_boot_partition(target) != ESP_OK) {
                reply(sock, "ERR image rejected\n");
            } else {
                reply(sock, "OK\n");
                shutdown(sock, SHUT_RDWR);
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        }
        close(sock);
    }
}

// One task per client, reading its commands until it closes.
static void client_task(void *arg) {
    int sock = (int)(intptr_t)arg;
    char line[64], chunk[128];
    size_t length = 0;
    int got;
    while ((got = recv(sock, chunk, sizeof(chunk), 0)) > 0) {
        for (int i = 0; i < got; i++) {
            if (chunk[i] != '\n' && length < sizeof(line) - 1) {
                line[length++] = chunk[i];
                continue;
            }
            line[length] = 0;
            length = 0;
            handle_line(line);
        }
    }
    xSemaphoreTake(client_lock, portMAX_DELAY);
    for (int i = 0; i < CLIENTS; i++)
        if (clients[i] == sock) clients[i] = -1;
    xSemaphoreGive(client_lock);
    close(sock);
    vTaskDelete(NULL);
}

// Accepts clients into a free slot, or the oldest one's when all are taken (its task then sees the socket shut and ends).
static void server_task(void *arg) {
    int listener = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in at = {.sin_family = AF_INET, .sin_port = htons(TCP_PORT), .sin_addr.s_addr = htonl(INADDR_ANY)};
    bind(listener, (struct sockaddr *)&at, sizeof(at));
    listen(listener, 2);
    while (true) {
        int sock = accept(listener, NULL, NULL);
        if (sock < 0) continue;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        xSemaphoreTake(client_lock, portMAX_DELAY);
        int slot = -1;
        for (int i = 0; i < CLIENTS && slot < 0; i++)
            if (clients[i] < 0) slot = i;
        if (slot < 0) {
            slot = oldest;
            oldest = (oldest + 1) % CLIENTS;
            shutdown(clients[slot], SHUT_RDWR);
        }
        clients[slot] = sock;
        xSemaphoreGive(client_lock);
        xTaskCreatePinnedToCore(client_task, "client", 4096, (void *)(intptr_t)sock, 5, NULL, 0);
    }
}

void app_main(void) {
    uart_config_t uart = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));

    client_lock = xSemaphoreCreateMutex();
    char reason[16];
    snprintf(reason, sizeof(reason), "R %d\n", (int)esp_reset_reason());
    write_line(reason);
    esc = xSemaphoreCreateMutex();
    ledc_timer_config_t timer = {.speed_mode = LEDC_HIGH_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT, .timer_num = LEDC_TIMER_0, .freq_hz = ESC_HZ, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channel = {.gpio_num = ESC_PIN, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    set_pulse(PULSE_STOP);

    gpio_config_t led = {.pin_bit_mask = 1ULL << LED, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&led));
    for (int cell = 0; cell < CELLS; cell++) {
        gpio_config_t sck = {.pin_bit_mask = 1ULL << sck_pins[cell], .mode = GPIO_MODE_OUTPUT};
        ESP_ERROR_CHECK(gpio_config(&sck));
        gpio_set_level(sck_pins[cell], 0);
        gpio_config_t dout = {.pin_bit_mask = 1ULL << dout_pins[cell], .mode = GPIO_MODE_INPUT, .pull_up_en = dout_pins[cell] < 34};
        ESP_ERROR_CHECK(gpio_config(&dout));
    }
    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
    adc_oneshot_chan_cfg_t sensing = {.atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, SUPPLY, &sensing));
    adc_cali_line_fitting_config_t fitting = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&fitting, &calibration));
    commanded_us = -WATCHDOG_US;
    xTaskCreatePinnedToCore(sample_task, "sample", 4096, NULL, 10, NULL, 1);

    start_wifi();
    xTaskCreatePinnedToCore(server_task, "server", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(update_task, "update", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(beacon_task, "beacon", 3072, NULL, 2, NULL, 0);

    char line[64];
    size_t length = 0;
    while (true) {
        uint8_t byte;
        if (uart_read_bytes(UART_NUM_0, &byte, 1, portMAX_DELAY) != 1) continue;
        if (byte != '\n' && length < sizeof(line) - 1) {
            line[length++] = byte;
            continue;
        }
        line[length] = 0;
        length = 0;
        handle_line(line);
    }
}
