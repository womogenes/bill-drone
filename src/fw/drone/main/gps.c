// The GPS receiver (gps.h): UBX over UART1, configured into the RAM layer at every start, since the module keeps no settings without its backup supply.
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "gps.h"

#define PORT UART_NUM_1
#define RX 23
#define TX 5
#define GPS_BAUD 115200
// A fresh M10 talks 38400 (Beitian ships some at 9600 or 115200); 115200 first, for a module that kept its baud across a drone restart.
static const uint32_t bauds[] = {115200, 38400, 9600, 230400};
// How long each baud gets to show a NAV-PVT, and how long without one sends the task back to searching.
#define LISTEN_MS 1500
#define SILENCE_US 3000000
// The diagnostics' poll period: 1 Hz keeps NAV-SAT (about 600 bytes with 50 satellites listed) at a twentieth of the link beside NAV-PVT's 1000 bytes/s.
#define POLL_US 1000000

static const UbxSetting configuration[] = {
    {0x40520001, GPS_BAUD},  // CFG-UART1-BAUDRATE
    {0x10740001, 1},         // CFG-UART1OUTPROT-UBX
    {0x10740002, 0},         // CFG-UART1OUTPROT-NMEA
    {0x20910007, 1},         // CFG-MSGOUT-UBX_NAV_PVT_UART1: every solution
    {0x30210001, 100},       // CFG-RATE-MEAS: 100 ms
    {0x20110021, 8},         // CFG-NAVSPG-DYNMODEL: airborne < 4 g
};

// Three major systems, the most that keep 10 Hz on the SPG 5.10 ROM (u-blox PCN UBX-23006557: 10 Hz for three, 5 Hz for four without the one-time-programmed high-rate mode), all in the 1575.42 MHz band; the ROM's default is GPS and Galileo only (interface description UBX-21035062 table 79). BeiDou's B1C shares GPS L1, where B1I (1561 MHz) or GLONASS (1602 MHz, near the 40th harmonic of the ESP32s' 40 MHz crystals) would widen what the front end must hear; GLONASS can be tried at 5 Hz through sw.gps_remote set.
static const UbxSetting signals[] = {
    {0x1031001f, 1},  // CFG-SIGNAL-GPS_ENA
    {0x10310001, 1},  // CFG-SIGNAL-GPS_L1CA_ENA
    {0x10310021, 1},  // CFG-SIGNAL-GAL_ENA
    {0x10310007, 1},  // CFG-SIGNAL-GAL_E1_ENA
    {0x10310022, 1},  // CFG-SIGNAL-BDS_ENA
    {0x1031000d, 0},  // CFG-SIGNAL-BDS_B1_ENA
    {0x1031000f, 1},  // CFG-SIGNAL-BDS_B1C_ENA
    {0x10310025, 0},  // CFG-SIGNAL-GLO_ENA
    {0x10310024, 1},  // CFG-SIGNAL-QZSS_ENA
    {0x10310012, 1},  // CFG-SIGNAL-QZSS_L1CA_ENA
    {0x10310020, 1},  // CFG-SIGNAL-SBAS_ENA
    {0x10310005, 1},  // CFG-SIGNAL-SBAS_L1CA_ENA
};

// WAAS corrections for GPS: the ROM's default search mask (0x72bc8) leaves out PRN 135 (Galaxy 30, operational since 2022) and spends channels on PRNs never seen from California, so it searches only WAAS's 131 (Eutelsat 117 West B), 133 (SES-15) and 135 (FAA PRN assignments, January 2026).
static const UbxSetting sbas[] = {
    {0x10360004, 1},       // CFG-SBAS-USE_DIFFCORR
    {0x10360003, 1},       // CFG-SBAS-USE_RANGING
    {0x50360006, 0xa800},  // CFG-SBAS-PRNSCANMASK: PRN 131, 133, 135
};

// The jamming and interference monitor (off in the ROM's defaults), which fills MON-RF's jamming state; its thresholds stay at the defaults (broadband 3, CW 15).
static const UbxSetting interference[] = {
    {0x1041000d, 1},  // CFG-ITFM-ENABLE
};

// Checked (VALGET) and set where they differ, at GPS_BAUD once the module answers there, each group in its own request so a key the ROM refuses costs only its group, in GpsSky.config's order. The output group repeats the search's configuration without the baud, so its acknowledgement can be heard.
static const struct {
    const UbxSetting *settings;
    int count;
} groups[] = {
    {signals, sizeof(signals) / sizeof(signals[0])},
    {sbas, sizeof(sbas) / sizeof(sbas[0])},
    {interference, sizeof(interference) / sizeof(interference[0])},
    {configuration + 1, sizeof(configuration) / sizeof(configuration[0]) - 1},
};
#define GROUPS (int)(sizeof(groups) / sizeof(groups[0]))

static QueueHandle_t events;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static GpsStats stats = {.fix = {.received_us = -1}};
static GpsSky sky = {.sat_us = -1, .rf_us = -1};
// The tunnel's frames waiting for gps_take, whole frames only, kept while tunnel_until (esp_timer µs) has not passed.
#define TUNNEL 4096
#define TUNNEL_IDLE_US 5000000
static uint8_t tunnel[TUNNEL];
static size_t tunnel_length;
static uint32_t tunnel_dropped;
static int64_t tunnel_until;
// What the configuration waits for, filled by receive() in the same task: the newest acknowledgement of a CFG message (answer_id; answer 1 accepted, 0 refused, -1 none yet) and VALGET reply.
static int answer;
static uint8_t answer_id;
static bool values_received;
static UbxFrame values;

// A frame other than NAV-PVT, rebuilt as the module sent it, for gps_take.
static void keep(const UbxFrame *frame, int64_t now) {
    size_t length = frame->length + 8;
    portENTER_CRITICAL(&lock);
    if (now < tunnel_until) {
        if (tunnel_length + length > TUNNEL) tunnel_dropped++;
        else tunnel_length += ubx_frame(frame->message_class, frame->id, frame->payload, frame->length, tunnel + tunnel_length, length);
    }
    portEXIT_CRITICAL(&lock);
}

// The diagnostics and the configuration's answers from a frame other than NAV-PVT.
static void note(const UbxFrame *frame, int64_t now) {
    static UbxSatellite satellites[GPS_SATELLITES];
    uint8_t id;
    bool acked;
    uint32_t itow;
    int count, known;
    UbxRf rf;
    if (ubx_ack(frame, &id, &acked)) {
        answer_id = id;
        answer = acked;
    } else if (frame->message_class == UBX_CFG && frame->id == UBX_CFG_VALGET) {
        values = *frame;
        values_received = true;
    } else if (ubx_sat(frame, &itow, satellites, GPS_SATELLITES, &count, &known)) {
        portENTER_CRITICAL(&lock);
        memcpy(sky.satellites, satellites, count * sizeof(satellites[0]));
        sky.count = count;
        sky.known = known;
        sky.sat_itow_ms = itow;
        sky.sat_us = now;
        portEXIT_CRITICAL(&lock);
    } else if (ubx_rf(frame, &rf)) {
        portENTER_CRITICAL(&lock);
        sky.rf = rf;
        sky.rf_us = now;
        portEXIT_CRITICAL(&lock);
    }
}

// Waits up to wait_ms for the driver's next event (the line idle for about a byte after a burst, so within a millisecond of a frame's end) and parses what is buffered; returns whether a NAV-PVT came.
static bool receive(UbxParser *parser, uint32_t wait_ms) {
    static uint8_t bytes[256];
    uart_event_t event;
    if (xQueueReceive(events, &event, pdMS_TO_TICKS(wait_ms)) != pdTRUE) return false;
    if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) uart_flush_input(PORT);
    size_t buffered = 0;
    uart_get_buffered_data_len(PORT, &buffered);
    int n = buffered ? uart_read_bytes(PORT, bytes, buffered < sizeof(bytes) ? buffered : sizeof(bytes), 0) : 0;
    if (n <= 0) return false;
    bool solution = false;
    const uint8_t *data = bytes;
    size_t length = n;
    static UbxFrame frame;
    while (true) {
        uint32_t skipped = parser->skipped, corrupt = parser->corrupt;
        bool complete = ubx_parse(parser, &data, &length, &frame);
        int64_t now = esp_timer_get_time();
        UbxPvt pvt;
        bool decoded = complete && ubx_pvt(&frame, &pvt);
        if (complete && !decoded) {
            note(&frame, now);
            keep(&frame, now);
        }
        portENTER_CRITICAL(&lock);
        stats.skipped += parser->skipped - skipped;
        stats.corrupt += parser->corrupt - corrupt;
        if (decoded) {
            stats.messages++;
            stats.fix.pvt = pvt;
            stats.fix.received_us = now;
        }
        portEXIT_CRITICAL(&lock);
        solution |= decoded;
        if (!complete) break;
    }
    return solution;
}

// Sends a CFG request and receives until the module answers it or wait_ms passes: 1 accepted (for VALGET, its values arrived), 0 refused, -1 no answer.
static int ask(UbxParser *parser, const uint8_t *request, size_t length, uint32_t wait_ms) {
    uint8_t id = request[3];
    answer = -1;
    values_received = false;
    uart_write_bytes(PORT, request, length);
    int64_t until = esp_timer_get_time() + wait_ms * 1000LL;
    while (esp_timer_get_time() < until) {
        receive(parser, 20);
        if (id == UBX_CFG_VALGET && values_received) return 1;
        if (answer >= 0 && answer_id == id) return answer;
    }
    return -1;
}

static void set_config(int group, int state) {
    portENTER_CRITICAL(&lock);
    sky.config = (sky.config & ~(3 << 2 * group)) | state << 2 * group;
    portEXIT_CRITICAL(&lock);
}

// Each group checked, and set where the module's RAM layer differs: a signal change restarts the module's GNSS part, so it happens only when needed (not again after a drone restart with the module powered), followed by the 0.5 s the interface description asks for.
static void configure_groups(UbxParser *parser) {
    static uint8_t request[160];
    for (int group = 0; group < GROUPS; group++) set_config(group, GPS_PENDING);
    for (int group = 0; group < GROUPS; group++) {
        const UbxSetting *settings = groups[group].settings;
        int count = groups[group].count;
        size_t length = ubx_valget(settings, count, request, sizeof(request));
        if (ask(parser, request, length, 500) == 1 && ubx_valget_matches(&values, settings, count)) {
            set_config(group, GPS_SET);
            continue;
        }
        length = ubx_valset(settings, count, request, sizeof(request));
        int result = ask(parser, request, length, 1000);
        set_config(group, result > 0 ? GPS_SET : result == 0 ? GPS_REFUSED : GPS_UNANSWERED);
        if (result > 0 && settings == signals) {
            int64_t until = esp_timer_get_time() + 500000;
            while (esp_timer_get_time() < until) receive(parser, 50);
        }
    }
}

static void gps_task(void *arg) {
    static UbxParser parser;
    static uint8_t request[96];
    size_t request_length = ubx_valset(configuration, sizeof(configuration) / sizeof(configuration[0]), request, sizeof(request));
    // NAV-SAT and MON-RF polls: a UBX frame with an empty payload each.
    uint8_t polls[2][8];
    ubx_frame(UBX_NAV, UBX_NAV_SAT, NULL, 0, polls[0], 8);
    ubx_frame(UBX_MON, UBX_MON_RF, NULL, 0, polls[1], 8);
    int next = 0;
    while (true) {
        // Configure at a candidate baud (the module switches after acknowledging), then listen at ours.
        uint32_t baud = bauds[next];
        next = (next + 1) % (sizeof(bauds) / sizeof(bauds[0]));
        uart_set_baudrate(PORT, baud);
        uart_flush_input(PORT);
        uart_write_bytes(PORT, request, request_length);
        uart_wait_tx_done(PORT, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(50));
        uart_set_baudrate(PORT, GPS_BAUD);
        uart_flush_input(PORT);
        xQueueReset(events);
        memset(&parser, 0, sizeof(parser));
        int64_t until = esp_timer_get_time() + LISTEN_MS * 1000LL, heard = -1;
        while (esp_timer_get_time() < until)
            if (receive(&parser, 100)) {
                heard = esp_timer_get_time();
                break;
            }
        if (heard < 0) continue;
        configure_groups(&parser);
        portENTER_CRITICAL(&lock);
        stats.configured = true;
        portEXIT_CRITICAL(&lock);
        next = 0;
        heard = esp_timer_get_time();
        int64_t polled = heard - POLL_US;
        while (esp_timer_get_time() - heard < SILENCE_US) {
            if (receive(&parser, 100)) heard = esp_timer_get_time();
            if (esp_timer_get_time() - polled >= POLL_US) {
                polled = esp_timer_get_time();
                uart_write_bytes(PORT, polls[0], 8);
                uart_write_bytes(PORT, polls[1], 8);
            }
        }
        portENTER_CRITICAL(&lock);
        stats.configured = false;
        portEXIT_CRITICAL(&lock);
    }
}

void gps_start(void) {
    uart_config_t link = {.baud_rate = GPS_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                          .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(PORT, 1024, 0, 16, &events, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &link));
    // The pull-up holds the line idle (high) without a module.
    ESP_ERROR_CHECK(uart_set_pin(PORT, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_pull_mode(RX, GPIO_PULLUP_ONLY);
    if (xTaskCreatePinnedToCore(gps_task, "gps", 4096, NULL, 3, NULL, 0) != pdPASS) abort();
}

static void hold(int64_t until) {
    portENTER_CRITICAL(&lock);
    if (until > tunnel_until) tunnel_until = until;
    portEXIT_CRITICAL(&lock);
}

void gps_tunnel_hold(uint32_t ms) {
    hold(esp_timer_get_time() + ms * 1000LL);
}

void gps_send(const uint8_t *bytes, size_t length) {
    hold(esp_timer_get_time() + TUNNEL_IDLE_US);
    uart_write_bytes(PORT, bytes, length);
}

size_t gps_take(uint8_t *out, size_t size, uint32_t *dropped) {
    hold(esp_timer_get_time() + TUNNEL_IDLE_US);
    portENTER_CRITICAL(&lock);
    // Whole frames only: as many from the front as fit.
    size_t taken = 0;
    while (taken + 6 <= tunnel_length) {
        size_t length = (tunnel[taken + 4] | tunnel[taken + 5] << 8) + 8;
        if (taken + length > size) break;
        taken += length;
    }
    memcpy(out, tunnel, taken);
    memmove(tunnel, tunnel + taken, tunnel_length - taken);
    tunnel_length -= taken;
    *dropped = tunnel_dropped;
    portEXIT_CRITICAL(&lock);
    return taken;
}

bool gps_fix(GpsFix *fix, uint32_t max_age_us) {
    portENTER_CRITICAL(&lock);
    *fix = stats.fix;
    portEXIT_CRITICAL(&lock);
    return fix->received_us >= 0 && esp_timer_get_time() - fix->received_us <= max_age_us;
}

void gps_stats(GpsStats *out) {
    portENTER_CRITICAL(&lock);
    *out = stats;
    portEXIT_CRITICAL(&lock);
}

void gps_sky(GpsSky *out) {
    portENTER_CRITICAL(&lock);
    *out = sky;
    portEXIT_CRITICAL(&lock);
}

static uint32_t age_ms(int64_t at, int64_t now) {
    return at < 0 ? UINT32_MAX : (now - at) / 1000;
}

size_t gps_report(uint8_t *out, size_t size, int64_t now) {
    static GpsSky copy;
    gps_sky(&copy);
    // Magic; drone time, NAV-SAT's time of week and age, MON-RF's age (ms); noise, AGC; jamming state, CW indicator, antenna status and power, configuration, satellites listed in all and here.
    size_t header = 31, length = header + copy.count * sizeof(UbxSatellite);
    if (length > size) return 0;
    uint32_t words[4] = {now / 1000, copy.sat_itow_ms, age_ms(copy.sat_us, now), age_ms(copy.rf_us, now)};
    uint8_t bytes[7] = {copy.rf.jamming, copy.rf.cw_jamming, copy.rf.antenna_status, copy.rf.antenna_power, copy.config, copy.known, copy.count};
    memcpy(out, "GPD1", 4);
    memcpy(out + 4, words, 16);
    memcpy(out + 20, &copy.rf.noise_per_ms, 2);
    memcpy(out + 22, &copy.rf.agc_count, 2);
    memcpy(out + 24, bytes, 7);
    memcpy(out + header, copy.satellites, copy.count * sizeof(UbxSatellite));
    return length;
}
