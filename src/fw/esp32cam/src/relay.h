#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include "driver/uart.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "../../relay_link.h"
extern int radioPower;
void loadRadio();

// The drone's radio relay (its radio_via_camera setting, fw/relay_link.h): the drone's ESP-NOW link goes out through this board's radio and u.FL dipole instead of the drone's own antenna under the GPS. The drone asks for the mode with its K R request (update.h) at every boot and keeps asking; the mode is kept in Preferences "relay" and applied at the next restart, since switching the Wi-Fi driver at runtime cost JPEG capture its DMA memory (ota.h). In relay mode the station takes the drone's MAC before it starts, so the controller needs no change, joins no network and stays on the link's channel 6; FPV and the radio preview, which would share the link's channel and airtime, and the access point are refused (H, A, V, P and Q). The drone never starts its own Wi-Fi in relay mode, so the MAC is on the air once. If the drone sends nothing to relay for silentMs the mode is dropped and the camera restarts with its own MAC and Wi-Fi, so a drone that went back to its own radio (an older drone image, or its fallback) is never answered by a second radio with its MAC.
namespace relay {
constexpr int channel = 6;
constexpr uint32_t silentMs = 20000;
// The stored mode (on, the drone's MAC and its controller's) and whether it runs: on but not active means the station refused the MAC.
bool on = false, active = false;
uint8_t own[6], peer[6];
uart_port_t port;
volatile uint32_t lastFrameMs = 0;
// For K S: packets heard and relayed to the drone, sends asked, sends delivered, and frames dropped for want of room on the wire.
volatile uint32_t heard = 0, asked = 0, delivered = 0, dropped = 0;
// The rate set, a fw/link.h link_rates index (0 1 Mbps, 1 and 2 long range 500 and 250 kbps, 3 2 Mbps, 4 5.5 Mbps, 5 11 Mbps, 6 802.11g 6 Mbps), as each frame from the drone asks.
int rateSet = -1;
RelayParser parser;

inline void load() {
    Preferences store;
    if (!store.begin("relay", true))
        return;
    on = store.getUChar("on", 0) && store.getBytes("own", own, 6) == 6 && store.getBytes("peer", peer, 6) == 6;
    store.end();
}

inline void save(bool wanted, const uint8_t *drone, const uint8_t *controller) {
    Preferences store;
    if (!store.begin("relay", false))
        return;
    store.putUChar("on", wanted);
    if (wanted) {
        store.putBytes("own", drone, 6);
        store.putBytes("peer", controller, 6);
    }
    store.end();
}

// One frame to the drone, from the Wi-Fi task; never waits, so a full buffer drops it (the drone reopens its send window after its own timeout).
inline void toDrone(uint8_t type, const uint8_t *payload, size_t length) {
    uint8_t frame[RELAY_FRAME];
    const size_t total = relay_frame(frame, type, 0, payload, length);
    size_t room = 0;
    if (!total || uart_get_tx_buffer_free_size(port, &room) != ESP_OK || room < total) {
        ++dropped;
        return;
    }
    uart_write_bytes(port, frame, total);
}

inline void received(const uint8_t *mac, const uint8_t *data, int length) {
    if (memcmp(mac, peer, 6) || length <= 0 || length > 250)
        return;
    uint8_t payload[RELAY_PAYLOAD];
    memcpy(payload, mac, 6);
    // This core's ESP-NOW callback has no receive information; the frame's rx_ctrl sits just before its 802.11 header and vendor element (39 bytes), as in the driver's buffer on IDF 4.4.
    const wifi_pkt_rx_ctrl_t *control = reinterpret_cast<const wifi_pkt_rx_ctrl_t *>(data - 39 - sizeof(wifi_pkt_rx_ctrl_t));
    payload[6] = uint8_t(int8_t(control->rssi));
    payload[7] = uint8_t(int8_t(control->noise_floor));
    memcpy(payload + 8, data, length);
    ++heard;
    toDrone('R', payload, 8 + length);
}

inline void sent(const uint8_t *, esp_now_send_status_t status) {
    const uint8_t failed = status != ESP_NOW_SEND_SUCCESS;
    delivered += !failed;
    toDrone('A', &failed, 1);
}

inline void setRate(int index) {
    static const wifi_phy_rate_t rates[] = {WIFI_PHY_RATE_1M_L, WIFI_PHY_RATE_LORA_500K, WIFI_PHY_RATE_LORA_250K, WIFI_PHY_RATE_2M_L, WIFI_PHY_RATE_5M_L, WIFI_PHY_RATE_11M_L, WIFI_PHY_RATE_6M};
    if (index >= 0 && index < int(sizeof(rates) / sizeof(rates[0])) && esp_wifi_config_espnow_rate(WIFI_IF_STA, rates[index]) == ESP_OK)
        rateSet = index;
}

// At boot instead of startWifi: true when relay mode runs.
inline bool start(uart_port_t link) {
    port = link;
    load();
    if (!on)
        return false;
    loadRadio();
    lastFrameMs = millis();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);
    // The MAC can only change with the station stopped; it is checked back before anything is sent.
    esp_wifi_stop();
    uint8_t mac[6] = {};
    if (esp_wifi_set_mac(WIFI_IF_STA, own) != ESP_OK || esp_wifi_start() != ESP_OK || esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK || memcmp(mac, own, 6))
        return true;
    esp_wifi_set_ps(WIFI_PS_NONE);
    // Long range as well as 802.11b/g/n, as the drone's own radio: the controller sends at the 500 kbps long-range rate.
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(radioPower);
    esp_now_peer_info_t partner = {};
    memcpy(partner.peer_addr, peer, 6);
    partner.ifidx = WIFI_IF_STA;
    if (esp_now_init() != ESP_OK || esp_now_register_recv_cb(received) != ESP_OK || esp_now_register_send_cb(sent) != ESP_OK || esp_now_add_peer(&partner) != ESP_OK)
        return true;
    setRate(0);
    active = true;
    return true;
}

// From the service task: the drone's frames, sent on as they come. A failed send is answered at once, as the send callback would.
inline void receive(const uint8_t *bytes, size_t n) {
    size_t total;
    while ((total = relay_parse(&parser, &bytes, &n))) {
        const uint8_t *frame = parser.bytes;
        const size_t length = total - RELAY_HEADER - 2;
        if (frame[4] != 'S' || !active || !length || length > 250)
            continue;
        lastFrameMs = millis();
        ++asked;
        if (frame[5] != rateSet)
            setRate(frame[5]);
        if (esp_now_send(peer, frame + RELAY_HEADER, length) != ESP_OK) {
            const uint8_t failed = 1;
            toDrone('A', &failed, 1);
        }
    }
}

inline void poll() {
    if (on && millis() - lastFrameMs > silentMs) {
        save(false, nullptr, nullptr);
        ESP.restart();
    }
}
} // namespace relay
