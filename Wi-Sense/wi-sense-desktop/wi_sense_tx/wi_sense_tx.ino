// =============================================================================
// Wi-Sense TX — UDP unicast to RX STA at TARGET_HZ on CH1 HT20
// Board: ESP32 Dev Module  |  Core: esp32:esp32:esp32
//
// WHY UDP instead of raw 802.11 injection:
//   esp_wifi_80211_tx() bypasses the normal 802.11 TX path. In a WPA2-secured
//   BSS, unicast Data frames MUST be CCMP-encrypted with the PTK. Raw-injected
//   frames lack CCMP, so the RX driver drops them before the LLTF estimator
//   runs → csi=0 forever. Real UDP traffic goes through lwIP → 802.11 MAC →
//   CCMP encryption → air → RX driver FULLY processes them → LLTF fires.
//
// WHY static RX IP (192.168.4.100):
//   We need RX's IP before the first packet can be sent. DHCP assignment fires
//   an event, but static IP lets TX hardcode the destination and start sending
//   the instant STACONNECTED fires — no DHCP race, no lookup needed.
//
// TX AP: 192.168.4.1 (default softAP gateway)
// RX STA: 192.168.4.100 (configured statically on the RX side)
// UDP port: 12345
// =============================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "esp_timer.h"
#include <string.h>

// ---- Configuration ----------------------------------------------------------
#define AP_SSID      "Wi-Sense-TX"
#define AP_PASS      "wisense123"
#define AP_CHANNEL   1
#define TARGET_HZ    100
#define UDP_PORT     12345

// RX uses a static IP of 192.168.4.100 — must match wi_sense_rx.ino
static const IPAddress RX_IP(192, 168, 4, 100);

// ---- Derived ----------------------------------------------------------------
static const uint64_t INTERVAL_US = 1000000ULL / TARGET_HZ;

static WiFiUDP   udp;
static bool      rxPresent  = false;
static uint32_t  pktSeq     = 0;
static uint64_t  nextTxUs   = 0;
static uint32_t  diagCount  = 0;
static uint64_t  diagUs     = 0;
static uint8_t   payload[8];

// =============================================================================
static void onStaConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    rxPresent = true;
    Serial.printf("[TX] RX associated  MAC=%02x:%02x:%02x:%02x:%02x:%02x → UDP ON\n",
        info.wifi_ap_staconnected.mac[0], info.wifi_ap_staconnected.mac[1],
        info.wifi_ap_staconnected.mac[2], info.wifi_ap_staconnected.mac[3],
        info.wifi_ap_staconnected.mac[4], info.wifi_ap_staconnected.mac[5]);
}

// =============================================================================
static void onStaDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    rxPresent = false;
    Serial.println("[TX] RX disconnected → idle");
}

// =============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n[Wi-Sense TX] Booting...");

    WiFi.onEvent(onStaConnected,    ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent(onStaDisconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    // g/n only: forces OFDM rates for all management and data frames.
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_max_tx_power(84);

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    Serial.printf("[Wi-Sense TX] AP MAC : %02x:%02x:%02x:%02x:%02x:%02x\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[Wi-Sense TX] AP IP  : %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[Wi-Sense TX] CH:%d  HT20  g/n-only  %d Hz  RX_IP=%s\n",
                  AP_CHANNEL, TARGET_HZ, RX_IP.toString().c_str());
    Serial.println("[Wi-Sense TX] Waiting for RX to associate...");

    memset(payload + 4, 0xAA, 4);
    nextTxUs = diagUs = (uint64_t)esp_timer_get_time();
}

// =============================================================================
void loop() {
    const uint64_t now = (uint64_t)esp_timer_get_time();

    if (now - diagUs >= 5000000ULL) {
        diagUs = now;
        Serial.printf("[TX] sent=%u  mode=%s  clients=%d\n",
                      diagCount,
                      rxPresent ? "UDP-UNICAST" : "idle",
                      WiFi.softAPgetStationNum());
        diagCount = 0;
    }

    if (!rxPresent) return;

    if (now >= nextTxUs) {
        nextTxUs += INTERVAL_US;
        if (now > nextTxUs + INTERVAL_US * 5ULL) nextTxUs = now + INTERVAL_US;

        pktSeq++;
        diagCount++;
        payload[0] = (uint8_t)(pktSeq >> 24);
        payload[1] = (uint8_t)(pktSeq >> 16);
        payload[2] = (uint8_t)(pktSeq >>  8);
        payload[3] = (uint8_t)(pktSeq      );

        udp.beginPacket(RX_IP, UDP_PORT);
        udp.write(payload, sizeof(payload));
        udp.endPacket();
    }
}
