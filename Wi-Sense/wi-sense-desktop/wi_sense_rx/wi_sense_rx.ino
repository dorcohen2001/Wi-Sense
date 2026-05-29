// =============================================================================
// Wi-Sense RX — STA connected to TX AP, CSI from UDP unicast frames
// Board: ESP32 Dev Module  |  Core: esp32:esp32:esp32
//
// WHY static IP (192.168.4.100):
//   TX sends UDP to a hardcoded RX_IP. Static IP on this STA guarantees it
//   matches without any DHCP timing dependency.
//
// WHY UDP socket bind:
//   Binding to UDP_PORT absorbs the TX's 100 Hz UDP stream. Without it the
//   lwIP stack would reply with ICMP port-unreachable for every packet, adding
//   noise and generating return traffic that could affect timing.
//
// Output format (one line per CSI callback):
//   {"s":N,"r":-45,"n":-92,"m":"aabbccddeeff","l":128,"d":"<hex>"}
//
// Python decode:
//   raw  = np.frombuffer(bytes.fromhex(obj['d']), dtype=np.int8)
//   csi  = raw[0::2].astype(float) + 1j*raw[1::2].astype(float)
//   csi63 = np.delete(csi, 32)   # drop DC subcarrier → 63 subcarriers
// =============================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

// ---- Configuration ----------------------------------------------------------
#define SERIAL_BAUD        921600
#define TX_SSID            "Wi-Sense-TX"
#define TX_PASS            "wisense123"
#define RECONNECT_MS       3000
#define CSI_BUF_LEN        256
#define QUEUE_DEPTH        64
#define UDP_PORT           12345

// Static IP must match RX_IP in wi_sense_tx.ino
static const IPAddress RX_STATIC_IP (192, 168,   4, 100);
static const IPAddress RX_GATEWAY   (192, 168,   4,   1);
static const IPAddress RX_SUBNET    (255, 255, 255,   0);

// ---- CSI item ---------------------------------------------------------------
typedef struct {
    uint32_t seq;
    int8_t   rssi;
    int8_t   noise_floor;
    uint8_t  mac[6];
    uint16_t len;
    int8_t   buf[CSI_BUF_LEN];
} csi_item_t;

static QueueHandle_t     csiQueue      = NULL;
static WiFiUDP           rxUdp;
static volatile uint32_t gPromiscTotal = 0;
static volatile uint32_t gCsiTotal     = 0;
static uint32_t          rxSeq         = 0;
static bool              gWasConnected = false;
static uint32_t          gLastAttempt  = 0;

// =============================================================================
static void IRAM_ATTR promisc_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    gPromiscTotal = gPromiscTotal + 1;
}

// =============================================================================
static void IRAM_ATTR csi_callback(void *ctx, wifi_csi_info_t *data) {
    if (!data || !data->buf || data->len == 0) return;
    gCsiTotal = gCsiTotal + 1;
    if (!csiQueue) return;

    csi_item_t item;
    rxSeq++;
    item.seq         = rxSeq;
    item.rssi        = data->rx_ctrl.rssi;
    item.noise_floor = data->rx_ctrl.noise_floor;
    item.len         = (uint16_t)((data->len > CSI_BUF_LEN) ? CSI_BUF_LEN : data->len);
    memcpy(item.mac, data->mac, 6);
    memcpy(item.buf, data->buf, item.len);
    xQueueSend(csiQueue, &item, 0);
}

static const char HEX_LUT[] = "0123456789abcdef";

// =============================================================================
static void outputTask(void *pvParameters) {
    csi_item_t item;
    static char outBuf[768];
    TickType_t  lastDiag = xTaskGetTickCount();
    uint32_t    lastPF = 0, lastCF = 0;

    while (true) {
        TickType_t now = xTaskGetTickCount();
        if ((now - lastDiag) >= pdMS_TO_TICKS(2000)) {
            lastDiag = now;
            uint32_t pf = gPromiscTotal, cf = gCsiTotal;
            Serial.printf("[RX] heard=%u csi=%u (last 2s)\n",
                          pf - lastPF, cf - lastCF);
            Serial.flush();
            lastPF = pf; lastCF = cf;
        }
        if (xQueueReceive(csiQueue, &item, pdMS_TO_TICKS(50)) != pdTRUE) continue;

        int pos = snprintf(outBuf, sizeof(outBuf),
            "{\"s\":%u,\"r\":%d,\"n\":%d,"
            "\"m\":\"%02x%02x%02x%02x%02x%02x\","
            "\"l\":%u,\"d\":\"",
            item.seq,
            (int)item.rssi, (int)item.noise_floor,
            item.mac[0], item.mac[1], item.mac[2],
            item.mac[3], item.mac[4], item.mac[5],
            item.len);

        const int maxHex = (int)sizeof(outBuf) - pos - 4;
        const int nBytes = (item.len * 2 <= maxHex) ? item.len : maxHex / 2;
        for (int i = 0; i < nBytes; i++) {
            const uint8_t b = (uint8_t)item.buf[i];
            outBuf[pos++] = HEX_LUT[b >> 4];
            outBuf[pos++] = HEX_LUT[b & 0xF];
        }
        outBuf[pos++] = '"';
        outBuf[pos++] = '}';
        outBuf[pos++] = '\n';
        Serial.write((const uint8_t *)outBuf, pos);
    }
}

// Re-applies power-save and CSI settings that the association handshake resets.
static void armCsi() {
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_csi(true);
    rxUdp.begin(UDP_PORT);   // absorb TX UDP stream — prevents ICMP port-unreachable
}

// =============================================================================
void setup() {
    Serial.setTxBufferSize(8192);
    Serial.begin(SERIAL_BAUD);
    Serial.println("\n[Wi-Sense RX] Booting...");

    csiQueue = xQueueCreate(QUEUE_DEPTH, sizeof(csi_item_t));
    if (!csiQueue) { Serial.println("[RX] FATAL: queue"); while(1) vTaskDelay(portMAX_DELAY); }
    xTaskCreatePinnedToCore(outputTask, "csi_out", 8192, NULL, 10, NULL, 1);

    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);

    // Static IP — must match RX_IP in wi_sense_tx.ino
    WiFi.config(RX_STATIC_IP, RX_GATEWAY, RX_SUBNET);

    // Disable power-save before any connection attempt.
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Promiscuous — diagnostic counter only.
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promisc_rx_cb);

    // CSI config — enable both LLTF (legacy OFDM) and HTLTF (802.11n).
    wifi_csi_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.lltf_en           = true;
    cfg.htltf_en          = true;
    cfg.stbc_htltf2_en    = false;
    cfg.ltf_merge_en      = true;
    cfg.channel_filter_en = false;
    cfg.manu_scale        = false;
    cfg.shift             = 0;

    esp_err_t e;
    e = esp_wifi_set_csi_config(&cfg);
    Serial.printf("[RX] csi_config : %s\n", esp_err_to_name(e));
    e = esp_wifi_set_csi_rx_cb(csi_callback, NULL);
    Serial.printf("[RX] csi_rx_cb  : %s\n", esp_err_to_name(e));
    e = esp_wifi_set_csi(true);
    Serial.printf("[RX] csi(true)  : %s\n", esp_err_to_name(e));

    Serial.printf("[RX] Static IP: %s\n", RX_STATIC_IP.toString().c_str());

    gLastAttempt = millis();
    WiFi.begin(TX_SSID, TX_PASS);
    Serial.printf("[RX] Connecting to '%s'...\n", TX_SSID);
}

// =============================================================================
void loop() {
    bool connected = (WiFi.status() == WL_CONNECTED);

    if (connected && !gWasConnected) {
        gWasConnected = true;
        armCsi();
        Serial.printf("[RX] Associated  BSSID=%s  CH=%d  RSSI=%d dBm  IP=%s\n",
                      WiFi.BSSIDstr().c_str(), WiFi.channel(), WiFi.RSSI(),
                      WiFi.localIP().toString().c_str());
        Serial.println("[RX] PS=NONE, CSI re-armed, UDP absorber active.");
    }

    if (!connected) {
        if (gWasConnected) {
            gWasConnected = false;
            rxUdp.stop();
            Serial.println("[RX] Lost connection — retrying...");
        }
        uint32_t now = millis();
        if (now - gLastAttempt >= (uint32_t)RECONNECT_MS) {
            gLastAttempt = now;
            WiFi.begin(TX_SSID, TX_PASS);
            Serial.printf("[RX] WiFi.begin('%s')...\n", TX_SSID);
        }
    }

    // Drain any received UDP packets (TX sends 100 Hz; absorb silently).
    while (rxUdp.parsePacket() > 0) rxUdp.flush();

    vTaskDelay(pdMS_TO_TICKS(200));
}
