/*
 * Wi-Sense TX Beacon  v1.0
 * =========================
 * Minimal transmitter sketch for the Wi-Sense standalone system.
 *
 * Role: Connect to the RX board's AP ("Wi-Sense-Net") and blast UDP packets
 *       at 100 Hz.  The RX ESP32 CSI subsystem measures Channel State
 *       Information from every received frame — it needs a steady stream of
 *       Wi-Fi packets to compute CSI at the target rate.
 *
 * Board : ESP32 Dev Module  (esp32:esp32:esp32)
 * Flash to: COM4  (TX / Transmitter board)
 *
 * No configuration needed — joins "Wi-Sense-Net" (open AP created by RX),
 * then sends a compact 32-byte UDP payload to 192.168.4.1:5500 every 10 ms.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ── Config ───────────────────────────────────────────────────────────────────
static const char*   AP_SSID    = "Wi-Sense-Net";   // RX board's AP name
static const char*   AP_PASS    = "";               // open network
static const char*   RX_IP      = "192.168.4.1";   // RX board static IP
static const uint16_t RX_PORT   = 5500;
static const uint32_t INTERVAL_MS = 10;             // 100 Hz

static const int  LED_PIN = 2;  // built-in LED

// ── State ────────────────────────────────────────────────────────────────────
static WiFiUDP    udp;
static uint32_t   seq        = 0;
static uint32_t   last_send  = 0;
static bool       connected  = false;

// 32-byte payload: magic + sequence number + padding
static uint8_t payload[32] = {
    0xC5, 0x11,  // magic "CSI beacon"
    0x00, 0x00, 0x00, 0x00,  // seq (filled at runtime)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};

// ── Helpers ──────────────────────────────────────────────────────────────────
static void blink(int times, int on_ms = 80, int off_ms = 80) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH); delay(on_ms);
        digitalWrite(LED_PIN, LOW);  delay(off_ms);
    }
}

static void connect_wifi() {
    Serial.printf("[TX] Connecting to AP '%s'", AP_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(AP_SSID, AP_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 60) {
        delay(500);
        Serial.print('.');
        tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.printf("\n[TX] Connected!  IP=%s  RSSI=%d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        udp.begin(5501);   // local port (arbitrary)
        blink(3, 150, 100);
    } else {
        Serial.println("\n[TX] Connection failed — will retry in 5 s");
        connected = false;
        blink(10, 50, 50);  // rapid blink = error
    }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(921600);
    delay(400);
    Serial.println("\n=== Wi-Sense TX Beacon v1.0 ===");
    pinMode(LED_PIN, OUTPUT);
    connect_wifi();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // Reconnect if association dropped
    if (WiFi.status() != WL_CONNECTED) {
        if (connected) {
            Serial.println("[TX] Lost connection — reconnecting...");
            connected = false;
        }
        delay(5000);
        connect_wifi();
        return;
    }

    uint32_t now = millis();
    if (now - last_send >= INTERVAL_MS) {
        last_send = now;

        // Write sequence number into payload bytes 2-5
        payload[2] = (uint8_t)(seq >> 24);
        payload[3] = (uint8_t)(seq >> 16);
        payload[4] = (uint8_t)(seq >>  8);
        payload[5] = (uint8_t)(seq);
        seq++;

        udp.beginPacket(RX_IP, RX_PORT);
        udp.write(payload, sizeof(payload));
        udp.endPacket();

        // LED: solid ON while transmitting (fades during any delay)
        digitalWrite(LED_PIN, HIGH);

        // Status every 1000 packets (~10 s)
        if (seq % 1000 == 0) {
            Serial.printf("[TX] Sent %lu packets  RSSI=%d dBm\n",
                          seq, WiFi.RSSI());
        }
    }
}
