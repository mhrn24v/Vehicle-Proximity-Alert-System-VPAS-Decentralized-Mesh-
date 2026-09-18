/*
  ============================================================================
  V2V Collision Avoidance - ESP32 Node Firmware (WiFi/RSSI-based distance)
  ============================================================================
  Protocol: ESP-NOW (broadcast + unicast handshake)
  Sensors:  GPS (TinyGPS++) - used only for sustained-danger location reporting

  HOW THIS WORKS:
    1. Every node broadcasts a small position packet over ESP-NOW (this also
       acts as a "beacon" so other nodes can discover it).
    2. When node A first hears node B's broadcast, node A adds B as a direct
       (unicast) ESP-NOW peer and sends a PING packet directly to B.
    3. Node B replies with a PONG. Once this round-trip completes, the
       WiFi "handshake" between A and B is considered established.
    4. Every packet received (broadcast, PING, or PONG) carries a WiFi
       signal-strength value (RSSI) automatically - no extra hardware
       needed. RSSI is converted into an estimated distance using a
       standard log-distance path-loss formula.
    5. The estimated distance drives the buzzer/LED indicator (3 zones)
       and is printed live to Serial Monitor.
    6. If a node stays continuously inside the DANGER zone for more than
       SUSTAINED_DANGER_MS (20 seconds), it prints its own GPS coordinates
       to Serial Monitor as an escalated alert.

  IMPORTANT ACCURACY NOTE:
    WiFi RSSI-based "distance" is a rough proximity estimate, not a
    precise measurement. Signal strength is affected by antenna
    orientation, obstacles, reflections, and interference - treat this
    as "roughly near / roughly far", not centimeter-accurate ranging.
    True precise wireless ranging needs dedicated UWB hardware (e.g.
    DWM1000/DWM3000 modules), which is a different piece of hardware
    entirely from what this project currently uses.

  CALIBRATION:
    The constants TX_POWER_AT_1M and PATH_LOSS_EXPONENT below are
    approximate defaults. For better real-world accuracy, place two
    boards exactly 1 meter apart, read the printed RSSI value, and set
    TX_POWER_AT_1M to that value. Increase PATH_LOSS_EXPONENT if you're
    testing indoors or around obstacles (try 3.0-4.0), decrease it for
    open outdoor line-of-sight (try 2.0-2.5).

  REQUIRED LIBRARY (install via Library Manager):
    - TinyGPSPlus by Mikal Hart

  REQUIRED ESP32 ARDUINO CORE:
    - This uses esp_now_recv_info_t with rx_ctrl->rssi, which needs a
      reasonably recent ESP32 Arduino core (2.0.5+). If you get compile
      errors about rx_ctrl or esp_now_recv_info_t, update the ESP32
      board package via Boards Manager.

  HOW TO FLASH MULTIPLE VEHICLES:
    1. Change NODE_ID below to 0, 1, 2... for each board.
    2. All boards must use the SAME WIFI_CHANNEL.
    3. Flash identical code otherwise - only NODE_ID changes per vehicle.

  WIRING:
    GPS:     TX -> GPIO 16 (ESP32 RX),  RX -> GPIO 17 (ESP32 TX)
    Buzzer:  +  -> GPIO 19,  -  -> GND
    LED:     anode -> GPIO 21 -> [220 ohm resistor] -> LED -> GND
  ============================================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TinyGPSPlus.h>

// ============================== USER CONFIG ================================
#define NODE_ID           1        // <-- CHANGE THIS: 0, 1, 2... per vehicle
#define WIFI_CHANNEL      1        // must match across all nodes
#define MAX_NODES         20       // scalable peer table size

#define GPS_RX_PIN        16       // ESP32 pin receiving GPS TX
#define GPS_TX_PIN        17       // ESP32 pin transmitting to GPS RX
#define GPS_BAUD          9600

#define BROADCAST_PERIOD_MS   100  // 10 Hz base rate
#define PEER_STALE_MS         1000 // ignore peer data older than this

// Packet types (plain #define, not enum, to sidestep the Arduino IDE's
// auto-prototype-generation ordering issue we hit earlier in this project)
#define PKT_POSITION 0
#define PKT_PING     1
#define PKT_PONG     2

// -------- RSSI -> distance calibration (see CALIBRATION note above) --------
#define TX_POWER_AT_1M      -40.0f  // dBm, RSSI expected at 1 meter
#define PATH_LOSS_EXPONENT    2.5f  // 2.0=open space, 3-4=indoors/obstructed

// -------- Buzzer + LED proximity indicator (WiFi RSSI-based distance) ------
#define BUZZER_PIN            19   // active buzzer, + here, - to GND
#define LED_PIN               21   // LED anode here (through 220 ohm resistor)

#define SAFE_DISTANCE_M       120.0f  // > this  -> neutral (LED off, buzzer off)
#define DANGER_DISTANCE_M      30.0f  // < this  -> danger  (LED on, buzzer rapid)
// between DANGER_DISTANCE_M and SAFE_DISTANCE_M -> caution (LED slow blink)

#define CAUTION_BLINK_MS       500  // LED slow-blink half-period in caution zone
#define DANGER_BEEP_MS           80  // buzzer rapid-beep half-period in danger zone
#define SERIAL_PRINT_PERIOD_MS 1000 // how often to print distance to Serial Monitor
#define SUSTAINED_DANGER_MS   20000 // continuous time in danger zone before GPS report
// ============================================================================

#pragma pack(push, 1)
typedef struct {
  uint8_t  node_id;
  uint8_t  packet_type;   // PKT_POSITION, PKT_PING, or PKT_PONG
  uint32_t seq_num;
  uint32_t timestamp_ms;
  float    lat;
  float    lon;
  float    speed_mps;
  float    heading_deg;
  uint8_t  crc8;
} V2VPacket;
#pragma pack(pop)

// ------------------------------ Globals ------------------------------------
V2VPacket        own_state = {0};
V2VPacket        peer_table[MAX_NODES];
uint32_t         peer_last_seen[MAX_NODES]      = {0};
int8_t           peer_rssi[MAX_NODES]           = {0};
bool             peer_handshake_done[MAX_NODES] = {false};
SemaphoreHandle_t stateMutex;

uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // UART1

// ------------------------------ CRC8 ----------------------------------------
uint8_t calcCRC8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

// ------------------------------ RSSI -> distance -----------------------------
float rssiToDistanceMeters(int8_t rssi) {
  return pow(10.0f, (TX_POWER_AT_1M - (float)rssi) / (10.0f * PATH_LOSS_EXPONENT));
}

// ------------------------------ ESP-NOW helpers ------------------------------
void ensureUnicastPeer(const uint8_t *mac) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t info = {};
    memcpy(info.peer_addr, mac, 6);
    info.channel = WIFI_CHANNEL;
    info.encrypt = false;
    esp_now_add_peer(&info);
  }
}

void sendPacket(const uint8_t *destMac, uint8_t type) {
  V2VPacket pkt;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  pkt = own_state;
  xSemaphoreGive(stateMutex);

  pkt.node_id = NODE_ID;
  pkt.packet_type = type;
  pkt.timestamp_ms = millis();
  pkt.crc8 = calcCRC8((uint8_t*)&pkt, sizeof(pkt) - 1);

  esp_now_send(destMac, (uint8_t*)&pkt, sizeof(pkt));
}

// ------------------------------ ESP-NOW receive callback --------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(V2VPacket)) return;

  V2VPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (calcCRC8((uint8_t*)&pkt, sizeof(pkt) - 1) != pkt.crc8) return; // corrupted
  if (pkt.node_id >= MAX_NODES || pkt.node_id == NODE_ID) return;

  int8_t rssi = 0;
  if (info->rx_ctrl) rssi = info->rx_ctrl->rssi;

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  peer_rssi[pkt.node_id] = rssi;
  peer_last_seen[pkt.node_id] = millis();
  if (pkt.packet_type == PKT_POSITION) {
    peer_table[pkt.node_id] = pkt;
  }
  xSemaphoreGive(stateMutex);

  // ---- Handshake logic ----
  bool alreadyShookHands = peer_handshake_done[pkt.node_id];

  if (pkt.packet_type == PKT_POSITION && !alreadyShookHands) {
    // First time hearing this peer's beacon -> initiate handshake
    ensureUnicastPeer(info->src_addr);
    sendPacket(info->src_addr, PKT_PING);
  }

  if (pkt.packet_type == PKT_PING) {
    // Someone pinged us directly -> reply to complete their handshake
    ensureUnicastPeer(info->src_addr);
    sendPacket(info->src_addr, PKT_PONG);
  }

  if (pkt.packet_type == PKT_PONG) {
    peer_handshake_done[pkt.node_id] = true;
    Serial.printf("[NODE %d] WiFi handshake complete with NODE %d (RSSI %d dBm)\n",
                  NODE_ID, pkt.node_id, rssi);
  }
}

// ------------------------------ ESP-NOW setup --------------------------------
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, restarting...");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(broadcastAddr)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add broadcast peer");
    }
  }
}

// ------------------------------ Task: GPS ------------------------------------
// GPS is kept only for sustained-danger location reporting, not for distance.
void gpsTask(void *pv) {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  for (;;) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated() && gps.location.isValid()) {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      own_state.lat = gps.location.lat();
      own_state.lon = gps.location.lng();
      if (gps.speed.isValid())  own_state.speed_mps  = gps.speed.mps();
      if (gps.course.isValid()) own_state.heading_deg = gps.course.deg();
      xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ------------------------------ Task: Broadcast (TX) -------------------------
void broadcastTask(void *pv) {
  for (;;) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    own_state.node_id = NODE_ID;
    own_state.packet_type = PKT_POSITION;
    own_state.seq_num++;
    own_state.timestamp_ms = millis();
    own_state.crc8 = calcCRC8((uint8_t*)&own_state, sizeof(own_state) - 1);
    V2VPacket toSend = own_state;
    xSemaphoreGive(stateMutex);

    esp_err_t result = esp_now_send(broadcastAddr, (uint8_t*)&toSend, sizeof(toSend));
    if (result != ESP_OK) {
      Serial.println("ESP-NOW send failed");
    }

    int jitter = random(-15, 15);
    vTaskDelay(pdMS_TO_TICKS(BROADCAST_PERIOD_MS + jitter));
  }
}

// ------------------------------ Task: Distance + Indicator -------------------
// Uses WiFi RSSI (from ESP-NOW packets) to estimate distance to the nearest
// peer, prints it to Serial Monitor, drives the buzzer/LED, and reports GPS
// location if the danger zone is sustained for more than 20 seconds.
void distanceIndicatorTask(void *pv) {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  bool blinkState = false;
  uint32_t lastToggle = 0;
  uint32_t lastPrint = 0;
  uint32_t dangerStartMs = 0;       // 0 = not currently in a continuous danger streak
  bool sustainedDangerReported = false;

  for (;;) {
    uint32_t now = millis();

    float nearestDist = -1.0f;
    int   nearestId = -1;
    int8_t nearestRssi = 0;
    bool  nearestHandshakeOk = false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    for (int i = 0; i < MAX_NODES; i++) {
      if (i == NODE_ID) continue;
      if (peer_last_seen[i] == 0) continue;
      if (now - peer_last_seen[i] > PEER_STALE_MS) continue; // stale, skip

      float d = rssiToDistanceMeters(peer_rssi[i]);
      if (nearestDist < 0 || d < nearestDist) {
        nearestDist = d;
        nearestId = i;
        nearestRssi = peer_rssi[i];
        nearestHandshakeOk = peer_handshake_done[i];
      }
    }
    float myLat = own_state.lat;
    float myLon = own_state.lon;
    xSemaphoreGive(stateMutex);

    // ---- Serial Monitor output, once per second ----
    if (now - lastPrint >= SERIAL_PRINT_PERIOD_MS) {
      lastPrint = now;
      if (nearestDist < 0) {
        Serial.printf("[NODE %d] No peer in WiFi range.\n", NODE_ID);
      } else {
        Serial.printf("[NODE %d] Distance to NODE %d: %.2f m (RSSI %d dBm, handshake %s)\n",
                      NODE_ID, nearestId, nearestDist, nearestRssi,
                      nearestHandshakeOk ? "OK" : "pending");
      }
    }

    // ---- Sustained danger -> GPS location report ----
    bool inDanger = (nearestDist >= 0 && nearestDist < DANGER_DISTANCE_M);

    if (inDanger) {
      if (dangerStartMs == 0) dangerStartMs = now; // just entered danger zone
      uint32_t dangerDuration = now - dangerStartMs;

      if (dangerDuration >= SUSTAINED_DANGER_MS && !sustainedDangerReported) {
        sustainedDangerReported = true;
        Serial.printf("[NODE %d] *** SUSTAINED DANGER (%lus) with NODE %d - GPS LOCATION: lat=%.6f lon=%.6f ***\n",
                      NODE_ID, (unsigned long)(dangerDuration / 1000), nearestId, myLat, myLon);
      }
    } else {
      dangerStartMs = 0;
      sustainedDangerReported = false;
    }

    // ---- Indicator zones ----
    if (nearestDist < 0 || nearestDist > SAFE_DISTANCE_M) {
      // NEUTRAL: no peer, or far apart
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, LOW);

    } else if (nearestDist >= DANGER_DISTANCE_M) {
      // CAUTION: LED slow blink, buzzer stays off
      digitalWrite(BUZZER_PIN, LOW);
      if (now - lastToggle >= CAUTION_BLINK_MS) {
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState ? HIGH : LOW);
        lastToggle = now;
      }

    } else {
      // DANGER: LED steady on, buzzer beeps rapidly
      digitalWrite(LED_PIN, HIGH);
      if (now - lastToggle >= DANGER_BEEP_MS) {
        blinkState = !blinkState;
        digitalWrite(BUZZER_PIN, blinkState ? HIGH : LOW);
        lastToggle = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // fine-grained loop so timing stays accurate
  }
}

// ------------------------------ Setup / Loop ----------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("Booting V2V node, NODE_ID = %d\n", NODE_ID);

  randomSeed(esp_random());

  stateMutex = xSemaphoreCreateMutex();
  memset(&own_state, 0, sizeof(own_state));
  memset(peer_table, 0, sizeof(peer_table));

  setupEspNow();

  xTaskCreatePinnedToCore(gpsTask,               "GPS",       4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(broadcastTask,         "Broadcast", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(distanceIndicatorTask, "Indicator", 4096, NULL, 1, NULL, 1);

  Serial.println("All tasks started.");
}

void loop() {
  // Everything happens in FreeRTOS tasks above.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
