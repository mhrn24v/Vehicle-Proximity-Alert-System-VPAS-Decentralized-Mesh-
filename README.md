# Vehicle-Proximity-Alert-System-VPAS-Decentralized-Mesh-
a part of V2V communication. Decentralized Inter-vehicle Ranging &amp; Emergency Collision Threat-alert.

A multi-node Vehicle-to-Vehicle (V2V) proximity warning system built on ESP32 microcontrollers. Each vehicle node continuously broadcasts its presence over ESP-NOW, measures its distance to nearby vehicles using WiFi signal strength, and triggers escalating visual and audible alerts as vehicles approach each other.

Built as a scalable, infrastructure-free mesh: no router, no access point, no internet connection required, No pre-establish IDs. Nodes discover each other automatically and communicate directly, radio to radio.

---

## Features

- **Infrastructure-free networking** — ESP-NOW peer-to-peer broadcast, no WiFi router or internet needed
- **Automatic peer discovery + handshake** — nodes find each other and complete a PING/PONG handshake without manual pairing
- **Wireless distance estimation** — RSSI-based ranging between nodes, no additional ranging hardware
- **Three-tier alert system** — neutral / caution / danger zones with distinct LED and buzzer behavior
- **GPS position reporting** — logs exact coordinates when a danger condition persists
- **Concurrent multi-node operation** — every node transmits and receives simultaneously; scales beyond three vehicles
- **Non-blocking architecture** — FreeRTOS tasks pinned across both ESP32 cores, no `delay()` calls stalling the system
- **Packet integrity checking** — CRC8 validation with silent drop of corrupted frames

---

## How It Works

Each node runs the same firmware and performs these steps continuously and independently:

1. **Read GPS position** — parses NMEA sentences from the GPS module for latitude, longitude, speed, and heading
2. **Broadcast position** — sends a position packet to all nodes via ESP-NOW broadcast (10 Hz)
3. **Peer handshake** — on first hearing a new node, initiates a PING; the peer replies with PONG to confirm the link
4. **Measure distance** — converts the WiFi RSSI of received packets into an estimated distance using a log-distance path-loss model
5. **Evaluate zone** — determines whether the nearest peer falls in the neutral, caution, or danger range
6. **Drive indicators** — updates LED and buzzer state, prints live distance to Serial Monitor
7. **Escalate sustained danger** — if the danger zone persists beyond the configured threshold, prints the node's GPS coordinates

Because transmission runs on a dedicated task and reception runs via an interrupt-driven callback, every node sends and listens at the same time. There is no turn-taking or master/slave relationship — all nodes are peers.

### Alert Zones

| Distance to nearest peer | Zone | LED | Buzzer |
|---|---|---|---|
| Greater than 50 m | Neutral | Off | Off |
| 31-49 m | Caution | Slow blink (500 ms) | Off |
| Less than 30 m | Danger | Steady on | Rapid beep (80 ms) |

If a node remains in the danger zone continuously for 20 seconds, it prints its GPS location to the Serial Monitor as an escalated alert. The report fires once per continuous danger streak and resets when the vehicles separate.

---

## Hardware Requirements

Per node (multiply by the number of vehicles):

| Component | Notes |
|---|---|
| ESP32 development board | Any variant; ESP32 Dev Module tested |
| GPS module | NEO-6M or similar, UART/NMEA output |
| Active piezo buzzer | 5V, 2-pin (built-in oscillator) |
| LED | Any standard 5mm LED |
| 220 Ω resistor | Current limiting for the LED |
| Jumper wires, breadboard | — |
| 5V power supply | USB adapter or battery; see Troubleshooting |

---

## Wiring

| Module | Module Pin | ESP32 Pin |
|---|---|---|
| GPS | TX | GPIO 16 (ESP32 RX) |
| GPS | RX | GPIO 17 (ESP32 TX) |
| GPS | VCC | 3.3V or 5V (check your module) |
| GPS | GND | GND |
| Buzzer | + | GPIO 19 |
| Buzzer | − | GND |
| LED | Anode (via 220 Ω) | GPIO 21 |
| LED | Cathode | GND |

**Notes:**
- GPS connections are **crossed** — the module's TX goes to the ESP32's RX pin and vice versa.
- All components must share a **common ground** with the ESP32.
- If your buzzer draws more than roughly 12 mA, drive it through an NPN transistor rather than directly from the GPIO pin.

---

## Software Setup

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or 2.x)
- ESP32 board support package — install via **Tools → Board → Boards Manager**, search for "esp32" by Espressif Systems (version 2.0.5 or later required for RSSI access)
- **TinyGPSPlus** library by Mikal Hart — install via **Sketch → Include Library → Manage Libraries**

### Flashing

1. Clone or download this repository.
2. Open `V2V_CollisionAvoidance.ino` in the Arduino IDE. The folder name must match the `.ino` filename.
3. Select your board under **Tools → Board** (e.g. ESP32 Dev Module) and the correct COM port.
4. **Set a unique node ID for each board.** Change this line near the top of the sketch:
   ```cpp
   #define NODE_ID  0    // use 0 for the first board, 1 for the second, 2 for the third...
   ```
   Everything else stays identical across all boards.
5. Upload, then open the Serial Monitor at **115200 baud**.

Repeat for each vehicle, incrementing `NODE_ID` each time.

---

## Configuration

All tunable parameters live in the `USER CONFIG` block at the top of the sketch:

```cpp#define NODE_ID             []     //[1,2,3,..] unique per board
#define WIFI_CHANNEL             1     // must match on all nodes
#define MAX_NODES               20     // peer table size

#define TX_POWER_AT_1M       -40.0f    // RSSI measured at 1 meter (calibrate this)
#define PATH_LOSS_EXPONENT     2.5f    // 2.0 open space, 3.0–4.0 indoors

#define SAFE_DISTANCE_M       120.0f    // neutral zone threshold
#define DANGER_DISTANCE_M      30.0f    // danger zone threshold

#define CAUTION_BLINK_MS       500     // LED blink rate in caution zone
#define DANGER_BEEP_MS          80     // buzzer beep rate in danger zone
#define SUSTAINED_DANGER_MS  20000     // time in danger before GPS report

### Calibration

For meaningful distance readings, calibrate `TX_POWER_AT_1M` for your specific hardware:

1. Flash two boards and place them exactly 1 meter apart.
2. Read the RSSI value printed in the Serial Monitor.
3. Set `TX_POWER_AT_1M` to that value and re-flash both boards.
```

Adjust `PATH_LOSS_EXPONENT` based on environment — lower for open outdoor line-of-sight, higher for indoor or obstructed testing.

---

## Example Output

```Booting V2V node, NODE_ID = 0
All tasks started.
[NODE 2] WiFi handshake complete with NODE 1 (RSSI -52 dBm)
[NODE 2] Distance to NODE 1: 225.24 m (RSSI -52 dBm, handshake OK)
[NODE 2] Distance to NODE 1: 180.91 m (RSSI -52 dBm, handshake OK)
[NODE 2] Distance to NODE 1: 100.15 m (RSSI -58 dBm, handshake OK)
[NODE 2] Distance to NODE 1: 50.80 m (RSSI -59 dBm, handshake OK)
[NODE 2] Distance to NODE 1: 30.80 m (RSSI -59 dBm, handshake OK)
[NODE 2] Distance to NODE 1: 5.80 m (RSSI -60 dBm, handshake OK)
[NODE 2] *** SUSTAINED DANGER (20s) with NODE 1 - GPS LOCATION: lat=20.267100 lon=85.842000 ***
```

---

## Architecture

The firmware uses FreeRTOS tasks distributed across both ESP32 cores:

```
Core 1                                Core 2
┌──────────────────────┐            ┌────────────────────────┐
│ Broadcast Task       │            │ GPS Task               │
│ (ESP-NOW TX @ 10 Hz) │            │ (UART NMEA parsing)    │
└──────────┬───────────┘            │                        │
           │                        │ Distance + Indicator   │
┌──────────▼───────────┐            │ Task (RSSI, zones,     │
│ ESP-NOW RX Callback  │            │ LED/buzzer, Serial)    │
│ (interrupt-driven)   │            └───────────┬────────────┘
└──────────┬───────────┘                        │
           └────────────┬───────────────────────┘
                        ▼
          ┌──────────────────────────────┐
          │ Shared State (mutex-guarded) │
          │ own_state, peer_table[]      │
          └──────────────────────────────┘
```

All shared state is protected by a FreeRTOS mutex, since the ESP-NOW receive callback executes in the WiFi task context and can preempt the application tasks at any time.

### Packet Format

```cpp
typedef struct {
  uint8_t  node_id;       // sender identity
  uint8_t  packet_type;   // POSITION, PING, or PONG
  uint32_t seq_num;       // rolling counter, detects packet loss
  uint32_t timestamp_ms;  // sender's millis() at transmit
  float    lat, lon;      // GPS coordinates
  float    speed_mps;     // ground speed
  float    heading_deg;   // course over ground
  uint8_t  crc8;          // integrity check
} V2VPacket;
```

Packets are fixed-size and byte-packed, well under the 250-byte ESP-NOW payload limit.

---

## Scaling Beyond Three Nodes

The architecture is not limited to three vehicles:

- Broadcasting to `FF:FF:FF:FF:FF:FF` means new nodes join automatically with no reconfiguration of existing nodes.
- Increase `MAX_NODES` to expand the peer table.
- At the default 10 Hz broadcast rate, total airtime usage remains low; the WiFi layer's built-in CSMA/CA handles contention.
- Beyond roughly 15–20 nodes in mutual range, consider adding TDMA-style broadcast slotting (offsetting each node's transmit phase by `node_id × period / N`) to bound worst-case latency.

---

## Limitations

**RSSI-based distance is approximate.** WiFi signal strength is affected by antenna orientation, obstacles, multipath reflections, and interference. Readings should be treated as a rough proximity indicator rather than precise measurement — expect several meters of variance, and some flicker between zones near threshold boundaries. Centimeter-accurate wireless ranging requires dedicated UWB hardware such as DWM1000/DWM3000 modules.

**GPS accuracy.** Consumer GPS modules are typically accurate to ±2–5 meters and require clear sky view. Cold-start fixes can take 30 seconds to several minutes, and indoor operation is generally unreliable.

**This is a prototype.** It is not a certified safety system and should not be relied upon as the sole collision-avoidance mechanism in any real vehicle.

---

## Troubleshooting

**`Brownout detector was triggered`**
The ESP32's supply voltage dipped below its safety threshold, usually during a WiFi transmission current spike. Use a short, thick USB cable and a proper 5V/1A+ power adapter rather than a laptop USB port. Adding a 470–1000 µF capacitor across the 5V and GND rails near the board resolves this in most cases. Avoid powering the buzzer and LED through the board's onboard 3.3V regulator.

**Compile error mentioning `AlertLevel` or a type "not declared in this scope"**
The Arduino IDE auto-generates function prototypes and inserts them near the top of the file, before user type definitions appear. Move any `enum` or `typedef` used in a function signature to the very top of the sketch, above all functions.

**Compile error mentioning `esp_now_recv_info_t` or `rx_ctrl`**
Your ESP32 board package is out of date. Update "esp32 by Espressif Systems" to version 2.0.5 or later via Boards Manager.

**GPS shows `lat=0.000000 lon=0.000000`**
The module has not acquired a satellite fix. Move outdoors with clear sky view and allow several minutes for a cold start. If satellite count stays at zero, verify TX/RX are not swapped and that the baud rate matches your module.

**Nodes not discovering each other**
Confirm all boards use the same `WIFI_CHANNEL` value and that each has a unique `NODE_ID`. Verify both boards are powered and within range.

**Serial Monitor shows nothing or garbled text**
Set the baud rate to 115200.

---

## Repository Structure

```
.
├── V2V_CollisionAvoidance/
│   └── V2V_CollisionAvoidance.ino    # Main firmware — flash to every node
├── Test_GPSModule/
│   └── Test_GPSModule.ino            # Standalone GPS test sketch
└── README.md
```

Standalone test sketches are included to verify individual modules before running the full system. Flashing and confirming each sensor separately makes hardware faults much faster to isolate.

---

## License

MIT License — free to use, modify, and distribute.
