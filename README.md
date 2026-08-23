# ZPS_Zigbee_MAVLink_Node
![ESP32-C6](https://img.shields.io/badge/ESP32--C6-Tested-success)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.3.2-blue)
![License](https://img.shields.io/github/license/Supreme-Soumya/ZPS_Zigbee_MAVLink_Node)

A Zigbee based WSN node that relays live GPS telemetry from an ArduPilot APM 2.8.0 flight controller over MAVLink

### Motivation
If you tried to make a project with zigbee outside the built-in examples from arduino ide or esp ide, you know it is frustratingly cumbersome. I went through it and hence sharing some of my experience, so you don't have to go through same hassle.

---

**This project makes use of ESP32-C6-Devkitc-1-N8 modules as the coordinator and the end device nodes. To flash these boards, I used ESP IDE with ESP IDF version 5.3.2 (For some reason they removed some of the zigbee stack featureset in later versions). I faced many compilation issue while doing this on Windows, so eventually installed Ubuntu 24.04.4 and there downloaded ESP IDF 5.3.2, then this worked. So if you are facing errors while compiling, the IDF version could be the issue.**

---

### Project Description
1. This is a variant of [`ZPS_ZIgbee_GPS_Node`](https://github.com/Supreme-Soumya/ZPS_ZIgbee_GPS_Node) — instead of reading a GPS module directly, the sensor node (`MAVLink_end_device`) taps into the **TELEM port of an ArduPilot APM 2.8.0** flight controller and parses its live MAVLink v1 telemetry stream on the ESP32-C6 itself, with a from-scratch parser (no external MAVLink library).
2. The flight controller flies using its own attached GPS, and simultaneously streams that same GPS + navigation data out over MAVLink — the sensor node listens in on this stream rather than reading a GPS chip on its own UART.
3. It extracts position, altitude, ground speed, heading, satellite count, HDOP and UTC time out of four MAVLink messages, packages them into the same compact JSON format used in the GPS-node project, and writes it to the coordinator over a custom Zigbee cluster every second.
4. The coordinator receives this ZCL write-attribute command, parses the JSON payload, and prints a formatted GPS update block to the serial monitor — identical to the original project.

---

### Block Diagram
<p align="center">
  <img src="photos/Block_Diagram.png" width="400">
</p>

> placeholder

---

### Repository Structure
```text
ZPS_Zigbee_MAVLink_Node/
├── documentations/               # Hardware photos / diagrams
├── src/
│   ├── MAVLink_coordinator/      # Coordinator firmware — receives + logs GPS data
│   │   ├── coordinator.c
│   │   └── coordinator.h
│   └── MAVLink_end_device/       # Sensor node firmware — parses MAVLink, sends over Zigbee
│       ├── end_device.c
│       ├── end_device.h
│       ├── mavlink_reader.c
│       └── mavlink_reader.h
├── LICENSE
└── README.md
```

---

### Software Requirements
- ESP-IDF v5.3.2
- ESP IDE
- ESP Zigbee SDK

---

### Hardware Requirements
- 2x ESP32-C6-DevKitC-1-N8 (one coordinator, one sensor node)
- 1x ArduPilot flight controller — tested on **APM 2.8.0**
- 1x GPS module wired to the flight controller itself (not to the ESP32-C6) — e.g. a NEO-6M/7M on the APM's GPS port

---

### Wiring
#### Sensor Node Side (`MAVLink_end_device`)

> **Note**
> There's no GPS module wired directly to this board. The sensor node instead taps into the APM 2.8's **TELEM port** over UART1 and reads the MAVLink stream the flight controller is already producing.

**APM 2.8 TELEM Port → ESP32-C6**

| APM 2.8 TELEM Pin | ESP32-C6 Pin |
|--------------------|--------------|
| GND                | GND |
| TELEM TX           | GPIO6 (UART1 RX) |
| TELEM RX           | GPIO7 (UART1 TX) |

UART baud rate: `57600` (APM 2.8 default TELEM baud). The TELEM port runs at 3.3V logic, so no level shifter is needed against the ESP32-C6. TX from the ESP32 (GPIO7) is only needed if you plan to send MAVLink commands upstream — this project only reads telemetry, so it's optional to wire.

---

### Pre-flashing Instructions
1. Before clicking build, make sure you have chosen the correct board. It is best practice to make sure your com port detects the chip automatically through the inbuilt detect option while you select the board.
2. After you have used this code, update the `Cmakelists.txt` within the main folder with the name of the .c files you are saving as.
   It should be like this — for the coordinator:
  ```cmake
	idf_component_register(
	  SRCS
		  "coordinator.c"
      INCLUDE_DIRS
      	"."
	)
  ```
   and for the sensor node:
  ```cmake
	idf_component_register(
	  SRCS
		  "end_device.c"
		  "mavlink_reader.c"
      INCLUDE_DIRS
      	"."
	)
  ```
3. While flashing, before you connect the usb cable to the ESP board, press and hold the boot button, connect the cable then release, makes the board enter boot mode.
4. If you have already tried flashing other zigbee firmwares before and want to start over again, you should erase the chip's old firmware before uploading new, cause even if you change firmware, the zigbee defined pan id and address remains unchanged. I like to do it by going in `eim > Open Dashboard > Open IDF Terminal (1st Option)` then type `esptool.py --chip YOUR_CHIP --port YOUR_PORT erase_flash`.
   For Windows, the com port would be like `COM5`, for Linux it would be of the format `/dev/ttyUSB0`

---

### Pairing Procedure
1. Flash the coordinator first
2. Flash the sensor node (Don't flash the codes from separate PCs, doing so assigns them separate addresses, hence pairing fails)
3. Turn on the coordinator first, wait a few seconds to let it form the network.
4. Power up the flight controller so it starts streaming MAVLink over its TELEM port, then turn on the sensor node next.
5. Check serial monitor to see if the sensor node joined the network.

---

### Expected Serial Output

**Sensor node**, right after boot, before it even joins the Zigbee network:

```text
I (612) MAV_READER: MAVLink UART started (RX=GPIO6 57600 baud)
I (614) MAV_READER: MAVLink reader task started
I (620) ED: Initialising Zigbee stack (End Device)
```

Once it joins and the flight controller is streaming telemetry:

```text
I (5320) ED: Joined network! Short addr=0xA1B2 PAN=0x6e19 CH=26
I (6320) ED: Sending GPS JSON (131 bytes): {"a":22.572646,"o":88.363895,"e":45.2,"s":0.0,"c":0.0,"h":0.0,"n":9,"d":1.10,"f":1,"t":"2024-07-15T08:30:00"}
```

**Coordinator**, once the sensor node's first packet arrives:

```text
I (512) COORDINATOR: Initialising Zigbee stack
I (1132) COORDINATOR: Network formed PAN:0x6e19 CH:26 Addr:0x0000
I (1142) COORDINATOR: Network steering started — waiting for devices
I (5312) COORDINATOR: Device joined: short=0xA1B2
I (6322) COORDINATOR: ┌─── GPS Update ──────────────────────────
I (6322) COORDINATOR: │ Fix valid   : YES
I (6322) COORDINATOR: │ UTC         : 2024-07-15T08:30:00
I (6322) COORDINATOR: │ Lat / Lon   : 22.572646, 88.363895
I (6322) COORDINATOR: │ Altitude    : 45.2 m
I (6322) COORDINATOR: │ Speed       : 0.0 km/h
I (6322) COORDINATOR: │ Course      : 0.0 °
I (6322) COORDINATOR: │ Heading     : 0.0 °
I (6322) COORDINATOR: │ Satellites  : 9
I (6322) COORDINATOR: │ HDOP        : 1.10
I (6322) COORDINATOR: └─────────────────────────────────────────
```

> **Note:** Heading/course stay at `0.0°` until the flight controller sends a valid `GLOBAL_POSITION_INT` or `VFR_HUD` message — this usually needs the APM to have a GPS lock and, for heading in particular, to be moving or armed.

---

### MAVLink Messages Parsed

The end device runs a **from-scratch MAVLink v1 parser** — no `mavlink-c-library-v2` dependency. It validates each frame's CRC (CRC-16/MCRF4XX with a per-message CRC-EXTRA byte) before accepting it, and writes results directly into a mutex-protected snapshot struct that `end_device.c` reads from every second.

| Msg ID | Name                  | Data extracted |
|--------|------------------------|-----------------|
| 2      | `SYSTEM_TIME`          | UTC timestamp |
| 24     | `GPS_RAW_INT`          | Fix type, lat, lon, altitude, HDOP, satellite count |
| 33     | `GLOBAL_POSITION_INT`  | Lat, lon, altitude, ground speed, heading |
| 74     | `VFR_HUD`              | Ground speed, heading (fallback source) |

---

### GPS Payload Format

Same custom Zigbee cluster and compact JSON schema as the original GPS-node project (`0xFF00`, attribute `0x0001`), so the coordinator firmware didn't need to change at all.

```json
{"a":22.572646,"o":88.363895,"e":45.2,"s":0.0,"c":0.0,"h":0.0,"n":9,"d":1.10,"f":1,"t":"2024-07-15T08:30:00"}
```

| Field | Meaning                          | MAVLink source |
|-------|------------------------------------|------------------|
| `a`   | Latitude (degrees, +N/-S)          | `GPS_RAW_INT` / `GLOBAL_POSITION_INT` |
| `o`   | Longitude (degrees, +E/-W)         | `GPS_RAW_INT` / `GLOBAL_POSITION_INT` |
| `e`   | Altitude (metres)                  | `GPS_RAW_INT` / `GLOBAL_POSITION_INT` |
| `s`   | Ground speed (km/h)                | `GLOBAL_POSITION_INT` / `VFR_HUD` |
| `c`   | Course (degrees)                   | Mirrors `h` — no separate GPS ground track is parsed |
| `h`   | Heading (degrees)                  | `GLOBAL_POSITION_INT` / `VFR_HUD` |
| `n`   | Satellites in use                  | `GPS_RAW_INT` |
| `d`   | HDOP                                | `GPS_RAW_INT` |
| `f`   | 1 = 3-D fix or better, 0 = no fix   | `GPS_RAW_INT` |
| `t`   | Timestamp (`YYYY-MM-DDTHH:MM:SS`)  | `SYSTEM_TIME` |

> **Important:** `CUSTOM_CLUSTER_ID`, `CUSTOM_ATTR_JSON_ID` and `CUSTOM_JSON_MAX_LEN` must match exactly between `coordinator.h` and `end_device.h`, or the coordinator will silently reject incoming packets.

---

### Troubleshooting
| Problem                              | Solution |
|----------------------------------------|-----------|
| Doesn't compile                        | Use ESP-IDF 5.3.2 |
| All GPS fields stay at 0                | Confirm the APM's TELEM port is actually streaming (check the serial protocol setting in Mission Planner), verify baud is 57600, check TX/RX aren't swapped |
| Heading/course stuck at 0° or -1       | APM hasn't sent a valid `GLOBAL_POSITION_INT`/`VFR_HUD` yet — needs a GPS lock, and heading is often only meaningful once armed/moving |
| Frames rarely getting through           | CRC mismatches from noisy wiring — keep the TELEM wires short, check grounds are common between APM and ESP32-C6 |
| Sensor node won't join                  | Erase flash on both boards, verify same channel mask |
| No serial output on coordinator         | Confirm sensor node joined ("Device joined"), verify `CUSTOM_JSON_MAX_LEN` matches between `coordinator.h` and `end_device.h` |

---

### Future Improvements
- [ ] MAVLink v2 framing support (currently v1 only)
- [ ] Parse additional messages (`BATTERY_STATUS`, `SYS_STATUS`) for flight-controller health monitoring
- [ ] Support for multiple sensor nodes reporting to a single coordinator
- [ ] On-board logging of received telemetry (SD card / flash)
- [ ] Integration with MQTT and cloud/dashboard platforms through a Zigbee gateway
- [ ] Real-time position visualization on a map
- [ ] Automatic network rejoin after power loss or communication failure
- [ ] Over-the-Air (OTA) firmware update support
- [ ] Encrypted Zigbee communication and enhanced network security
- [ ] PCB design for a compact and deployable hardware module

---

### References
- ESP-IDF Documentation
- ESP Zigbee SDK Documentation
- Espressif ESP32-C6 Technical Reference Manual
- ArduPilot APM 2.8.0 Documentation
- MAVLink v1 Message Definitions (`common.xml`)
- MAVLink Serial Protocol Reference

---

### License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
