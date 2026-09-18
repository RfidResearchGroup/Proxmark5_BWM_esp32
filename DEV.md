# Proxmark5 BLE/WiFi Module - User Manual

> **Document Version**: Based on firmware commit: 245002f  
> **Applicable Hardware**: ESP32-C2 (ESP8684) BLE+WiFi passthrough module  
> **Device Model**: 0xDA10  
> **Last Updated**: 2026-08-06

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Hardware Information](#2-hardware-information)
3. [Storage Information](#3-storage-information)
4. [UART Communication Protocol](#4-uart-communication-protocol)
5. [BLE Information](#5-ble-information)
6. [WiFi Information](#6-wifi-information)
7. [Passthrough Information](#7-passthrough-information)
8. [OTA Information](#8-ota-information)
9. [Command List](#9-command-list)
10. [Broadcast and Asynchronous Communication](#10-broadcast-and-asynchronous-communication)
11. [Configuration Persistence and Recovery](#11-configuration-persistence-and-recovery)
12. [Host-Side Configuration Flow and Examples](#12-host-side-configuration-flow-and-examples)
13. [Over-the-Air Characteristics](#13-over-the-air-characteristics)

---

## 1. System Architecture

### 1.1 Design Intent

This firmware runs on an ESP32-C2 (ESP8684) module as a BLE/WiFi wireless expansion module for Proxmark5. Its core function is **bidirectional data passthrough**: UART data from the host (Proxmark5) is sent to a peer device over BLE SPP or WiFi (TCP/UDP/MQTT), while wireless data received from the peer is sent back to the host UART.

### 1.2 Module Function Overview

```
                                    ┌─────────────────────┐
                                    │   BLE SPP Server    │  -> Phone/PC BLE client
                                    ├─────────────────────┤
              Host UART  ◄────────► │   WiFi TCP Server   │  -> WiFi TCP client
              (commands+data)       ├─────────────────────┤
                                    │   WiFi TCP Client   │  -> Remote TCP server
                                    ├─────────────────────┤
                                    │   WiFi UDP Server   │  -> WiFi UDP peer
                                    ├─────────────────────┤
                                    │   WiFi UDP Client   │  -> Remote UDP server
                                    ├─────────────────────┤
                                    │   WiFi MQTT Client  │  -> MQTT broker
                                    ├─────────────────────┤
                                    │   WiFi Scanner      │  -> Scan nearby APs
                                    ├─────────────────────┤
                                    │   WiFi SNTP         │  -> NTP time sync
                                    ├─────────────────────┤
                                    │   OTA Firmware Upg. │
                                    └─────────────────────┘
```

### 1.3 Code Module List

| Module Directory | Function |
|----------|------|
| `main/` | Main program entry, command routing, global state management, persistent configuration |
| `app_uart_cmd/` | UART command/response packet I/O, CRC verification, state-machine parsing |
| `app_power/` | Runtime power-save switch: DFS, light sleep, advertising interval |
| `app_uart_log/` | Forward system logs to UART |
| `app_ble_spp/` | BLE SPP (Serial Port Profile) server based on the NimBLE stack |
| `app_wifi_connect/` | WiFi STA connection management (auto-reconnect, event callbacks) |
| `app_wifi_scanner/` | Active/passive WiFi scanning |
| `app_wifi_netif_cfg/` | WiFi network interface configuration (IP/DHCP/hostname) |
| `app_wifi_sntp/` | SNTP time synchronization service |
| `app_tcp_server/` | TCP server (listen port, receive client connections) |
| `app_tcp_client/` | TCP client (connect to a remote server) |
| `app_udp_server/` | UDP server (listen port, supports fixed peer address) |
| `app_udp_client/` | UDP client (connect to a remote server) |
| `app_mqtt_client/` | MQTT client (supports TCP/SSL/WebSocket/PSK) |
| `app_ota_ops/` | OTA firmware update operation wrappers |
| `app_nvs_rw/` | NVS (non-volatile storage) read/write abstraction layer |
| `app_common/` | Common utilities (network helpers, RTOS task management) |

### 1.4 System Startup Flow

```
app_main()
   ├── srand(esp_random())              // Random seed
   ├── app_nvs_flash_init()             // Initialize NVS partition
   ├── app_nvs_flash_load()             // Load persisted configuration (incl. power save, WiFi power-save type)
   ├── app_uart_init()                  // Initialize UART command port (460800 bps)
   ├── app_uart_set_command_callback()  // Register UART command callback
   ├── app_uart_set_baudrate_change_callback() // Register baud-rate change callback
   ├── app_power_init()                 // Apply the power-save switch (DFS, light sleep, adv interval) before BLE starts
   ├── app_log_uart_init()              // Initialize log forwarding module
   ├── app_log_uart_set_tx_callback()   // Register log forwarding callback
   ├── app_ble_init()                   // Initialize BLE module
   ├── app_ble_set_rx_callback()        // Register BLE receive callback
   ├── [If the persisted BLE switch is on]
   │   └── app_ble_start()              // Start BLE and begin advertising
   ├── [If configured for WIFI_FORWARD mode]
   │   ├── wifi_forward_common_init()   // Initialize WiFi forwarding
   │   ├── Apply DHCP/static IP/hostname/MAC configuration
   │   ├── app_wifi_connect_start()     // Start WiFi connection
   │   └── Apply WiFi TX power/inactive time
   └── system_ready = true              // Mark system as ready
```

> **Source**: `main/main.c:3508-3558` (`app_main` function)

### 1.5 WiFi Function Modes

The module supports three WiFi function modes, switched by UART command:

| Enum Value | Mode | Description |
|--------|------|------|
| `WIFI_FUNCTION_MODE_WIFI_DISABLE` (0) | WiFi disabled | Only BLE passthrough is available |
| `WIFI_FUNCTION_MODE_WIFI_SCANNER` (1) | WiFi scanner | Scan nearby APs and report results via broadcast |
| `WIFI_FUNCTION_MODE_WIFI_FORWARD` (2) | WiFi forwarding | Enable WiFi passthrough (TCP/UDP/MQTT) |

> **Source**: `main/main.c:49-53` (`wifi_function_mode_t`)

### 1.6 WiFi Forwarding Protocol Types

When the module is in `WIFI_FUNCTION_MODE_WIFI_FORWARD` mode, the following protocols are available:

| Enum Value | Type | Command Range |
|--------|------|----------|
| `WIFI_FORWARD_TCP_SERVER` (0) | TCP server | 2200 |
| `WIFI_FORWARD_TCP_CLIENT` (1) | TCP client | 2300 |
| `WIFI_FORWARD_UDP_SERVER` (2) | UDP server | 2400 |
| `WIFI_FORWARD_UDP_CLIENT` (3) | UDP client | 2500 |
| `WIFI_FORWARD_MQTT_CLIENT` (4) | MQTT client | 2600 |

> **Source**: `main/main.c:56-63` (`wifi_forward_type_t`)

---

## 2. Hardware Information

### 2.1 Chip Platform

- **Chip Model**: ESP32-C2 (ESP8684)
- **Device Model Identifier**: `0xDA10` (Model ID)
- **WiFi**: 2.4 GHz 802.11b/g/n
- **BLE**: Bluetooth 5.0 LE
- **Flash**: 4 MB (default) or 2 MB (development/no OTA)

> **Source**: `main/main.c:429-433` (`APP_CMD_GET_DEVICE_MODEL` handler)

### 2.2 UART Pin Definitions

UART pins are configured through the ESP-IDF Kconfig system (`sdkconfig`) and are not hardcoded in this repository.

| Signal | Config Item | Description |
|------|--------|------|
| TXD | `CONFIG_UART_SPP_TXP` | Command/passthrough UART transmit pin |
| RXD | `CONFIG_UART_SPP_RXP` | Command/passthrough UART receive pin |
| UART Port Number | `CONFIG_UART_SPP_NUM` | UART peripheral index in use |

> **Source**: `components/app_uart_cmd/app_cmd_uart.h:9-11`

### 2.3 UART Electrical Parameters

| Parameter | Default | Description |
|------|--------|------|
| Baud Rate | 460800 | Can be changed dynamically by command |
| Data Bits | 8 | Fixed |
| Parity | None | Fixed |
| Stop Bits | 1 | Fixed |
| Flow Control | None | Fixed |
| Maximum Baud Rate | `CONFIG_SOC_UART_BITRATE_MAX` | Determined by the chip and cannot be exceeded |

> **Source**: `components/app_uart_cmd/app_cmd_uart.c:333-338` (`app_uart_init`)

### 2.4 Default UART Timing Parameters

| Parameter | Value | Description |
|------|-----|------|
| RX Timeout | 200 ms | Inter-frame receive timeout; resets the state machine on timeout |
| RX Buffer | 4096 bytes | Internal ring buffer of the UART driver |
| TX Buffer | 0 bytes | Synchronous transmit mode; sending blocks until complete |
| Event Queue Depth | 10 | UART event queue length |

> **Source**: `components/app_uart_cmd/app_cmd_uart.h:13-16`

---

## 3. Storage Information

### 3.1 Partition Table (4 MB Flash, Production Configuration)

```
Partition Name   Type    Subtype   Offset      Size       Description
─────────────────────────────────────────────────────────────────────
nvs              data    nvs       (auto)      0x54000    User data (336 KB)
otadata          data    ota       (auto)      0x2000     OTA data (8 KB)
phy_init         data    phy       (auto)      0x1000     PHY calibration (4 KB)
app_a            app     ota_0     (auto)      0x1D0000   Firmware A (1856 KB)
app_b            app     ota_1     (auto)      0x1D0000   Firmware B (1856 KB)
```

Flash layout summary:
- Second-stage bootloader: 32 KB (0x8000)
- Partition table: 4 KB (0x1000)
- nvs: 336 KB (0x54000)
- otadata: 8 KB (0x2000)
- phy_init: 4 KB (0x1000)
- app_a: 1856 KB (0x1D0000)
- app_b: 1856 KB (0x1D0000)
- **Total: 4096 KB = 4 MB**

> **Source**: `partitions.csv`

### 3.2 Partition Table (2 MB Flash, Development Configuration, No OTA)

```
Partition Name   Type    Subtype   Offset      Size       Description
─────────────────────────────────────────────────────────────────────
nvs              data    nvs       (auto)      0x24000    User data (144 KB)
otadata          data    ota       (auto)      0x2000     OTA data (8 KB)
phy_init         data    phy       (auto)      0x1000     PHY calibration (4 KB)
factory          app     factory   (auto)      0x1D0000   Firmware (1856 KB)
```

> **Warning**: The 2 MB partition table is for early development and testing only and has no OTA capability. Production must use the 4 MB configuration.

> **Source**: `partitions_2m_noota_bigapp.csv`

### 3.3 NVS Namespaces and Keys

NVS is used to persist user configuration. The following namespaces and keys are used in the code:

#### Namespace `app_sys` (System Configuration)

| Key | Type | Description |
|------|------|------|
| `timezone` | string | Time zone string (for example, `CST-8`) |
| `pwr_save` | u8 | Power-save switch (0=off, 1=on; default on) |
| `ble_en` | u8 | Persisted BLE on/off switch (0=off, 1=on; default on) |

#### Namespace `app_host` (Host (PM5) Opaque Value Slots)

| Key | Type | Description |
|------|------|------|
| `h<id>` | u64 | Host value slot 0-255 (`APP_CMD_SET/GET_SYS_HOST_VALUE`); low 32 bits = value, bit 32 = "ever set" flag |

#### Namespace `app_wifi` (WiFi Configuration)

| Key | Type | Description |
|------|------|------|
| `wifi_mode` | i8 | WiFi function mode (0/1/2) |
| `wifi_fwd_type` | u8 | WiFi forwarding protocol type (0-4) |
| `wifi_tx_pwr` | i8 | WiFi transmit power (8-80, 0.25 dBm step) |
| `wifi_ps` | u8 | WiFi modem power-save type (0=none, 1=min modem, 2=max modem; default 1) |
| `wifi_inact_tm` | u16 | WiFi inactive timeout (seconds) |
| `wifi_dhcp_en` | u8 | DHCP enable (0/1) |
| `wifi_mac_addr` | blob(6) | WiFi STA MAC address |
| `wifi_ip_addr` | blob(12) | Static IP/gateway/netmask |
| `wifi_host_name` | string | WiFi host name |
| `wifi_sntp_en` | u8 | SNTP enable |
| `wifi_sntp_srv` | string | SNTP server address |
| `wifi_sntp_intv` | u32 | SNTP sync interval (ms) |

#### Namespace `app_ble` (BLE Configuration)

| Key | Type | Description |
|------|------|------|
| `ble_bond_en` | u8 | Bonding enable |
| `ble_bond_key` | u32 | Pairing password (6 digits) |
| `ble_adv_mfg` | blob | Advertising manufacturer data |
| `ble_name` | string | BLE device name |
| `ble_addr` | blob(6) | BLE device address |
| `ble_ntf_nmem` | u16 | Notify no-memory retry limit |
| `ble_ntf_fail` | u16 | Notify send-failure retry limit |
| `ble_txp_adv` | u8 | BLE advertising TX power |
| `ble_txp_conn` | u8 | BLE connection TX power |

#### Namespace `wifi_connect` (WiFi Connection Parameters)

| Key | Type | Description |
|------|------|------|
| `reconn_intvl` | u16 | WiFi reconnect interval (seconds) |

> **Source**: `main/main_settings.c`, `components/app_ble_spp/app_ble_spp.c`, `components/app_wifi_sntp/app_wifi_sntp.c`, `components/app_wifi_connect/app_wifi_connect.c`

---

## 4. UART Communication Protocol

### 4.1 Packet Format

All UART communication uses a unified binary packet format:

```
┌────────┬────────┬──────────────┬──────────────┬─────────────────┬──────────────┐
│ HDR1   │ HDR2   │ CMD/Type     │ Payload Len  │ Payload         │ CRC16        │
│ 1 byte │ 1 byte │ 2 bytes (LE) │ 2 bytes (LE) │ 0~N bytes       │ 2 bytes (LE) │
└────────┴────────┴──────────────┴──────────────┴─────────────────┴──────────────┘
```

| Field | Offset | Length | Endianness | Description |
|------|------|------|--------|------|
| Header Byte 1 | 0 | 1 | - | First magic byte of the packet type |
| Header Byte 2 | 1 | 1 | - | Second magic byte of the packet type |
| Command/Type | 2 | 2 | Little-endian | Command code or broadcast type code |
| Payload Length | 4 | 2 | Little-endian | Payload length (0 to `MAX_PAYLOAD_LEN`) |
| Payload | 6 | N | - | Payload data |
| CRC16 | 6+N | 2 | Little-endian | CRC16-CCITT from Header1 through the end of Payload |

### 4.2 Packet Types and Magic Bytes

| Type | Enum Value | HDR1 | HDR2 | Direction | Description |
|------|--------|------|------|------|------|
| `TYPE_HOST_CMD` | 0 | `0x7C` | `0xC7` | Host -> module | Command issued by the host |
| `TYPE_SLAVE_RESP` | 1 | `0x2D` | `0x3D` | Module -> host | Module response to a command |
| `TYPE_SLAVE_BCAST` | 2 | `0xD2` | `0xD3` | Module -> host | Module-initiated broadcast/asynchronous report |

> **Source**: `components/app_uart_cmd/app_cmd_uart.h:28-34`

### 4.3 CRC Verification

- **Algorithm**: CRC16-CCITT
- **Polynomial**: `0x1021`
- **Initial Value**: `0xFFFF`
- **Coverage**: All bytes from Header1 through the end of Payload
- **On Validation Failure**: Discard the current packet, reset the state machine, and wait for the next packet

> **Source**: `components/app_uart_cmd/app_cmd_uart.c:20-33` (`crc16_ccitt`)

### 4.4 State-Machine Parsing

UART receive handling uses the following byte-by-byte parser state machine:

```
STATE_IDLE ──[HDR1 match]──► STATE_HDR_2 ──[HDR2 match]──► STATE_CMD_LO
                                                                      │
STATE_CRC_HI ◄── STATE_CRC_LO ◄── [len==0?] ◄── STATE_LEN_HI ◄── STATE_LEN_LO
       │                                                       │
       └──[CRC valid]──► callback on_uart_cmd_complete()       │
                                     ┌────────────────────────────────────┘
                                     ▼
                              STATE_PAYLOAD ──[payload complete]──► STATE_CRC_LO
```

> **Source**: `components/app_uart_cmd/app_cmd_uart.c:110-223` (`uart_rx_parser`)

### 4.5 Response Format

For a `TYPE_HOST_CMD` command sent by the host, the module returns a `TYPE_SLAVE_RESP` response:

- **Success**: `CMD` field = original command code, `Payload` contains return data
- **Failure**: the module **does not** return an error in the response packet; instead, it sends an `APP_BROADCAST_CMD_ERROR` broadcast

> **Source**: `main/main.c:415-416`, `main/main.c:402-411`

### 4.6 Error Reporting Mechanism

A command code the firmware does not know is answered with a `CMD_ERROR` broadcast carrying `ESP_ERR_NOT_SUPPORTED`, so a host built for a newer protocol fails fast instead of waiting out its timeout.

When command processing fails, the module reports the error through a broadcast packet:

```
Broadcast Type: APP_BROADCAST_CMD_ERROR (8091)
Payload (6 bytes, PACKED):
   ┌──────────────┬──────────────┐
   │ cmd (2 bytes)│ err (4 bytes)│
   │ uint16_t LE  │ int32_t LE   │
   └──────────────┴──────────────┘
```

> **Source**: `main/main.c:402-411` (`uart_cmd_error_report`)

### 4.7 Baud-Rate Switching Mechanism

Baud-rate switching follows a special sequence:

1. The host sends the `APP_CMD_SET_SYS_UART_CMD_BAUD_RATE` command with the new baud rate.
2. The module validates that the baud rate is supported.
3. The module first stays on the old baud rate and notifies the host through a callback.
4. In the callback, the module sends the `TYPE_SLAVE_RESP` response at the **old** baud rate.
5. After UART TX completes, the module switches to the **new** baud rate.
6. If the switch fails, the module automatically reboots and restores the default baud rate to avoid becoming inaccessible.

> **Important**: After the host receives the response to this command, it must immediately switch its own baud rate to match the new rate.

> **Source**: `components/app_uart_cmd/app_cmd_uart.c:440-483` (`app_uart_set_baud_rate`)

### 4.8 Light Sleep and the Wake Preamble

The module uses ESP-IDF automatic light sleep. The command UART can wake it on RX
edges, but the bytes that carried those edges are lost, so the link follows two rules:

1. **Module side**: any byte sent or received on the command UART takes a
   no-light-sleep lock; the lock is released 2 s (`UART_LINK_AWAKE_MS`) after the
   last byte. The module is therefore always awake for the reply to a command and
   for the host's reply to forwarded data.
2. **Host side**: before its first frame, and whenever it has not sent anything
   itself for more than 1 s, it sends a throw-away preamble (four `0x55` bytes, five rising edges
   each) and waits about 10 ms before the real frame. The parser discards the
   preamble as noise, so it is harmless to module firmware without light sleep.

The module never drops its no-light-sleep lock until it has seen the preamble (a
run of >= 3 bytes of `0x55`) at least once since boot. A host that never sends the
preamble at all (older PM5 firmware, for example) therefore never meets a sleeping
module: light sleep simply stays off for that link, and no frame is ever lost to it.
DFS (`CONFIG_PM_ENABLE`) runs independently of this and still applies.

**Command UART clock.** Under DFS the ESP32-C2 switches its PLL off whenever the CPU parks on the crystal (there is no PLL consumer refcount on this chip, unlike C3/C6). The command UART therefore runs from `UART_SCLK_XTAL`, never the default PLL branch: a PLL-clocked UART loses its baud clock between transfers and drops every unsolicited host frame, which looks like a dead module. Keep it that way.

**Power-save switch.** Everything above, DFS and the two-phase advertising
(section 5.4) hang off one persisted switch, `APP_CMD_SET_SYS_POWER_SAVE`
(section 9.2). Off, the module pins its CPU at `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ`,
never light-sleeps and advertises at NimBLE's fast default: the stock behaviour,
kept for A/B power measurements and for troubleshooting the link. BLE controller
modem sleep (`CONFIG_BT_LE_SLEEP_ENABLE`) is compile-time and stays on either way.

> **Source**: `components/app_uart_cmd/app_cmd_uart.c` (`link_touch`, `link_idle_cb`, `link_preamble_scan`, `link_sleep_init`), `components/app_power/app_power.c`

---

## 5. BLE Information

### 5.1 Overview

The module acts as a BLE Peripheral and runs an SPP (Serial Port Profile) server while also providing the standard Battery Service. It is based on the NimBLE stack (the Apache NimBLE component in ESP-IDF).

### 5.2 BLE Service Definitions

#### SPP Service

| Attribute | Value |
|------|-----|
| Service UUID (16-bit) | `0xAE86` |
| Characteristic UUID (16-bit) | `0xAE88` |
| Properties | Read, Write, Write No Response, Notify |
| Extra when bonding/encryption is enabled | Read_Enc, Write_Enc, Notify_Indicate_Enc |

#### Battery Service

| Attribute | Value |
|------|-----|
| Service UUID (16-bit) | `0x180F` (standard) |
| Characteristic UUID (16-bit) | `0x2A19` (standard) |
| Properties | Read, Notify |

> **Source**: `components/app_ble_spp/ble_spp_server.h:19-28`

### 5.3 Advertising Data

| Advertising Field | Content |
|----------|------|
| Flags | LE General Discoverable, BR/EDR Not Supported |
| TX Power Level | Current advertising power (dBm) |
| Device Name | Full device name (default `CONFIG_DEVICE_IDENTIFIER`) |
| 16-bit Service UUIDs | `0xAE86` (SPP), `0x180F` (BAS) |
| Scan Response Manufacturer Data | User-configurable (up to 24 bytes) |

> **Source**: `components/app_ble_spp/app_ble_spp.c:464-513`

### 5.4 Connection Parameters

| Parameter | Description |
|------|------|
| Connection Mode | `BLE_GAP_CONN_MODE_UND` (undirected connectable) |
| Discovery Mode | `BLE_GAP_DISC_MODE_GEN` (general discoverable) |
| Advertising Duration | Infinite (`BLE_HS_FOREVER`); with power save on the fast phase is bounded to `ADV_FAST_DURATION_MS` (30 s), then restarted infinite at the slow interval |
| Advertising Interval | Power save on (default): NimBLE's fast default (30-60 ms) for 30 s after boot, a disconnect or a power-save toggle, then 1022.5 ms (`ADV_SLOW_ITVL`) until something connects. Power save off: NimBLE's fast default throughout. Changes take effect at once. |
| Preferred MTU | `CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU` (config item) |
| Preferred PHY | 2M PHY when both sides support it |

> **Source**: `components/app_ble_spp/app_ble_spp.c:551-553, 806-817`

### 5.5 Pairing and Bonding

| Parameter | Default | Description |
|------|--------|------|
| Bonding | Disabled | Can be enabled by command |
| IO Capability | Display Only when bonding is enabled / No Input No Output when not using secure pairing |
| MITM | Enabled when bonding is enabled | |
| Secure Connections | Enabled when bonding is enabled | |
| Pairing PIN | `123456` | 6-digit number; can be changed by command |
| Key Distribution | Encryption key + identity key | |

**Pairing behavior**:
- Bonding disabled: security level = 1 (security unsupported), NimBLE directly replies with Pairing Failed, and the phone does not show a pairing dialog.
- Bonding enabled: security level = 0, static PIN is used (Display Only), and the phone can pair by entering the preset 6-digit PIN.

> **Source**: `components/app_ble_spp/app_ble_spp.c:384-397` (`app_ble_apply_sm_cfg`)

### 5.6 BLE Transmission Mechanism

- **Requirement**: BLE data can only be sent when a BLE connection is established and notifications are enabled.
- **Segmentation**: Data is automatically split according to the negotiated ATT MTU (maximum per packet = `MTU - 3`).
- **Retry**: If send returns `BLE_HS_ENOMEM`, retry automatically (default maximum 250 attempts).
- **Thread Safety**: Safe to call from multiple tasks under FreeRTOS.

> **Source**: `components/app_ble_spp/app_ble_spp.c:1241-1281` (`app_ble_send`)

### 5.7 Battery Service

| Parameter | Value |
|------|-----|
| Default Battery Level | `0xFF` (unknown) |
| Valid Range | 0-100 (percent), or `0xFF` (unknown) |
| Auto Notify on Update | Yes |

> **Source**: `components/app_ble_spp/app_ble_spp.c:34, 1775-1787`

---

## 6. WiFi Information

### 6.1 WiFi Mode

The module always operates in **STA (Station)** mode and does not support AP mode.

### 6.2 WiFi Connection Management

WiFi connection management is driven by an internal state machine:

```
APP_WIFI_DISCONNECTED ──► APP_WIFI_CONNECTING ──► APP_WIFI_CONNECTED
             ▲                        │                       │
             │                        ▼                       ▼
             └── APP_WIFI_RECONNECT_WAIT ◄────────────────────┘
                                    │
                                    ▼
                     APP_WIFI_CONNECT_STOP
```

| State | Exported Value | Description |
|------|--------|------|
| `APP_WIFI_CONNECT_STOP` | 0 | Connection task stopped |
| `APP_WIFI_CONNECTED` (no IP) | 1 | Connected to AP, IPv4 not obtained |
| `APP_WIFI_CONNECTED` (with IP) | 2 | Connected to AP, IPv4 obtained |
| `APP_WIFI_CONNECTING/RECONNECT_WAIT` | 3 | Connecting or waiting to reconnect |
| `APP_WIFI_DISCONNECTED` | 4 | Disconnected |

> **Source**: `components/app_wifi_connect/app_wifi_connect.c:1067-1097` (`app_wifi_connect_get_status`)

### 6.3 WiFi Configuration Parameters

| Parameter | Default | Range | Persisted |
|------|--------|------|--------|
| SSID | (empty) | Up to 32 bytes | Yes (`esp_wifi_set_config`) |
| Password | (empty) | Up to 64 bytes | Yes (`esp_wifi_set_config`) |
| BSSID | (empty) | 0 or 6 bytes | Yes (`esp_wifi_set_config`) |
| Authmode Threshold | `WIFI_AUTH_OPEN` | See `wifi_auth_mode_t` | Yes |
| Listen Interval | 3 | 1-100 (beacon interval) | Yes |
| Power-save type | 1 (`WIFI_PS_MIN_MODEM`) | 0=none, 1=min modem, 2=max modem (sleeps for the listen interval) | Yes (NVS `wifi_ps`) |
| Scan Method | `WIFI_FAST_SCAN` | 0=fast, 1=all-channel | Yes |
| PMF Mode | 0 (disabled) | 0=disabled, 1=capable, 3=required | Yes |
| Reconnect Interval | 1 second | 0-65535 seconds (0=connect only once) | Yes |
| Country Code | Auto (`01`, channels 1-11) | Configurable by UART command | No |
| TX Power | Maximum (`sdkconfig`) | 8-80 (0.25 dBm step -> 2-20 dBm) | Yes |
| Inactive Timeout | 6 seconds | U16 (seconds) | Yes |
| DHCP | Enabled | 0=disabled, 1=enabled | Yes |
| Static IP | (empty) | IPv4 little-endian U32 x 3 | Yes |
| Hostname | `CONFIG_DEVICE_IDENTIFIER` | Up to 32 bytes | Yes |
| MAC Address | eFuse-burned value | 6 bytes | Yes |

> **Source**: `main/main.c:85-97`, `main/main_settings.c`

### 6.4 WiFi Scanning

| Parameter | Default | Description |
|------|--------|------|
| Scan Type | Active | Active or passive |
| Show Hidden APs | true | |
| Minimum Scan Time | 120 ms | Minimum per-channel scan time in active mode |
| Maximum Scan Time | 120 ms | Maximum per-channel scan time in active mode / per-channel scan time in passive mode |
| Channel Bitmap | 0 (all channels) | 2.4 GHz channel bitmap mask |

Scan results are reported through `APP_BROADCAST_WIFI_SCAN_RESULT` broadcasts.

> **Source**: `components/app_wifi_scanner/app_wifi_scanner.c:201-207`

### 6.5 WiFi Protocol Standards

Use `APP_CMD_SET_WIFI_CFG_PROTOCOL` to set the 2.4 GHz protocol bitmask:

| Bit | Protocol |
|----|------|
| bit 0 | 802.11b |
| bit 1 | 802.11g |
| bit 2 | 802.11n |
| bit 3 | 802.11 LR (Espressif proprietary) |
| bit 4 | 802.11ax |

### 6.6 SNTP Time Synchronization

| Parameter | Default | Description |
|------|--------|------|
| Server | `pool.ntp.org` | Can be changed by command |
| Sync Interval | `sdkconfig` default | Minimum 15000 ms (15 s) |
| Operating Mode | Poll | |
| Enabled by Default | Yes | Can be disabled by command |

**SNTP sync status**:

| Exported Value | Description |
|--------|------|
| 0 | SNTP disabled |
| 1 | SNTP idle (not started) |
| 2 | Sync in progress |
| 3 | Sync completed |

> **Source**: `components/app_wifi_sntp/app_wifi_sntp.c`

---

## 7. Passthrough Information

### 7.1 Passthrough Architecture

Passthrough uses a **bidirectional multi-channel** model:

```
                                     ┌──────────────┐
 Host ◄──UART CMD/Data──►│              │
                                     │  Forwarding  │
                                     │    Engine    ├── BLE SPP ────► BLE peer
                                     │   (main.c)   │
                                     │              ├── TCP Server ─► TCP client
                                     │              ├── TCP Client ─► TCP server
                                     │              ├── UDP Server ─► UDP peer
                                     │              ├── UDP Client ─► UDP server
                                     │              └── MQTT Client ─► MQTT broker
                                     └──────────────┘
```

### 7.2 Data Flow Direction

#### UART -> Wireless (Uplink)

1. The host sends `APP_CMD_SEND_FORWARD_DATA` (5000); the payload is the data to forward.
2. `tx_data_forward_from_uart()` sends the data to both:
    - **BLE** (always attempted)
    - **WiFi** (only when `wifi_function_mode == WIFI_FORWARD`)
3. The `TYPE_SLAVE_RESP` for 5000 is sent before step 2, when the frame leaves the UART ring (flow-control credit for the host). A frame no channel accepts is logged and dropped.

#### Wireless -> UART (Downlink)

1. The BLE/WiFi receive callback `on_forward_data_received()` is triggered.
2. The data is sent to the host UART through the `APP_BROADCAST_DATA_FORWARD` (8089) broadcast packet.

> **Source**: `main/main.c:294-334` (`tx_data_forward_from_uart`), `main/main.c:142-145`

### 7.3 Send Command

```
Host ──► APP_CMD_SEND_FORWARD_DATA (5000) + Payload ──► Module
Module ──► TYPE_SLAVE_RESP (5000) ──► Host (frame taken out of the UART ring; sent before forwarding)
```

> **Important**: `APP_CMD_SEND_FORWARD_DATA` does not specify a target. The data is sent to **all currently active passthrough channels**. To disable or isolate a channel, control it through WiFi mode or BLE start/stop commands.

### 7.4 Passthrough Receive

```
Module ──► APP_BROADCAST_DATA_FORWARD (8089) + Payload ──► Host
```

When the host receives this broadcast, it means data from a wireless peer has arrived.

### 7.5 BLE Passthrough Channel

- **Always running**: BLE is initialized and started in `app_main()`.
- **Send condition**: BLE must be connected and notifications must be enabled.
- **Stop**: Can be stopped with `APP_CMD_STOP_BLE_SPP` and restarted with `APP_CMD_START_BLE_SPP`.
- **Switch**: `APP_CMD_SET_BLE_ENABLE` (4023) is the persisted on/off; with it off the stack is not started at boot. The start/stop pair above is transient (used around OTA) and does not touch the setting: with the switch off, `APP_CMD_START_BLE_SPP` (4021) is acknowledged but does nothing, since hosts send it unconditionally after an OTA.
- **Security**: with bonding off (the default) any central in range can connect and use the SPP characteristic. With bonding on (`APP_CMD_SET_BLE_BONDING_ENABLE`, persisted) pairing is LE Secure Connections with MITM and the static 6-digit passkey (`APP_CMD_SET_BLE_BONDING_KEY`, default 123456), and the SPP characteristic requires an encrypted link, so an unpaired central can connect but cannot read, write or subscribe. Bonded centrals reconnect without the passkey until removed (`APP_CMD_DEL_BLE_BONDED_DEVICE` / `APP_CMD_CLEAR_BLE_BONDED`). A change of the bonding flag takes effect at the next stack start (stop + start, or reboot).

### 7.6 WiFi Passthrough Channel

- **Conditionally started**: Only runs in `WIFI_FUNCTION_MODE_WIFI_FORWARD` mode.
- **Start timing**: The corresponding application protocol stack starts automatically after WiFi obtains an IP address.
- **Stop timing**: The application protocol stack stops automatically when WiFi disconnects.
- **Protocol switching**: Reissue `APP_CMD_SET_TO_WIFI_FORWARD_MODE` with a new protocol type.

---

## 8. OTA Information

### 8.1 OTA Principle

The module uses the native ESP-IDF OTA mechanism:

1. Flash contains two equally sized app partitions (`app_a` / `app_b`, corresponding to `ota_0` / `ota_1`).
2. The currently running firmware is in one partition, and OTA writes the new firmware to the other partition.
3. After the write completes, `esp_ota_set_boot_partition()` marks the new partition for the next boot.
4. On reboot, the bootloader loads the new firmware.

### 8.2 OTA Flow

```
Host                              Module
 │                                  │
 │── APP_CMD_OTA_BEGIN ───────────► │  1. Start OTA (pass total firmware size)
 │   (firmware_size: uint32_t)      │     Select the OTA partition not currently running
 │◄── TYPE_SLAVE_RESP ───────────── │
 │                                  │
 │── APP_CMD_OTA_WRITE ───────────► │  2. Write data chunk (can be called many times)
 │   (firmware_chunk_data)          │     Total written size must not exceed the declared size
 │◄── TYPE_SLAVE_RESP ───────────── │
 │   ... (repeat until all data is written) │
 │                                  │
 │── APP_CMD_OTA_END ─────────────► │  3. Finish OTA
 │                                  │     Validate written size == declared size
 │                                  │     Set boot partition to the new image
 │◄── TYPE_SLAVE_RESP ───────────── │
 │                                  │
 │── APP_CMD_REBOOT ──────────────► │  4. Reboot
 │◄── TYPE_SLAVE_RESP ───────────── │     Module reboots into the new firmware
 │                                  │  === Module reboot ===
```

### 8.3 OTA Packet Format

| Command | Payload | Length | Description |
|------|---------|------|------|
| `OTA_BEGIN` | `uint32_t` firmware_size (LE) | 4 | Total firmware size |
| `OTA_WRITE` | Firmware binary chunk | Variable | One chunk per call |
| `OTA_END` | (none) | 0 | Finish and switch partition |

### 8.4 OTA Notes

- **No resume support**: If the update fails midway, it must be restarted from the beginning.
- **Size must match**: At `OTA_END`, the total written size must equal the size declared in `OTA_BEGIN`.
- **BEGIN must come first**: Calling `OTA_WRITE` before `OTA_BEGIN` returns `ESP_ERR_INVALID_STATE`.
- **No overflow allowed**: If the cumulative written size exceeds the declared total size, return `ESP_ERR_INVALID_SIZE`.

> **Source**: `components/app_ota_ops/app_ota_ops.c`

---

## 9. Command List

### 9.1 Conventions

- **Command Codes**: All enum values are defined in `app_command_code_t` (`main/app_com_defs.h`).
- **Endianness**: All multi-byte values are little-endian.
- **Response**: Success returns `TYPE_SLAVE_RESP` (`CMD` = original command code); failures are reported through `APP_BROADCAST_CMD_ERROR` broadcasts.
- **Timing Model**: Commands use a request/response model; broadcasts are asynchronous and unsolicited.

### 9.2 System and General Commands (1000~)

#### 1000 - APP_CMD_GET_VERSION_INFO - Get Firmware Version

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = version string (for example, `"v1.0.0"`) |

> **Source**: `main/main.c:418-424`

#### 1001 - APP_CMD_GET_DEVICE_MODEL - Get Device Model

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `[0xDA, 0x10]` (2 bytes, ESP32-C2 model identifier) |

> **Source**: `main/main.c:426-434`

#### 1002 - APP_CMD_GET_SYS_FREE_HEAP - Get Free System Heap

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint32_t` free_heap_size (LE) |

> **Source**: `main/main.c:436-439`

#### 1003 - APP_CMD_GET_SYS_TIMESTAMP - Get System UTC Timestamp

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint32_t` tv_sec (LE, Unix timestamp) |

> **Source**: `main/main.c:441-446`

#### 1004 - APP_CMD_GET_APP_COMPILE_DATETIME - Get Firmware Build Time

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = compile time string (for example, `"Aug  6 2026 10:00:00"`) |

> **Source**: `main/main.c:448-462`

#### 1005 - APP_CMD_SET_SYS_TIMESTAMP - Set System Time

| Direction | Format |
|------|------|
| Send | Payload = `uint64_t` tv_sec (LE, 8 bytes, Unix timestamp) |
| Response | (empty payload) |

> **Source**: `main/main.c:464-474`

#### 1006 - APP_CMD_GET_SYS_TIME_ZONE - Get Time Zone

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = time zone string (for example, `"CST-8"`, `"UTC0"`) |

> **Source**: `main/main.c:476-482`

#### 1007 - APP_CMD_SET_SYS_TIME_ZONE - Set Time Zone (Persisted)

| Direction | Format |
|------|------|
| Send | Payload = time zone string (maximum 50 bytes, GNU libc TZ format) |
| Response | (empty payload) |

**Example**: `"CST-8"` = China Standard Time UTC+8, `"UTC0"` = UTC.

> **Source**: `main/main.c:484-507`

#### 1008 - APP_CMD_GET_SYS_BASE_MAC_ADDR - Get eFuse Base MAC Address

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = 6-byte MAC address (or 8 bytes if the chip supports IEEE 802.15.4) |

> **Source**: `main/main.c:509-527`

#### 1009 - APP_CMD_GET_SYS_UART_CMD_BAUD_RATE - Get Current UART Baud Rate

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint32_t` baud_rate (LE) |

> **Source**: `main/main.c:529-533`

#### 1010 - APP_CMD_GET_SYS_UART_CMD_MAX_BAUD_RATE - Get Maximum UART Baud Rate

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint32_t` max_baud_rate (LE) |

> **Source**: `main/main.c:535-539`

#### 1011 - APP_CMD_SET_SYS_UART_CMD_BAUD_RATE - Set UART Baud Rate

| Direction | Format |
|------|------|
| Send | Payload = `uint32_t` baud_rate (LE, 4 bytes) |
| Response | (empty payload, **sent at the old baud rate**) |

**Special behavior**: See [4.7 Baud-Rate Switching Mechanism](#47-baud-rate-switching-mechanism).

> **Source**: `main/main.c:541-566`

#### 1012 - APP_CMD_GET_SYS_NVS_STATS - Get NVS Statistics

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | 20-byte PACKED struct: `used_entries(u32)` + `free_entries(u32)` + `available_entries(u32)` + `total_entries(u32)` + `namespace_count(u32)` (all LE) |

> **Source**: `main/main.c:568-594`

#### 1013 - APP_CMD_RESTORE_TO_FACTORY_SETTINGS - Restore Factory Settings

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | (empty payload) |

**Warning**: This command **completely erases the NVS partition**, and all user configuration will be lost. After erasing, NVS is automatically reinitialized. Make sure the module remains powered during the operation.

> **Source**: `main/main.c:596-612`

#### 1014 - APP_CMD_SET_LOG_UART_FORWARD_ENABLE - Enable or Disable Log Forwarding

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` (0=stop, non-zero=start) (1 byte) |
| Response | (empty payload) |

When enabled, ESP_LOGx output is reported through `APP_BROADCAST_SYS_LOG_MESSAGE` broadcasts.

> **Source**: `main/main.c:777-789`

#### 1015 - APP_CMD_GET_LOG_UART_FORWARD_ENABLE - Get Log Forwarding Status

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint8_t` (0=stopped, 1=enabled) |

> **Source**: `main/main.c:792-797`

#### 1016 - APP_CMD_SET_LOG_LEVEL - Set Log Level

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` level + optional `tag_filter` string |
| Response | (empty payload) |

**Log level values** (ESP-IDF standard):

| Value | Level |
|----|------|
| 0 | None |
| 1 | Error |
| 2 | Warning |
| 3 | Info |
| 4 | Debug |
| 5 | Verbose |

**Tag filter rules**:
- Payload only 1 byte: set global log level
- Payload = level + empty string: set global log level
- Payload = level + non-empty string: set log level only for the specified tag

> **Source**: `main/main.c:799-820`

#### 1017 - APP_CMD_GET_LOG_LEVEL - Get Log Level

| Direction | Format |
|------|------|
| Send | Payload = tag string (empty = get global level) |
| Response | Payload = `uint8_t` level |

> **Source**: `main/main.c:822-832`

#### 1018 - APP_CMD_GET_SYS_READY_STATUS - Get System Ready Status

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint8_t` (0=not ready, 1=ready) |

> **Important**: The host should only send other functional commands after the module reports ready.

> **Source**: `main/main.c:614-618`

#### 1019 - APP_CMD_SET_SYS_POWER_SAVE - Set Power-Save Mode

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` (0=off, 1=on) |
| Response | Payload = `uint8_t` applied state |

Applies at once (no reboot) and is persisted in NVS (`app_sys/pwr_save`). See section 4.8 for what the switch covers.

> **Source**: `main/main.c`, `components/app_power/app_power.c`

#### 1020 - APP_CMD_GET_SYS_POWER_SAVE - Get Power-Save Mode

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint8_t` (0=off, 1=on) |

> **Source**: `main/main.c`

#### 1021 - APP_CMD_SET_SYS_HOST_VALUE - Store a Host Setting

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` id + `uint32_t` value (LE) |
| Response | Payload = `uint32_t` stored value |

A setting of the host (PM5) that has no storage of its own. The module keeps it in NVS (`app_host/h<id>`, a u64 holding the value plus a set flag, so a never-stored slot reads as absent rather than 0) and hands it back; it does not interpret it. The ids are the host's: the PM5 uses 0 for its auto power-off switch, 1 for the idle seconds, and 2 for the unplug switch.

> **Source**: `main/main.c`, `main/main_settings.c`

#### 1022 - APP_CMD_GET_SYS_HOST_VALUE - Read a Host Setting

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` id |
| Response | Payload = `uint8_t` present (0 = never stored) + `uint32_t` value (LE) |

> **Source**: `main/main.c`

### 9.3 OTA and Reboot Commands (1800~)

#### 1800 - APP_CMD_OTA_BEGIN - Start OTA

| Direction | Format |
|------|------|
| Send | Payload = `uint32_t` firmware_total_size (LE, 4 bytes) |
| Response | (empty payload) |

> **Source**: `main/main.c:631-645`

#### 1801 - APP_CMD_OTA_WRITE - Write OTA Data Chunk

| Direction | Format |
|------|------|
| Send | Payload = firmware binary data (variable length) |
| Response | (empty payload) |

> **Source**: `main/main.c:647-661`

#### 1802 - APP_CMD_OTA_END - Finish OTA

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | (empty payload) |

> **Source**: `main/main.c:663-671`

#### 1803 - APP_CMD_REBOOT - Reboot Module

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | (empty payload, **the module reboots immediately afterward**) |

**Special behavior**: The module sends the response first, waits for UART TX completion, and then calls `esp_restart()`. Once the host receives this response, the module is about to reboot.

> **Source**: `main/main.c:620-629`

### 9.4 WiFi Mode Commands (2000~)

#### 2000 - APP_CMD_SET_TO_WIFI_DISABLE_MODE - Switch to WiFi Disabled Mode

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | (empty payload) |

Turns off WiFi and frees resources. Only BLE passthrough remains available.

> **Source**: `main/main.c:673-694`

#### 2001 - APP_CMD_SET_TO_WIFI_FORWARD_MODE - Switch to WiFi Forwarding Mode

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` forward_type (0=TCP Server, 1=TCP Client, 2=UDP Server, 3=UDP Client, 4=MQTT Client) |
| Response | (empty payload) |

**Special behavior**:
- When switching from SCAN mode, the scanner is stopped automatically.
- If already in FORWARD mode but the type changes, the old resources are released automatically and the new protocol is initialized.
- If already in FORWARD mode with the same type, no action is taken.

> **Source**: `main/main.c:696-747`

#### 2002 - APP_CMD_SET_TO_WIFI_SCAN_MODE - Switch to WiFi Scan Mode

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | (empty payload) |

> **Source**: `main/main.c:749-775`

#### 2003 - APP_CMD_START_WIFI_SCAN_TASK - Start WiFi Scan Task

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | (empty payload) |

Scan results are reported asynchronously through `APP_BROADCAST_WIFI_SCAN_RESULT`. The module must be in SCAN mode first.

> **Source**: `main/main.c:834-839`

#### 2004 - APP_CMD_STOP_WIFI_SCAN_TASK - Stop WiFi Scan Task

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | (empty payload) |

> **Source**: `main/main.c:841-846`

#### 2005 - APP_CMD_SET_WIFI_SCAN_CONFIG - Set WiFi Scan Parameters

| Direction | Format |
|------|------|
| Send | 11-byte PACKED struct (see below) |
| Response | (empty payload) |

```
Payload structure (PACKED, 11 bytes):
  ┌──────────────┬──────────────┬───────────────────┬───────────────────┬────────────────────────┐
  │ scan_type    │ show_hidden  │ scan_time_min     │ scan_time_max     │ channel_bitmap_2ghz    │
  │ uint8_t      │ uint8_t      │ uint16_t (LE)     │ uint16_t (LE)     │ uint16_t (LE)          │
  │ byte0        │ byte1        │ byte2-3           │ byte4-5           │ byte6-7                │
  └──────────────┴──────────────┴───────────────────┴───────────────────┴────────────────────────┘
```

| Field | Description |
|------|------|
| `scan_type` | 0=active, 1=passive |
| `show_hidden` | 0=hide, 1=show hidden APs |
| `scan_time_min` | Minimum scan time per channel in active mode (ms) |
| `scan_time_max` | Maximum scan time per channel in active mode / scan time per channel in passive mode (ms) |
| `channel_bitmap_2ghz` | Channel bitmap; 0 = all channels |

> **Source**: `main/main.c:848-884`

#### 2006 - APP_CMD_GET_WIFI_SCAN_STATUS - Get WiFi Scan Status

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint8_t` (0=not running, 1=running) |

> **Source**: `main/main.c:886-890`

#### 2007 - APP_CMD_SET_WIFI_CFG_COUNTRY - Set WiFi Country Code

| Direction | Format |
|------|------|
| Send | 6-byte PACKED struct: `country_policy(u8)` + `country_code[3]` + `start_channel(u8)` + `total_channel_count(u8)` |
| Response | (empty payload) |

| Field | Description |
|------|------|
| `country_policy` | 0=auto, 1=manual |
| `country_code` | ISO 3166-1 country code (for example, `"CN "`) |
| `start_channel` | First channel number |
| `total_channel_count` | Total channel count |

> **Source**: `main/main.c:892-907`

#### 2008 - APP_CMD_GET_WIFI_CFG_COUNTRY - Get WiFi Country Code

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | 6-byte PACKED struct (same format as SET) |

> **Source**: `main/main.c:909-928`

#### 2009 - APP_CMD_SET_WIFI_CFG_TX_PWR - Set WiFi TX Power (Persisted)

| Direction | Format |
|------|------|
| Send | Payload = `int8_t` tx_power (1 byte, automatically clamped to 8-80) |
| Response | (empty payload) |

**Power conversion**: value x 0.25 = dBm. Range 8-80 corresponds to 2-20 dBm.

> **Source**: `main/main.c:930-957`

#### 2010 - APP_CMD_GET_WIFI_CFG_TX_PWR - Get WiFi TX Power

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `int8_t` max_power (1 byte) |

> **Source**: `main/main.c:959-968`

#### 2011 - APP_CMD_SET_WIFI_CFG_INACTIVE_TIME - Set WiFi Inactive Timeout (Persisted)

| Direction | Format |
|------|------|
| Send | Payload = `uint16_t` inactive_time_seconds (LE, 2 bytes) |
| Response | (empty payload) |

> **Source**: `main/main.c:970-996`

#### 2012 - APP_CMD_GET_WIFI_CFG_INACTIVE_TIME - Get WiFi Inactive Timeout

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint16_t` seconds (LE, 2 bytes) |

> **Source**: `main/main.c:998-1007`

#### 2013 - APP_CMD_SET_WIFI_CFG_DHCP - Set DHCP (Persisted)

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` (0=disabled, non-zero=enabled) (1 byte) |
| Response | (empty payload) |

**Note**: If DHCP is enabled and you later want to set a static IP, disable DHCP first and then set the IP parameters.

> **Source**: `main/main.c:1009-1029`

#### 2014 - APP_CMD_GET_WIFI_CFG_DHCP - Get DHCP Status

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint8_t` dhcp_status (`esp_netif_dhcp_status_t`) |

> **Source**: `main/main.c:1031-1040`

#### 2015 - APP_CMD_SET_WIFI_CFG_PROTOCOL - Set WiFi Protocol Standards

| Direction | Format |
|------|------|
| Send | Payload = `uint16_t` protocol_bitmask (LE, 2 bytes) |
| Response | (empty payload) |

> **Source**: `main/main.c:1042-1064`

#### 2016 - APP_CMD_GET_WIFI_CFG_PROTOCOL - Get WiFi Protocol Standards

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint16_t` protocol_bitmask (LE, 2 bytes) |

> **Source**: `main/main.c:1066-1075`

#### 2017 - APP_CMD_SET_WIFI_CFG_MAC_ADDR - Set WiFi MAC Address (Persisted)

| Direction | Format |
|------|------|
| Send | Payload = 6-byte MAC address |
| Response | (empty payload) |

> **Source**: `main/main.c:1077-1100`

#### 2018 - APP_CMD_GET_WIFI_CFG_MAC_ADDR - Get WiFi MAC Address

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = 6-byte MAC address |

> **Source**: `main/main.c:1102-1110`

#### 2019 - APP_CMD_SET_WIFI_CFG_IP_ADDR - Set Static IP Address (Persisted)

| Direction | Format |
|------|------|
| Send | Payload = 12 bytes: `IP(u32 LE)` + `Gateway(u32 LE)` + `Netmask(u32 LE)` |
| Response | (empty payload) |

**Note**: The IP address uses big-endian byte order in human-readable form (`192.168.1.100` = `[0xC0, 0xA8, 0x01, 0x64]`), but is stored internally as little-endian `u32` values.

> **Source**: `main/main.c:1112-1146`

#### 2020 - APP_CMD_GET_WIFI_CFG_IP_ADDR - Get IP Address Configuration

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = 12 bytes (same as SET) |

> **Source**: `main/main.c:1148-1157`

#### 2021 - APP_CMD_SET_WIFI_CFG_HOST_NAME - Set WiFi Hostname (Persisted)

| Direction | Format |
|------|------|
| Send | Payload = hostname string (maximum 32 bytes) |
| Response | (empty payload) |

> **Source**: `main/main.c:1159-1185`

#### 2022 - APP_CMD_GET_WIFI_CFG_HOST_NAME - Get WiFi Hostname

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = hostname string |

> **Source**: `main/main.c:1187-1199`

#### 2023~2036 - WiFi Connection Configuration Commands

| Command Code | Name | Payload |
|--------|------|---------|
| 2023 | `SET_WIFI_CONNECT_CFG_SSID` | SSID string (<=32 bytes, empty=clear) |
| 2024 | `GET_WIFI_CONNECT_CFG_SSID` | (empty) -> SSID string |
| 2025 | `SET_WIFI_CONNECT_CFG_PASSWORD` | Password string (<=64 bytes, empty=clear) |
| 2026 | `GET_WIFI_CONNECT_CFG_PASSWORD` | (empty) -> password string |
| 2027 | `SET_WIFI_CONNECT_CFG_BSSID` | 6-byte BSSID (empty=clear) |
| 2028 | `GET_WIFI_CONNECT_CFG_BSSID` | (empty) -> 6-byte BSSID |
| 2029 | `SET_WIFI_CONNECT_CFG_AUTHMODE` | `uint8_t` authmode |
| 2030 | `GET_WIFI_CONNECT_CFG_AUTHMODE` | (empty) -> `uint8_t` authmode |
| 2031 | `SET_WIFI_CONNECT_CFG_LISTEN_INTERVAL` | `uint16_t` interval (LE) |
| 2032 | `GET_WIFI_CONNECT_CFG_LISTEN_INTERVAL` | (empty) -> `uint16_t` interval (LE) |
| 2033 | `SET_WIFI_CONNECT_CFG_SCAN_MODE` | `uint8_t` scan_mode |
| 2034 | `GET_WIFI_CONNECT_CFG_SCAN_MODE` | (empty) -> `uint8_t` scan_mode |
| 2035 | `SET_WIFI_CONNECT_CFG_PMF` | `uint8_t` pmf_mode (0/1/3) |
| 2036 | `GET_WIFI_CONNECT_CFG_PMF` | (empty) -> `uint8_t` pmf_mode |
| 2037 | `SET_WIFI_CONNECT_CFG_RECONNECT_INTERVAL` | `uint16_t` interval_sec (LE) |
| 2038 | `GET_WIFI_CONNECT_CFG_RECONNECT_INTERVAL` | (empty) -> `uint16_t` interval_sec (LE) |

#### 2039~2048 - SNTP Commands

| Command Code | Name | Payload |
|--------|------|---------|
| 2039 | `SET_WIFI_SNTP_ENABLE` | `uint8_t` enable (0/1) |
| 2040 | `GET_WIFI_SNTP_ENABLE` | (empty) -> `uint8_t` enable |
| 2041 | `SET_WIFI_SNTP_SERVER` | Server address string |
| 2042 | `GET_WIFI_SNTP_SERVER` | (empty) -> server address string |
| 2043 | `SET_WIFI_SNTP_INTERVAL` | `uint32_t` interval_ms (LE, >=15000) |
| 2044 | `GET_WIFI_SNTP_INTERVAL` | (empty) -> `uint32_t` interval_ms (LE) |
| 2045 | `START_WIFI_SNTP` | (empty) |
| 2046 | `STOP_WIFI_SNTP` | (empty) |
| 2047 | `GET_WIFI_SNTP_SYNC_STATUS` | (empty) -> `uint8_t` status (0-3) |

#### 2048~2050 - WiFi Connection Control Commands

| Command Code | Name | Payload |
|--------|------|---------|
| 2048 | `START_WIFI_CONNECT_TASK` | (empty) |
| 2049 | `STOP_WIFI_CONNECT_TASK` | (empty) |
| 2050 | `GET_WIFI_CONNECT_STATUS` | (empty) -> `uint8_t` status (0-4) |

#### 2051 - APP_CMD_WAIT_FOR_WIFI_CONNECT_TASK - Wait for WiFi Connection Result

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` timeout_seconds (1 byte) |
| Response | 2-byte PACKED struct: `connect_result(u8)` + `wifi_err_reason(u8)` |

| `connect_result` | Meaning |
|-----------------|------|
| 0 | Connection successful |
| 1 | Connection failed (see `wifi_err_reason`) |
| 2 | Timed out |

**Special behavior**: This command **blocks while waiting** for a WiFi connection result. During that wait, the module does not process other commands until success, failure, or timeout.

> **Source**: `main/main.c:1533-1549`

#### 2052 - APP_CMD_SET_WIFI_CFG_PS_MODE - Set WiFi Power-Save Type

| Direction | Format |
|------|------|
| Send | Payload = `uint8_t` (0=none, 1=min modem, 2=max modem) |
| Response | Payload = `uint8_t` applied type |

Persisted in NVS (`app_wifi/wifi_ps`). Applied at once when WiFi is running, and re-applied after every `esp_wifi_start()`, which resets the type to the IDF default (min modem). Max modem sleeps for the configured listen interval (2031/2032) and trades latency for current.

> **Source**: `main/main.c`

#### 2053 - APP_CMD_GET_WIFI_CFG_PS_MODE - Get WiFi Power-Save Type

| Direction | Format |
|------|------|
| Send | (no payload) |
| Response | Payload = `uint8_t` (0=none, 1=min modem, 2=max modem) |

> **Source**: `main/main.c`

### 9.5 Passthrough Command (5000)

#### 5000 - APP_CMD_SEND_FORWARD_DATA - Send Passthrough Data

| Direction | Format |
|------|------|
| Send | Payload = data to send (any length <= `MAX_PAYLOAD_LEN`) |
| Response | (empty payload) |

The response is sent as soon as the frame has been parsed out of the UART ring, before the data goes out over BLE/WiFi: it is a flow-control credit for the host, not a delivery receipt. The data is then sent to BLE and, if WiFi Forward mode is active, to WiFi; a frame no channel accepts is logged and dropped (no `CMD_ERROR`).

> **Source**: `main/main.c:1551-1560`

### 9.6 TCP Server Commands (2200~)

| Command Code | Name | Payload |
|--------|------|---------|
| 2200 | `GET_TCP_SERVER_STATUS` | (empty) -> `uint8_t` state |
| 2201 | `START_TCP_SERVER` | (empty) |
| 2202 | `STOP_TCP_SERVER` | (empty) |
| 2203 | `SET_TCP_SERVER_IP_PROTOCOL` | `uint8_t` mode (IP protocol version) |
| 2204 | `GET_TCP_SERVER_IP_PROTOCOL` | (empty) -> `uint8_t` mode |
| 2205 | `SET_TCP_SERVER_PORT` | `uint16_t` port (LE) |
| 2206 | `GET_TCP_SERVER_PORT` | (empty) -> `uint16_t` port (LE) |
| 2207 | `SET_TCP_SERVER_SO_LINGER` | `int32_t` so_linger (LE, 4 bytes) |
| 2208 | `GET_TCP_SERVER_SO_LINGER` | (empty) -> `int32_t` so_linger (LE) |
| 2209 | `SET_TCP_SERVER_NODELAY` | `uint8_t` enable |
| 2210 | `GET_TCP_SERVER_NODELAY` | (empty) -> `uint8_t` enable |
| 2211 | `SET_TCP_SERVER_SO_SNDTIMEO` | `int32_t` sndtimeo_ms (LE) |
| 2212 | `GET_TCP_SERVER_SO_SNDTIMEO` | (empty) -> `int32_t` sndtimeo_ms (LE) |
| 2213 | `SET_TCP_SERVER_KEEP_ALIVE` | 13-byte PACKED struct (see below) |
| 2214 | `GET_TCP_SERVER_KEEP_ALIVE` | (empty) -> 13-byte PACKED struct |

**TCP Server keep-alive payload (13 bytes)**:
```
enable(u8/1B) + keep_idle(i32/4B LE) + keep_interval(i32/4B LE) + keep_count(i32/4B LE)
```

### 9.7 TCP Client Commands (2300~)

| Command Code | Name | Payload |
|--------|------|---------|
| 2300 | `GET_TCP_CLIENT_STATUS` | (empty) -> `uint8_t` state |
| 2301 | `START_TCP_CLIENT` | (empty) |
| 2302 | `STOP_TCP_CLIENT` | (empty) |
| 2303 | `SET_TCP_CLIENT_IP_ADDR` | IP string (<=48 bytes) |
| 2304 | `GET_TCP_CLIENT_IP_ADDR` | (empty) -> IP string |
| 2305 | `SET_TCP_CLIENT_PORT` | `uint16_t` port (LE) |
| 2306 | `GET_TCP_CLIENT_PORT` | (empty) -> `uint16_t` port (LE) |
| 2307 | `SET_TCP_CLIENT_SO_LINGER` | `int32_t` so_linger (LE) |
| 2308 | `GET_TCP_CLIENT_SO_LINGER` | (empty) -> `int32_t` so_linger (LE) |
| 2309 | `SET_TCP_CLIENT_NODELAY` | `uint8_t` enable |
| 2310 | `GET_TCP_CLIENT_NODELAY` | (empty) -> `uint8_t` enable |
| 2311 | `SET_TCP_CLIENT_SO_SNDTIMEO` | `int32_t` sndtimeo_ms (LE) |
| 2312 | `GET_TCP_CLIENT_SO_SNDTIMEO` | (empty) -> `int32_t` sndtimeo_ms (LE) |
| 2313 | `SET_TCP_CLIENT_KEEP_ALIVE` | 13-byte PACKED struct |
| 2314 | `GET_TCP_CLIENT_KEEP_ALIVE` | (empty) -> 13-byte PACKED struct |

**Note**: `SET_TCP_CLIENT_PORT` must be called after `SET_TCP_CLIENT_IP_ADDR`, otherwise IPv4 vs IPv6 cannot be determined correctly.

### 9.8 UDP Server Commands (2400~)

| Command Code | Name | Payload |
|--------|------|---------|
| 2400 | `GET_UDP_SERVER_STATUS` | (empty) -> `uint8_t` state |
| 2401 | `START_UDP_SERVER` | (empty) |
| 2402 | `STOP_UDP_SERVER` | (empty) |
| 2403 | `SET_UDP_SERVER_IP_PROTOCOL` | `uint8_t` mode |
| 2404 | `GET_UDP_SERVER_IP_PROTOCOL` | (empty) -> `uint8_t` mode |
| 2405 | `SET_UDP_SERVER_PORT` | `uint16_t` port (LE) |
| 2406 | `GET_UDP_SERVER_PORT` | (empty) -> `uint16_t` port (LE) |
| 2407 | `SET_UDP_SERVER_SO_SNDTIMEO` | `int32_t` sndtimeo_ms (LE) |
| 2408 | `GET_UDP_SERVER_SO_SNDTIMEO` | (empty) -> `int32_t` sndtimeo_ms (LE) |
| 2409 | `SET_UDP_SERVER_CLIENT_IP_ADDR` | IP string (<=48 bytes) |
| 2410 | `GET_UDP_SERVER_CLIENT_IP_ADDR` | (empty) -> IP string |
| 2411 | `SET_UDP_SERVER_CLIENT_PORT` | `uint16_t` port (LE) |
| 2412 | `GET_UDP_SERVER_CLIENT_PORT` | (empty) -> `uint16_t` port (LE) |

**Difference between UDP Server and TCP Server**: UDP Server supports setting a fixed target client IP/port. If not set, outgoing UDP packets may use the source address of the last received packet.

### 9.9 UDP Client Commands (2500~)

| Command Code | Name | Payload |
|--------|------|---------|
| 2500 | `GET_UDP_CLIENT_STATUS` | (empty) -> `uint8_t` state |
| 2501 | `START_UDP_CLIENT` | (empty) |
| 2502 | `STOP_UDP_CLIENT` | (empty) |
| 2503 | `SET_UDP_CLIENT_IP_PROTOCOL` | `uint8_t` mode |
| 2504 | `GET_UDP_CLIENT_IP_PROTOCOL` | (empty) -> `uint8_t` mode |
| 2505 | `SET_UDP_CLIENT_LOCAL_PORT` | `uint16_t` port (LE) |
| 2506 | `GET_UDP_CLIENT_LOCAL_PORT` | (empty) -> `uint16_t` port (LE) |
| 2507 | `SET_UDP_CLIENT_SO_SNDTIMEO` | `int32_t` sndtimeo_ms (LE) |
| 2508 | `GET_UDP_CLIENT_SO_SNDTIMEO` | (empty) -> `int32_t` sndtimeo_ms (LE) |
| 2509 | `SET_UDP_CLIENT_SERVER_IP_ADDR` | IP string (<=48 bytes) |
| 2510 | `GET_UDP_CLIENT_SERVER_IP_ADDR` | (empty) -> IP string |
| 2511 | `SET_UDP_CLIENT_SERVER_PORT` | `uint16_t` port (LE) |
| 2512 | `GET_UDP_CLIENT_SERVER_PORT` | (empty) -> `uint16_t` port (LE) |

### 9.10 MQTT Client Commands (2600~)

| Command Code | Name | Payload |
|--------|------|---------|
| 2600 | `GET_MQTT_CLIENT_STATUS` | (empty) -> `uint8_t` state |
| 2601 | `START_MQTT_CLIENT` | (empty) |
| 2602 | `STOP_MQTT_CLIENT` | (empty) |
| 2603 | `SET_MQTT_CLIENT_HOST` | Host string (<=128 bytes, empty=NULL) |
| 2604 | `GET_MQTT_CLIENT_HOST` | (empty) -> host string |
| 2605 | `SET_MQTT_CLIENT_PORT` | `uint16_t` port (LE) |
| 2606 | `GET_MQTT_CLIENT_PORT` | (empty) -> `uint16_t` port (LE) |
| 2607 | `SET_MQTT_CLIENT_PATH` | Path string (<=256 bytes, empty=NULL) |
| 2608 | `GET_MQTT_CLIENT_PATH` | (empty) -> path string |
| 2609 | `SET_MQTT_CLIENT_SCHEME` | `uint8_t` scheme (0-11, see table below) |
| 2610 | `GET_MQTT_CLIENT_SCHEME` | (empty) -> `uint8_t` scheme |
| 2611 | `SET_MQTT_CLIENT_SUBSCRIBE_TOPIC` | Topic string (<=128 bytes, empty=NULL) |
| 2612 | `GET_MQTT_CLIENT_SUBSCRIBE_TOPIC` | (empty) -> topic string |
| 2613 | `SET_MQTT_CLIENT_SUBSCRIBE_QOS` | `uint8_t` qos (0/1/2) |
| 2614 | `GET_MQTT_CLIENT_SUBSCRIBE_QOS` | (empty) -> `uint8_t` qos |
| 2615 | `SET_MQTT_CLIENT_PUBLISH_TOPIC` | Topic string (<=128 bytes, empty=NULL) |
| 2616 | `GET_MQTT_CLIENT_PUBLISH_TOPIC` | (empty) -> topic string |
| 2617 | `SET_MQTT_CLIENT_PUBLISH_QOS` | `uint8_t` qos (0/1/2) |
| 2618 | `GET_MQTT_CLIENT_PUBLISH_QOS` | (empty) -> `uint8_t` qos |
| 2619 | `SET_MQTT_CLIENT_PUBLISH_RETAIN` | `uint8_t` retain (0/1) |
| 2620 | `GET_MQTT_CLIENT_PUBLISH_RETAIN` | (empty) -> `uint8_t` retain |
| 2621 | `SET_MQTT_CLIENT_CLIENT_ID` | Client ID string (<=256 bytes, empty=NULL) |
| 2622 | `GET_MQTT_CLIENT_CLIENT_ID` | (empty) -> client ID string |
| 2623 | `SET_MQTT_CLIENT_USERNAME` | Username string (<=1024 bytes, empty=NULL) |
| 2624 | `GET_MQTT_CLIENT_USERNAME` | (empty) -> username string |
| 2625 | `SET_MQTT_CLIENT_PASSWORD` | Password string (<=1024 bytes, empty=NULL) |
| 2626 | `GET_MQTT_CLIENT_PASSWORD` | (empty) -> password string |
| 2627 | `SET_MQTT_CLIENT_KEEP_ALIVE` | `int32_t` keep_alive (LE, 4 bytes) |
| 2628 | `GET_MQTT_CLIENT_KEEP_ALIVE` | (empty) -> `int32_t` keep_alive (LE) |
| 2629 | `SET_MQTT_CLIENT_DISABLE_CLEAN_SESSION` | `uint8_t` disable (0/1) |
| 2630 | `GET_MQTT_CLIENT_DISABLE_CLEAN_SESSION` | (empty) -> `uint8_t` disable |
| 2631 | `SET_MQTT_CLIENT_LWT_TOPIC` | LWT topic (<=128 bytes, empty=NULL) |
| 2632 | `GET_MQTT_CLIENT_LWT_TOPIC` | (empty) -> LWT topic string |
| 2633 | `SET_MQTT_CLIENT_LWT_MESSAGE` | LWT message (<=128 bytes, empty=NULL) |
| 2634 | `GET_MQTT_CLIENT_LWT_MESSAGE` | (empty) -> LWT message string |
| 2635 | `SET_MQTT_CLIENT_LWT_QOS` | `uint8_t` lwt_qos (0/1/2) |
| 2636 | `GET_MQTT_CLIENT_LWT_QOS` | (empty) -> `uint8_t` lwt_qos |
| 2637 | `SET_MQTT_CLIENT_LWT_RETAIN` | `uint8_t` lwt_retain (0/1) |
| 2638 | `GET_MQTT_CLIENT_LWT_RETAIN` | (empty) -> `uint8_t` lwt_retain |
| 2639 | `ADD_MQTT_CLIENT_ALPN` | ALPN string (<=64 bytes) |
| 2640 | `DEL_MQTT_CLIENT_ALPN` | `uint8_t` index |
| 2641 | `GET_MQTT_CLIENT_ALPN` | (empty) -> `\0`-separated ALPN list string |
| 2642 | `GET_MQTT_CLIENT_ALPN_COUNT` | (empty) -> `uint8_t` count |
| 2643 | `CLEAR_MQTT_CLIENT_ALPN` | (empty) |
| 2644 | `SET_MQTT_CLIENT_SNI_HOST` | SNI hostname (<=256 bytes, empty=NULL) |
| 2645 | `GET_MQTT_CLIENT_SNI_HOST` | (empty) -> SNI hostname string |
| 2646 | `SET_MQTT_CLIENT_CACERT` | CA certificate PEM string (<=`MAX_PAYLOAD_LEN`, empty=NULL) |
| 2647 | `GET_MQTT_CLIENT_CACERT` | (empty) -> CA certificate string |
| 2648 | `SET_MQTT_CLIENT_CCERT` | Client certificate PEM string (<=`MAX_PAYLOAD_LEN`, empty=NULL) |
| 2649 | `GET_MQTT_CLIENT_CCERT` | (empty) -> client certificate string |
| 2650 | `SET_MQTT_CLIENT_CCKEY` | Client private key PEM string (<=`MAX_PAYLOAD_LEN`, empty=NULL) |
| 2651 | `GET_MQTT_CLIENT_CCKEY` | (empty) -> client private key string |
| 2652 | `SET_MQTT_CLIENT_PSK_DATA` | PSK binary data (<=128 bytes, empty=clear) |
| 2653 | `GET_MQTT_CLIENT_PSK_DATA` | (empty) -> PSK binary data |
| 2654 | `SET_MQTT_CLIENT_PSK_HINT` | PSK hint string (<=64 bytes, empty=NULL) |
| 2655 | `GET_MQTT_CLIENT_PSK_HINT` | (empty) -> PSK hint string |

**MQTT scheme codes**:

| Value | Description |
|----|------|
| 0 | MQTT over TCP |
| 1 | MQTT over TLS (do not validate server certificate) |
| 2 | MQTT over TLS (validate server certificate) |
| 3 | MQTT over TLS (provide client certificate) |
| 4 | MQTT over TLS (validate server and provide client certificate) |
| 5 | MQTT over TLS (PSK pre-shared key) |
| 6 | MQTT over WebSocket (TCP) |
| 7 | MQTT over WebSocket Secure (TLS, no validation) |
| 8 | MQTT over WebSocket Secure (TLS, validate server certificate) |
| 9 | MQTT over WebSocket Secure (TLS, provide client certificate) |
| 10 | MQTT over WebSocket Secure (TLS, validate server and provide client certificate) |
| 11 | MQTT over WebSocket Secure (TLS, PSK) |

### 9.11 BLE Commands (4000~)

| Command Code | Name | Payload |
|--------|------|---------|
| 4000 | `SET_BLE_ADV_MFG_DATA` | Manufacturer data (<=24 bytes, empty=clear) |
| 4001 | `GET_BLE_ADV_MFG_DATA` | (empty) -> manufacturer data |
| 4002 | `SET_BLE_DEVICE_NAME` | Device name string (<=31 characters) |
| 4003 | `GET_BLE_DEVICE_NAME` | (empty) -> device name string |
| 4004 | `SET_BLE_NOTIFY_RETRY_MAX` | 4 bytes: `nomem_retry_max(u16 LE)` + `fail_retry_max(u16 LE)` |
| 4005 | `GET_BLE_NOTIFY_RETRY_MAX` | (empty) -> 4 bytes (same as above) |
| 4006 | `SET_BLE_DEVICE_ADDR` | 6-byte MAC address |
| 4007 | `GET_BLE_DEVICE_ADDR` | (empty) -> 6-byte MAC address |
| 4008 | `SET_BLE_BONDING_ENABLE` | `uint8_t` enable (0/1) |
| 4009 | `GET_BLE_BONDING_ENABLE` | (empty) -> `uint8_t` enable |
| 4010 | `SET_BLE_BONDING_KEY` | 6- or 7-byte PIN string (for example, `"123456"`) |
| 4011 | `GET_BLE_BONDING_KEY` | (empty) -> 6-byte PIN string |
| 4012 | `GET_BLE_BONDED_DEVICE_NUMS` | (empty) -> `uint8_t` count |
| 4013 | `GET_BLE_BONDED_DEVICE_ADDR` | `uint8_t` index -> 7 bytes: 6B addr + 1B addr_type |
| 4014 | `DEL_BLE_BONDED_DEVICE` | `uint8_t` index |
| 4015 | `CLEAR_BLE_BONDED` | (empty) |
| 4016 | `SET_BLE_BATTERY_LEVEL` | `uint8_t` level (0-100, or 255=unknown) |
| 4017 | `GET_BLE_BATTERY_LEVEL` | (empty) -> `uint8_t` level |
| 4018 | `SET_BLE_TX_POWER` | 2 bytes: `type(u8)` + `power_level(u8)` |
| 4019 | `GET_BLE_TX_POWER` | `uint8_t` type -> `uint8_t` power_level |
| 4020 | `GET_BLE_SPP_STATUS` | (empty) -> `uint8_t` state (0=stopped, 1=started not connected, 2=connected) |
| 4021 | `START_BLE_SPP` | (empty). Acknowledged but a no-op while the persisted BLE switch (4023) is off: hosts send it unconditionally after an OTA, and the switch wins. |
| 4022 | `STOP_BLE_SPP` | (empty) |
| 4023 | `SET_BLE_ENABLE` | `uint8_t` 0=off 1=on -> `uint8_t` stored state. Persisted (`app_sys/ble_en`, default on); starts or stops the stack at once. Off = radio silent, nothing can connect, the host is reachable over USB/WiFi only. Unlike 4021/4022 it survives a reboot. |
| 4024 | `GET_BLE_ENABLE` | (empty) -> `uint8_t` 0=off 1=on |

**BLE TX power types**:

| type | Meaning |
|------|------|
| 0 | Advertising TX power |
| 1 | Connection TX power |

**Power level codes** (`esp_power_level_t`):

| Level | dBm | Level | dBm |
|-------|-----|-------|-----|
| 0 | -24 | 8 | 0 |
| 1 | -21 | 9 | +3 |
| 2 | -18 | 10 | +6 |
| 3 | -15 | 11 | +9 |
| 4 | -12 | 12 | +12 |
| 5 | -9 | 13 | +15 |
| 6 | -6 | 14 | +18 |
| 7 | -3 | 15 | +20 |

---

## 10. Broadcast and Asynchronous Communication

The module actively pushes information to the host through `TYPE_SLAVE_BCAST` packets. All broadcast packets use header `[0xD2, 0xD3]`.

### 10.1 APP_BROADCAST_WIFI_SCAN_RESULT (8088)

**WiFi scan result report**

```
Payload = N x wifi_scan_result_t (PACKED, 50 bytes each):
  ┌──────┬──────────────────┬──────┬──────────┬───────┬────────────────┬────────────────┬──────┬──────┬──────┬──────┬──────┬─────┬─────┬──────┬──────────┬─────┐
  │ ecn  │ ssid[33]         │ rssi │ mac[6]   │ chan  │ pairwise_cipher│ group_cipher   │is11b │is11g │is11n │is11lr│is11ax│is11a│is11ac│reserved│ wps      │
  │ u8   │ char[33]         │ i8   │ u8[6]    │ u8    │ u8             │ u8             │1b    │1b    │1b    │1b    │1b    │1b   │1b    │9b      │ u8       │
  └──────┴──────────────────┴──────┴──────────┴───────┴────────────────┴────────────────┴──────┴──────┴──────┴──────┴──────┴─────┴─────┴──────┴──────────┴─────┘
```

> **Source**: `main/main.c:130-133`, `components/app_wifi_scanner/app_wifi_scanner.h:7-88`

### 10.2 APP_BROADCAST_DATA_FORWARD (8089)

**Passthrough data receive report**

```
Payload = raw data sent by the wireless peer
```

Whenever BLE or WiFi receives peer data, it is forwarded to the host through this broadcast.

> **Source**: `main/main.c:142-145`

### 10.3 APP_BROADCAST_SYS_LOG_MESSAGE (8090)

**System log report**

```
Payload = log text (UTF-8 string)
```

Reported only when enabled with `APP_CMD_SET_LOG_UART_FORWARD_ENABLE`. The content is ESP-IDF log output, including timestamp, level, tag, and message.

> **Source**: `main/main.c:117-120`, `main/app_com_defs.h:9`

### 10.4 APP_BROADCAST_CMD_ERROR (8091)

**Command execution failure report**

```
Payload (6 bytes, PACKED):
  ┌──────────────┬──────────────┐
  │ cmd (2 bytes)│ err (4 bytes)│
  │ uint16_t LE  │ int32_t LE   │
  └──────────────┴──────────────┘
```

| Field | Description |
|------|------|
| `cmd` | Failed command code |
| `err` | ESP-IDF error code (for example, `ESP_ERR_INVALID_ARG = 0x102`) |

> **Source**: `main/main.c:402-411`

---

### 10.5 APP_BROADCAST_LINK_STATE (8092)

**Wireless client link state**, sent on every change (a client connects to or leaves the BLE SPP service or the WiFi TCP server, or the TCP server is torn down). The module does not repeat it, so a host that boots after the module should query `APP_CMD_GET_BLE_SPP_STATUS` once (2 = connected) for the BLE half. The host uses it to hold off its idle power-off while a client is on.

```
Payload (2 bytes):
  ┌──────────────┬──────────────┐
  │ ble (1 byte) │ wifi (1 byte)│
  │ uint8_t      │ uint8_t      │
  └──────────────┴──────────────┘
```

| Field | Description |
|------|------|
| `ble` | 1 = a BLE central is connected, 0 = none |
| `wifi` | 1 = a client is connected to the TCP server, 0 = none |

## 11. Configuration Persistence and Recovery

### 11.1 Configuration Save Mechanism

The module uses ESP-IDF NVS (Non-Volatile Storage) to persist configuration:

- **Storage Medium**: The `nvs` partition in flash
- **Save Timing**: Most SET commands write to NVS immediately after a successful change
- **Exceptions**: WiFi SSID/Password/BSSID/Authmode/ListenInterval/ScanMethod/PMF and similar settings are written directly into `wifi_config_t`, and are automatically persisted by `esp_wifi_set_config()`

### 11.2 Configuration Load Timing

During startup in `app_main()`, `app_nvs_flash_load()` loads configuration in this order:

1. Time zone, then applies it immediately
2. Power-save switch (`pwr_save`)
3. BLE switch (`ble_en`)
4. WiFi function mode, which determines later initialization path
5. WiFi forwarding type
6. WiFi TX power
7. WiFi power-save type (`wifi_ps`)
8. WiFi inactive timeout
9. DHCP enable state
10. MAC address (or uses the default eFuse value if not present)
11. Static IP information
12. Hostname

BLE-related settings are loaded in `app_ble_init()` -> `app_ble_load_persisted_params()`:
- Bonding enable, pairing key, manufacturer data, device name, device address, notify retry limits, TX power

WiFi SNTP settings are loaded in `app_wifi_sntp_init()`.
WiFi reconnect interval is loaded in `app_wifi_connect_init()`.

### 11.3 Restore Factory Settings

Executed through the `APP_CMD_RESTORE_TO_FACTORY_SETTINGS` (1013) command.

**Effect**: Calls `nvs_flash_erase()` to completely erase the NVS partition, then calls `nvs_flash_init()` to reinitialize it. **All configuration is restored to default values**.

**Notes**:

- After the erase, the module **does not reboot automatically**; the current runtime parameters remain unchanged.
- NVS is reinitialized after erasing, so subsequent SET commands can persist normally again.
- Ensure stable power supply while the command is running.

> **Source**: `main/main.c:596-612`

---

## 12. Host-Side Configuration Flow and Examples

### 12.1 Basic Startup Flow

```
1. Host powers up and waits for the module to become ready
   Send: APP_CMD_GET_SYS_READY_STATUS (1018)
   Wait for: TYPE_SLAVE_RESP with status=1

2. (Optional) Query firmware information
   Send: APP_CMD_GET_VERSION_INFO (1000)
   Send: APP_CMD_GET_DEVICE_MODEL (1001)

3. Configure and start passthrough channels...
```

### 12.2 Configure BLE Passthrough (Enabled by Default)

```
(BLE starts advertising automatically after power-up)

Optional configuration:
  Send: APP_CMD_SET_BLE_DEVICE_NAME (4002) + "PM5-BLE"
  Send: APP_CMD_SET_BLE_BONDING_ENABLE (4008) + 0x01
  Send: APP_CMD_SET_BLE_BONDING_KEY (4010) + "654321"

Once a phone or PC discovers and connects to the BLE device, data can be sent and received.
```

### 12.3 Configure WiFi TCP Server Passthrough

```
1. Configure WiFi connection parameters
   Send: APP_CMD_SET_WIFI_CONNECT_CFG_SSID (2023) + "MyWiFi"
   Send: APP_CMD_SET_WIFI_CONNECT_CFG_PASSWORD (2025) + "password123"

2. Configure TCP Server parameters
   Send: APP_CMD_SET_TCP_SERVER_PORT (2205) + 0x1F90 (8080, LE)
   Send: APP_CMD_SET_TCP_SERVER_NODELAY (2209) + 0x01

3. Switch to WiFi TCP Server forwarding mode
   Send: APP_CMD_SET_TO_WIFI_FORWARD_MODE (2001) + 0x00 (TCP Server)

4. Start WiFi connection
   Send: APP_CMD_START_WIFI_CONNECT_TASK (2048)

5. Wait for connection
   Send: APP_CMD_WAIT_FOR_WIFI_CONNECT_TASK (2051) + 0x1E (30-second timeout)
   Response: connect_result=0 indicates success

6. At this point the TCP Server is listening and waiting for a WiFi client connection
   Query: APP_CMD_GET_TCP_SERVER_STATUS (2200) to inspect server state

7. After a TCP client connects, the host communicates as follows:
   Send passthrough data: APP_CMD_SEND_FORWARD_DATA (5000) + data
   Receive passthrough data: listen for APP_BROADCAST_DATA_FORWARD (8089)
```

### 12.4 Configure WiFi TCP Client Passthrough

```
1. Configure the TCP client target
   Send: APP_CMD_SET_TCP_CLIENT_IP_ADDR (2303) + "192.168.1.200"
   Send: APP_CMD_SET_TCP_CLIENT_PORT (2305) + 0x1F90 (8080, LE)

2. Switch to WiFi TCP Client forwarding mode
   Send: APP_CMD_SET_TO_WIFI_FORWARD_MODE (2001) + 0x01 (TCP Client)

3. (Follow the same WiFi connection steps as in 12.3)
```

### 12.5 Configure WiFi UDP Client Passthrough

```
1. Configure the destination server
   Send: APP_CMD_SET_UDP_CLIENT_SERVER_IP_ADDR (2509) + "192.168.1.200"
   Send: APP_CMD_SET_UDP_CLIENT_SERVER_PORT (2511) + 0x1F90 (8080, LE)
   Send: APP_CMD_SET_UDP_CLIENT_LOCAL_PORT (2505) + 0x1F91 (8081, LE)

2. Switch to WiFi UDP Client forwarding mode
   Send: APP_CMD_SET_TO_WIFI_FORWARD_MODE (2001) + 0x03 (UDP Client)

3. (Follow the same WiFi connection steps as in 12.3)
```

### 12.6 Configure MQTT Client Passthrough

```
1. Configure the MQTT broker
   Send: APP_CMD_SET_MQTT_CLIENT_HOST (2603) + "mqtt.example.com"
   Send: APP_CMD_SET_MQTT_CLIENT_PORT (2605) + 0x075B (1883, LE)
   Send: APP_CMD_SET_MQTT_CLIENT_CLIENT_ID (2621) + "pm5-device-01"

2. Configure MQTT topics
   Send: APP_CMD_SET_MQTT_CLIENT_PUBLISH_TOPIC (2615) + "pm5/data/tx"
   Send: APP_CMD_SET_MQTT_CLIENT_SUBSCRIBE_TOPIC (2611) + "pm5/data/rx"

3. (Optional) Configure authentication
   Send: APP_CMD_SET_MQTT_CLIENT_USERNAME (2623) + "user"
   Send: APP_CMD_SET_MQTT_CLIENT_PASSWORD (2625) + "pass"

4. Switch to MQTT Client forwarding mode
   Send: APP_CMD_SET_TO_WIFI_FORWARD_MODE (2001) + 0x04 (MQTT Client)

5. (Follow the same WiFi connection steps as in 12.3)
```

### 12.7 Change the WiFi Forwarding Protocol

```
(Assume the module is currently in TCP Server mode and you want to switch to MQTT)

Send: APP_CMD_SET_TO_WIFI_FORWARD_MODE (2001) + 0x04 (MQTT Client)
-> The module automatically:
   1. Stops and releases TCP Server resources
   2. Updates forward_type = MQTT_CLIENT
   3. Initializes MQTT Client resources
   4. Starts MQTT Client automatically after WiFi obtains an IP
```

### 12.8 OTA Firmware Upgrade

```
1. Start OTA
   Send: APP_CMD_OTA_BEGIN (1800) + firmware_size (4 bytes LE)

2. Write the firmware in chunks
   while (data remains) {
       Send: APP_CMD_OTA_WRITE (1801) + chunk_data
   }

3. Finish OTA
   Send: APP_CMD_OTA_END (1802)

4. Reboot into the new firmware
   Send: APP_CMD_REBOOT (1803)
   -> The module returns a response and then reboots
```

### 12.9 Enable Log Debugging

```
1. Set log level
   Send: APP_CMD_SET_LOG_LEVEL (1016) + 0x03 (INFO level)

2. Enable log forwarding
   Send: APP_CMD_SET_LOG_UART_FORWARD_ENABLE (1014) + 0x01

3. After this, ESP_LOGx output is reported through APP_BROADCAST_SYS_LOG_MESSAGE (8090)
```

### 12.10 Set System Time

```
1. Set time zone
   Send: APP_CMD_SET_SYS_TIME_ZONE (1007) + "CST-8"

2. Set UTC timestamp (if SNTP is not enabled)
   Send: APP_CMD_SET_SYS_TIMESTAMP (1005) + timestamp (8 bytes LE)
```

---

## 13. Over-the-Air Characteristics

### 13.1 BLE Over-the-Air Characteristics

| Parameter | Value/Description |
|------|---------|
| BLE Version | Bluetooth 5.0 LE |
| PHY | 1M by default; requests 2M PHY after connection |
| Advertising Interval | NimBLE default (determined by `BLE_HS_FOREVER` and default parameters) |
| Advertising Channels | 37, 38, 39 (standard BLE advertising channels) |
| MTU | Negotiable (default `CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU`) |
| Security | Supports LE Secure Connections + Bonding (optional) |
| Connection Interval | Determined by the central; the module does not constrain it |
| Slave Latency | Determined by the central |
| Supervision Timeout | Determined by the central |

### 13.2 WiFi Over-the-Air Characteristics

| Parameter | Value/Description |
|------|---------|
| WiFi Standards | 802.11b/g/n (ax configurable) |
| Band | 2.4 GHz |
| Mode | STA (Station), client-only |
| Channel | 1-11 by default (automatic), can be restricted by country code |
| TX Power | 2-20 dBm (configurable, 0.25 dBm step) |
| Security | Open / WEP / WPA-PSK / WPA2-PSK / WPA3-PSK / WPA2-Enterprise, etc. |
| PMF | Supported (configurable as Capable/Required) |
| DHCP | Enabled by default (can switch to static IP) |
| Hostname | Can be reported through DHCP |

### 13.3 Power Characteristics

[Inferred] When the module powers up, BLE advertising and WiFi connection may both be active by default if the stored mode is Forward. Power consumption depends on:

- BLE advertising interval and transmit power
- WiFi connection state and data throughput
- Transmit power can be adjusted through `APP_CMD_SET_BLE_TX_POWER` and `APP_CMD_SET_WIFI_CFG_TX_PWR`
- WiFi power-saving behavior can be adjusted through `APP_CMD_SET_WIFI_CFG_INACTIVE_TIME`
- WiFi can be disabled entirely through `APP_CMD_SET_TO_WIFI_DISABLE_MODE` to reduce power consumption

### 13.4 WiFi Scanning Characteristics

| Parameter | Value/Description |
|------|---------|
| Scan Mode | Active (sends Probe Request) or Passive (listen to Beacon only) |
| Per-Scan Duration | Configurable (default active 120 ms/channel) |
| Result Limit | `CONFIG_WIFI_SCAN_LIST_SIZE` (`sdkconfig` item) |
| Report Method | Reports the full result set once after each scan cycle through a broadcast packet |
| Result Fields | SSID, BSSID, RSSI, channel, encryption mode, cipher suites, PHY modes, WPS support |

---

## Appendix A: Command Code Quick Reference

| Range | Category | Command Count |
|--------|------|--------|
| 1000~1022 | System and general | 23 |
| 1800~1803 | OTA and reboot | 4 |
| 2000~2053 | WiFi mode/configuration/connection | 54 |
| 2200~2214 | TCP Server | 15 |
| 2300~2314 | TCP Client | 15 |
| 2400~2412 | UDP Server | 13 |
| 2500~2512 | UDP Client | 13 |
| 2600~2655 | MQTT Client | 56 |
| 4000~4024 | BLE | 25 |
| 5000 | Passthrough | 1 |
| **Total** | | **219** |

## Appendix B: Broadcast Type Quick Reference

| Type Code | Name | Direction | Description |
|--------|------|------|------|
| 8088 | `WIFI_SCAN_RESULT` | Module -> host | WiFi scan results |
| 8089 | `DATA_FORWARD` | Module -> host | Incoming wireless passthrough data |
| 8090 | `SYS_LOG_MESSAGE` | Module -> host | System log (requires forwarding enabled) |
| 8091 | `CMD_ERROR` | Module -> host | Command execution failure |
| 8092 | `LINK_STATE` | Module -> host | BLE / WiFi client connected or gone (on change only) |

## Appendix C: Error Code Reference

Error codes use standard ESP-IDF `esp_err_t` values. Some common error codes are:

| Error Code | Name | Description |
|--------|------|------|
| 0x0000 | `ESP_OK` | Success |
| 0x0101 | `ESP_ERR_NO_MEM` | Out of memory |
| 0x0102 | `ESP_ERR_INVALID_ARG` | Invalid argument |
| 0x0103 | `ESP_ERR_INVALID_STATE` | Invalid state |
| 0x0104 | `ESP_ERR_INVALID_SIZE` | Invalid size |
| 0x0105 | `ESP_ERR_NOT_FOUND` | Not found |
| 0x0106 | `ESP_ERR_NOT_SUPPORTED` | Not supported |
| 0x0107 | `ESP_ERR_TIMEOUT` | Timeout |
| 0x010C | `ESP_ERR_INVALID_CRC` | CRC check failed |
| 0xFFFF | `ESP_FAIL` | Generic failure |

> **Note**: For the actual failure reason, use the `int32_t` value returned in the `APP_BROADCAST_CMD_ERROR` broadcast.
