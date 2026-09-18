#ifndef APP_COM_DEFS_H
#define APP_COM_DEFS_H


// Broadcast type definitions
typedef enum {
    APP_BROADCAST_WIFI_SCAN_RESULT              = 8088,    // WiFi scan async report
    APP_BROADCAST_DATA_FORWARD                  = 8089,    // Transparent-forward data
    APP_BROADCAST_SYS_LOG_MESSAGE               = 8090,    // System log message; all ESP_LOGx output is also forwarded here
    APP_BROADCAST_CMD_ERROR                     = 8091,    // Command execution failure report; payload is cmd(uint16) + err(int32)
    APP_BROADCAST_LINK_STATE                    = 8092,    // Wireless client link state; payload ble(uint8) + wifi(uint8), 1 = a client is connected. Sent on every change.
} app_broadcast_type_t;


// Command code definitions
typedef enum {

    /*
     * System and general utility commands; codes start at 1000.
     * When adding new commands, append to the end rather than inserting in the middle
     * to avoid breaking host-side compatibility.
     */

    APP_CMD_GET_VERSION_INFO                    = 1000,    // Get firmware version
    APP_CMD_GET_DEVICE_MODEL                          ,    // Get device model
    APP_CMD_GET_SYS_FREE_HEAP                         ,    // Get system free heap
    APP_CMD_GET_SYS_TIMESTAMP                         ,    // Get system time
    APP_CMD_GET_APP_COMPILE_DATETIME                  ,    // Get current app firmware compile date-time (string)
    APP_CMD_SET_SYS_TIMESTAMP                         ,    // Set system time
    APP_CMD_GET_SYS_TIME_ZONE                         ,    // Get time zone
    APP_CMD_SET_SYS_TIME_ZONE                         ,    // Set time zone (persisted)
    APP_CMD_GET_SYS_BASE_MAC_ADDR                     ,    // Get factory-fixed base MAC address (from eFuse, read-only)
    APP_CMD_GET_SYS_UART_CMD_BAUD_RATE                ,    // Get command UART current baud rate
    APP_CMD_GET_SYS_UART_CMD_MAX_BAUD_RATE            ,    // Get command UART maximum baud rate for this chip
    APP_CMD_SET_SYS_UART_CMD_BAUD_RATE                ,    // Set command UART baud rate (must NOT be persisted)
    APP_CMD_GET_SYS_NVS_STATS                         ,    // Get NVS statistics
    APP_CMD_RESTORE_TO_FACTORY_SETTINGS               ,    // Restore factory settings; erases all user config
    APP_CMD_SET_LOG_UART_FORWARD_ENABLE               ,    // Enable log forwarding to command UART
    APP_CMD_GET_LOG_UART_FORWARD_ENABLE               ,    // Get log-forward-to-command-UART state
    APP_CMD_SET_LOG_LEVEL                             ,    // Set log output level
    APP_CMD_GET_LOG_LEVEL                             ,    // Get log output level
    APP_CMD_GET_SYS_READY_STATUS                      ,    // Get system ready status; only safe to call other commands after system is ready
    APP_CMD_SET_SYS_POWER_SAVE                        ,    // Set power-save mode (persisted); payload uint8_t 0=off 1=on; response = applied state
    APP_CMD_GET_SYS_POWER_SAVE                        ,    // Get power-save mode; response uint8_t 0=off 1=on

    // --- NOTE: OTA and reboot commands are critical for firmware download during development;
    //  do NOT change their codes (order). Add new OTA-related commands after these entries
    //  to keep them contiguous so the host can identify them consistently. ---

    APP_CMD_OTA_BEGIN                           = 1800,    // OTA: begin writing new firmware; param = total size (uint32)
    APP_CMD_OTA_WRITE                                 ,    // OTA: write firmware chunk
    APP_CMD_OTA_END                                   ,    // OTA: finish writing and set next boot partition
    APP_CMD_REBOOT                                    ,    // Reboot the system

    /*
     * WiFi-related commands; codes start at 2000.
     */
    
    APP_CMD_SET_TO_WIFI_DISABLE_MODE            = 2000,    // Switch to WiFi disabled mode
    APP_CMD_SET_TO_WIFI_FORWARD_MODE                  ,    // Switch to WiFi forward mode
    APP_CMD_SET_TO_WIFI_SCAN_MODE                     ,    // Switch to WiFi scan mode
    APP_CMD_START_WIFI_SCAN_TASK                      ,    // Start WiFi scan task
    APP_CMD_STOP_WIFI_SCAN_TASK                       ,    // Stop WiFi scan task
    APP_CMD_SET_WIFI_SCAN_CONFIG                      ,    // Set WiFi scan startup config
    APP_CMD_GET_WIFI_SCAN_STATUS                      ,    // Get WiFi scan status
    APP_CMD_SET_WIFI_CFG_COUNTRY                      ,    // WiFi config: set country code (persisted)
    APP_CMD_GET_WIFI_CFG_COUNTRY                      ,    // WiFi config: get country code
    APP_CMD_SET_WIFI_CFG_TX_PWR                       ,    // WiFi config: set TX power (persisted)
    APP_CMD_GET_WIFI_CFG_TX_PWR                       ,    // WiFi config: get TX power
    APP_CMD_SET_WIFI_CFG_INACTIVE_TIME                ,    // WiFi config: set inactive time
    APP_CMD_GET_WIFI_CFG_INACTIVE_TIME                ,    // WiFi config: get inactive time
    APP_CMD_SET_WIFI_CFG_DHCP                         ,    // WiFi config: set DHCP enable
    APP_CMD_GET_WIFI_CFG_DHCP                         ,    // WiFi config: check DHCP enable
    APP_CMD_SET_WIFI_CFG_PROTOCOL                     ,    // WiFi config: set WiFi protocol standard
    APP_CMD_GET_WIFI_CFG_PROTOCOL                     ,    // WiFi config: get WiFi protocol standard
    APP_CMD_SET_WIFI_CFG_MAC_ADDR                     ,    // WiFi config: set WiFi MAC address
    APP_CMD_GET_WIFI_CFG_MAC_ADDR                     ,    // WiFi config: get WiFi MAC address
    APP_CMD_SET_WIFI_CFG_IP_ADDR                      ,    // WiFi config: set WiFi IP address
    APP_CMD_GET_WIFI_CFG_IP_ADDR                      ,    // WiFi config: get WiFi IP address
    APP_CMD_SET_WIFI_CFG_HOST_NAME                    ,    // WiFi config: set WiFi hostname
    APP_CMD_GET_WIFI_CFG_HOST_NAME                    ,    // WiFi config: get WiFi hostname
    APP_CMD_SET_WIFI_CONNECT_CFG_SSID                 ,    // WiFi config: set target SSID
    APP_CMD_GET_WIFI_CONNECT_CFG_SSID                 ,    // WiFi config: get target SSID
    APP_CMD_SET_WIFI_CONNECT_CFG_PASSWORD             ,    // WiFi config: set target password
    APP_CMD_GET_WIFI_CONNECT_CFG_PASSWORD             ,    // WiFi config: get target password
    APP_CMD_SET_WIFI_CONNECT_CFG_BSSID                ,    // WiFi config: set target BSSID
    APP_CMD_GET_WIFI_CONNECT_CFG_BSSID                ,    // WiFi config: get target BSSID
    APP_CMD_SET_WIFI_CONNECT_CFG_AUTHMODE             ,    // WiFi config: set auth mode threshold
    APP_CMD_GET_WIFI_CONNECT_CFG_AUTHMODE             ,    // WiFi config: get auth mode threshold
    APP_CMD_SET_WIFI_CONNECT_CFG_LISTEN_INTERVAL      ,    // WiFi config: set AP beacon listen interval
    APP_CMD_GET_WIFI_CONNECT_CFG_LISTEN_INTERVAL      ,    // WiFi config: get AP beacon listen interval
    APP_CMD_SET_WIFI_CONNECT_CFG_SCAN_MODE            ,    // WiFi config: set scan mode
    APP_CMD_GET_WIFI_CONNECT_CFG_SCAN_MODE            ,    // WiFi config: get scan mode
    APP_CMD_SET_WIFI_CONNECT_CFG_PMF                  ,    // WiFi config: set PMF (Protected Management Frames)
    APP_CMD_GET_WIFI_CONNECT_CFG_PMF                  ,    // WiFi config: get PMF (Protected Management Frames)
    APP_CMD_SET_WIFI_CONNECT_CFG_RECONNECT_INTERVAL   ,    // WiFi config: set reconnect interval
    APP_CMD_GET_WIFI_CONNECT_CFG_RECONNECT_INTERVAL   ,    // WiFi config: get reconnect interval
    APP_CMD_SET_WIFI_SNTP_ENABLE                      ,    // WiFi config: set SNTP enable
    APP_CMD_GET_WIFI_SNTP_ENABLE                      ,    // WiFi config: get SNTP enable
    APP_CMD_SET_WIFI_SNTP_SERVER                      ,    // WiFi config: set SNTP server address
    APP_CMD_GET_WIFI_SNTP_SERVER                      ,    // WiFi config: get SNTP server address
    APP_CMD_SET_WIFI_SNTP_INTERVAL                    ,    // WiFi config: set SNTP sync interval
    APP_CMD_GET_WIFI_SNTP_INTERVAL                    ,    // WiFi config: get SNTP sync interval
    APP_CMD_START_WIFI_SNTP                           ,    // WiFi control: start SNTP
    APP_CMD_STOP_WIFI_SNTP                            ,    // WiFi control: stop SNTP
    APP_CMD_GET_WIFI_SNTP_SYNC_STATUS                 ,    // WiFi config: get SNTP sync status
    APP_CMD_START_WIFI_CONNECT_TASK                   ,    // Start WiFi connection task
    APP_CMD_STOP_WIFI_CONNECT_TASK                    ,    // Stop WiFi connection task; disconnects any existing connection
    APP_CMD_GET_WIFI_CONNECT_STATUS                   ,    // Get WiFi connection task status
    APP_CMD_WAIT_FOR_WIFI_CONNECT_TASK                ,    // Wait for WiFi connection task to succeed, fail, or timeout
    APP_CMD_SET_WIFI_CFG_PS_MODE                      ,    // WiFi config: set modem power-save type (persisted): uint8_t 0=none 1=min (default) 2=max
    APP_CMD_GET_WIFI_CFG_PS_MODE                      ,    // WiFi config: get modem power-save type: uint8_t

    /*
     * TCP server commands; codes start at 2200.
     */

    APP_CMD_GET_TCP_SERVER_STATUS               = 2200,    // TCP server: get status
    APP_CMD_START_TCP_SERVER                          ,    // TCP server: start
    APP_CMD_STOP_TCP_SERVER                           ,    // TCP server: stop
    APP_CMD_SET_TCP_SERVER_IP_PROTOCOL                ,    // TCP server config: set IP protocol
    APP_CMD_GET_TCP_SERVER_IP_PROTOCOL                ,    // TCP server config: get IP protocol
    APP_CMD_SET_TCP_SERVER_PORT                       ,    // TCP server config: set listen port
    APP_CMD_GET_TCP_SERVER_PORT                       ,    // TCP server config: get listen port
    APP_CMD_SET_TCP_SERVER_SO_LINGER                  ,    // TCP server config: set SO_LINGER
    APP_CMD_GET_TCP_SERVER_SO_LINGER                  ,    // TCP server config: get SO_LINGER
    APP_CMD_SET_TCP_SERVER_NODELAY                    ,    // TCP server config: set TCP_NODELAY
    APP_CMD_GET_TCP_SERVER_NODELAY                    ,    // TCP server config: get TCP_NODELAY
    APP_CMD_SET_TCP_SERVER_SO_SNDTIMEO                ,    // TCP server config: set SO_SNDTIMEO
    APP_CMD_GET_TCP_SERVER_SO_SNDTIMEO                ,    // TCP server config: get SO_SNDTIMEO
    APP_CMD_SET_TCP_SERVER_KEEP_ALIVE                 ,    // TCP server config: set keep-alive
    APP_CMD_GET_TCP_SERVER_KEEP_ALIVE                 ,    // TCP server config: get keep-alive

    /*
     * TCP client commands; codes start at 2300.
     */

    APP_CMD_GET_TCP_CLIENT_STATUS               = 2300,    // TCP client: get status
    APP_CMD_START_TCP_CLIENT                          ,    // TCP client: start
    APP_CMD_STOP_TCP_CLIENT                           ,    // TCP client: stop
    APP_CMD_SET_TCP_CLIENT_IP_ADDR                    ,    // TCP client config: set IP address
    APP_CMD_GET_TCP_CLIENT_IP_ADDR                    ,    // TCP client config: get IP address
    APP_CMD_SET_TCP_CLIENT_PORT                       ,    // TCP client config: set port
    APP_CMD_GET_TCP_CLIENT_PORT                       ,    // TCP client config: get port
    APP_CMD_SET_TCP_CLIENT_SO_LINGER                  ,    // TCP client config: set SO_LINGER
    APP_CMD_GET_TCP_CLIENT_SO_LINGER                  ,    // TCP client config: get SO_LINGER
    APP_CMD_SET_TCP_CLIENT_NODELAY                    ,    // TCP client config: set TCP_NODELAY
    APP_CMD_GET_TCP_CLIENT_NODELAY                    ,    // TCP client config: get TCP_NODELAY
    APP_CMD_SET_TCP_CLIENT_SO_SNDTIMEO                ,    // TCP client config: set SO_SNDTIMEO
    APP_CMD_GET_TCP_CLIENT_SO_SNDTIMEO                ,    // TCP client config: get SO_SNDTIMEO
    APP_CMD_SET_TCP_CLIENT_KEEP_ALIVE                 ,    // TCP client config: set keep-alive
    APP_CMD_GET_TCP_CLIENT_KEEP_ALIVE                 ,    // TCP client config: get keep-alive

    /*
     * UDP server commands; codes start at 2400.
     */

    APP_CMD_GET_UDP_SERVER_STATUS               = 2400,    // UDP server: get status
    APP_CMD_START_UDP_SERVER                          ,    // UDP server: start
    APP_CMD_STOP_UDP_SERVER                           ,    // UDP server: stop
    APP_CMD_SET_UDP_SERVER_IP_PROTOCOL                ,    // UDP server config: set IP protocol
    APP_CMD_GET_UDP_SERVER_IP_PROTOCOL                ,    // UDP server config: get IP protocol
    APP_CMD_SET_UDP_SERVER_PORT                       ,    // UDP server config: set listen port
    APP_CMD_GET_UDP_SERVER_PORT                       ,    // UDP server config: get listen port
    APP_CMD_SET_UDP_SERVER_SO_SNDTIMEO                ,    // UDP server config: set SO_SNDTIMEO
    APP_CMD_GET_UDP_SERVER_SO_SNDTIMEO                ,    // UDP server config: get SO_SNDTIMEO
    APP_CMD_SET_UDP_SERVER_CLIENT_IP_ADDR             ,    // UDP server config: set fixed target IP address
    APP_CMD_GET_UDP_SERVER_CLIENT_IP_ADDR             ,    // UDP server config: get fixed target IP address
    APP_CMD_SET_UDP_SERVER_CLIENT_PORT                ,    // UDP server config: set fixed target port
    APP_CMD_GET_UDP_SERVER_CLIENT_PORT                ,    // UDP server config: get fixed target port


    /*
     * UDP client commands; codes start at 2500.
     */

    APP_CMD_GET_UDP_CLIENT_STATUS               = 2500,    // UDP client: get status
    APP_CMD_START_UDP_CLIENT                          ,    // UDP client: start
    APP_CMD_STOP_UDP_CLIENT                           ,    // UDP client: stop
    APP_CMD_SET_UDP_CLIENT_IP_PROTOCOL                ,    // UDP client config: set IP protocol
    APP_CMD_GET_UDP_CLIENT_IP_PROTOCOL                ,    // UDP client config: get IP protocol
    APP_CMD_SET_UDP_CLIENT_LOCAL_PORT                 ,    // UDP client config: set local port
    APP_CMD_GET_UDP_CLIENT_LOCAL_PORT                 ,    // UDP client config: get local port
    APP_CMD_SET_UDP_CLIENT_SO_SNDTIMEO                ,    // UDP client config: set SO_SNDTIMEO
    APP_CMD_GET_UDP_CLIENT_SO_SNDTIMEO                ,    // UDP client config: get SO_SNDTIMEO
    APP_CMD_SET_UDP_CLIENT_SERVER_IP_ADDR             ,    // UDP client config: set target server IP address
    APP_CMD_GET_UDP_CLIENT_SERVER_IP_ADDR             ,    // UDP client config: get target server IP address
    APP_CMD_SET_UDP_CLIENT_SERVER_PORT                ,    // UDP client config: set target server port
    APP_CMD_GET_UDP_CLIENT_SERVER_PORT                ,    // UDP client config: get target server port

    /*
     * MQTT client commands; codes start at 2600.
     */

    APP_CMD_GET_MQTT_CLIENT_STATUS              = 2600,    // MQTT client: get status
    APP_CMD_START_MQTT_CLIENT                         ,    // MQTT client: start
    APP_CMD_STOP_MQTT_CLIENT                          ,    // MQTT client: stop
    APP_CMD_SET_MQTT_CLIENT_HOST                      ,    // MQTT client config: set broker host address
    APP_CMD_GET_MQTT_CLIENT_HOST                      ,    // MQTT client config: get broker host address
    APP_CMD_SET_MQTT_CLIENT_PORT                      ,    // MQTT client config: set broker port
    APP_CMD_GET_MQTT_CLIENT_PORT                      ,    // MQTT client config: get broker port
    APP_CMD_SET_MQTT_CLIENT_PATH                      ,    // MQTT client config: set broker path
    APP_CMD_GET_MQTT_CLIENT_PATH                      ,    // MQTT client config: get broker path
    APP_CMD_SET_MQTT_CLIENT_SCHEME                    ,    // MQTT client config: set connection scheme
    APP_CMD_GET_MQTT_CLIENT_SCHEME                    ,    // MQTT client config: get connection scheme
    APP_CMD_SET_MQTT_CLIENT_SUBSCRIBE_TOPIC           ,    // MQTT client config: set subscribe topic
    APP_CMD_GET_MQTT_CLIENT_SUBSCRIBE_TOPIC           ,    // MQTT client config: get subscribe topic
    APP_CMD_SET_MQTT_CLIENT_SUBSCRIBE_QOS             ,    // MQTT client config: set subscribe QoS
    APP_CMD_GET_MQTT_CLIENT_SUBSCRIBE_QOS             ,    // MQTT client config: get subscribe QoS
    APP_CMD_SET_MQTT_CLIENT_PUBLISH_TOPIC             ,    // MQTT client config: set publish topic
    APP_CMD_GET_MQTT_CLIENT_PUBLISH_TOPIC             ,    // MQTT client config: get publish topic
    APP_CMD_SET_MQTT_CLIENT_PUBLISH_QOS               ,    // MQTT client config: set publish QoS
    APP_CMD_GET_MQTT_CLIENT_PUBLISH_QOS               ,    // MQTT client config: get publish QoS
    APP_CMD_SET_MQTT_CLIENT_PUBLISH_RETAIN            ,    // MQTT client config: set publish retain flag
    APP_CMD_GET_MQTT_CLIENT_PUBLISH_RETAIN            ,    // MQTT client config: get publish retain flag
    APP_CMD_SET_MQTT_CLIENT_CLIENT_ID                 ,    // MQTT client config: set client ID
    APP_CMD_GET_MQTT_CLIENT_CLIENT_ID                 ,    // MQTT client config: get client ID
    APP_CMD_SET_MQTT_CLIENT_USERNAME                  ,    // MQTT client config: set username
    APP_CMD_GET_MQTT_CLIENT_USERNAME                  ,    // MQTT client config: get username
    APP_CMD_SET_MQTT_CLIENT_PASSWORD                  ,    // MQTT client config: set password
    APP_CMD_GET_MQTT_CLIENT_PASSWORD                  ,    // MQTT client config: get password
    APP_CMD_SET_MQTT_CLIENT_KEEP_ALIVE                ,    // MQTT client config: set keep-alive
    APP_CMD_GET_MQTT_CLIENT_KEEP_ALIVE                ,    // MQTT client config: get keep-alive
    APP_CMD_SET_MQTT_CLIENT_DISABLE_CLEAN_SESSION     ,    // MQTT client config: set disable clean session flag
    APP_CMD_GET_MQTT_CLIENT_DISABLE_CLEAN_SESSION     ,    // MQTT client config: get disable clean session flag
    APP_CMD_SET_MQTT_CLIENT_LWT_TOPIC                 ,    // MQTT client config: set LWT topic
    APP_CMD_GET_MQTT_CLIENT_LWT_TOPIC                 ,    // MQTT client config: get LWT topic
    APP_CMD_SET_MQTT_CLIENT_LWT_MESSAGE               ,    // MQTT client config: set LWT message
    APP_CMD_GET_MQTT_CLIENT_LWT_MESSAGE               ,    // MQTT client config: get LWT message
    APP_CMD_SET_MQTT_CLIENT_LWT_QOS                   ,    // MQTT client config: set LWT QoS
    APP_CMD_GET_MQTT_CLIENT_LWT_QOS                   ,    // MQTT client config: get LWT QoS
    APP_CMD_SET_MQTT_CLIENT_LWT_RETAIN                ,    // MQTT client config: set LWT retain flag
    APP_CMD_GET_MQTT_CLIENT_LWT_RETAIN                ,    // MQTT client config: get LWT retain flag
    APP_CMD_ADD_MQTT_CLIENT_ALPN                      ,    // MQTT client config: add ALPN protocol
    APP_CMD_DEL_MQTT_CLIENT_ALPN                      ,    // MQTT client config: remove ALPN protocol
    APP_CMD_GET_MQTT_CLIENT_ALPN                      ,    // MQTT client config: get ALPN protocol list
    APP_CMD_GET_MQTT_CLIENT_ALPN_COUNT                ,    // MQTT client config: get ALPN protocol count
    APP_CMD_CLEAR_MQTT_CLIENT_ALPN                    ,    // MQTT client config: clear ALPN protocol list
    APP_CMD_SET_MQTT_CLIENT_SNI_HOST                  ,    // MQTT client config: set SNI server name
    APP_CMD_GET_MQTT_CLIENT_SNI_HOST                  ,    // MQTT client config: get SNI server name
    APP_CMD_SET_MQTT_CLIENT_CACERT                    ,    // MQTT client config: set server CA certificate
    APP_CMD_GET_MQTT_CLIENT_CACERT                    ,    // MQTT client config: get server CA certificate
    APP_CMD_SET_MQTT_CLIENT_CCERT                     ,    // MQTT client config: set client certificate
    APP_CMD_GET_MQTT_CLIENT_CCERT                     ,    // MQTT client config: get client certificate
    APP_CMD_SET_MQTT_CLIENT_CCKEY                     ,    // MQTT client config: set client private key
    APP_CMD_GET_MQTT_CLIENT_CCKEY                     ,    // MQTT client config: get client private key
    APP_CMD_SET_MQTT_CLIENT_PSK_DATA                  ,    // MQTT client config: set PSK key data
    APP_CMD_GET_MQTT_CLIENT_PSK_DATA                  ,    // MQTT client config: get PSK key data
    APP_CMD_SET_MQTT_CLIENT_PSK_HINT                  ,    // MQTT client config: set PSK key identity hint
    APP_CMD_GET_MQTT_CLIENT_PSK_HINT                  ,    // MQTT client config: get PSK key identity hint

    /*
     * BLE commands; codes start at 4000.
     */

    APP_CMD_SET_BLE_ADV_MFG_DATA                = 4000,    // BLE config: set advertising manufacturer data
    APP_CMD_GET_BLE_ADV_MFG_DATA                      ,    // BLE config: get advertising manufacturer data
    APP_CMD_SET_BLE_DEVICE_NAME                       ,    // BLE config: set device name
    APP_CMD_GET_BLE_DEVICE_NAME                       ,    // BLE config: get device name
    APP_CMD_SET_BLE_NOTIFY_RETRY_MAX                  ,    // BLE config: set notify retry max count
    APP_CMD_GET_BLE_NOTIFY_RETRY_MAX                  ,    // BLE config: get notify retry max count
    APP_CMD_SET_BLE_DEVICE_ADDR                       ,    // BLE config: set device address
    APP_CMD_GET_BLE_DEVICE_ADDR                       ,    // BLE config: get device address
    APP_CMD_SET_BLE_BONDING_ENABLE                    ,    // BLE config: set bonding enable
    APP_CMD_GET_BLE_BONDING_ENABLE                    ,    // BLE config: get bonding enable state
    APP_CMD_SET_BLE_BONDING_KEY                       ,    // BLE config: set bonding passkey (6-digit string)
    APP_CMD_GET_BLE_BONDING_KEY                       ,    // BLE config: get bonding passkey
    APP_CMD_GET_BLE_BONDED_DEVICE_NUMS                ,    // BLE config: get number of bonded devices
    APP_CMD_GET_BLE_BONDED_DEVICE_ADDR                ,    // BLE config: get bonded device address
    APP_CMD_DEL_BLE_BONDED_DEVICE                     ,    // BLE config: delete specified bonded device
    APP_CMD_CLEAR_BLE_BONDED                          ,    // BLE config: clear all bonded devices
    APP_CMD_SET_BLE_BATTERY_LEVEL                     ,    // BLE config: set battery level
    APP_CMD_GET_BLE_BATTERY_LEVEL                     ,    // BLE config: get battery level
    APP_CMD_SET_BLE_TX_POWER                          ,    // BLE config: set TX power level
    APP_CMD_GET_BLE_TX_POWER                          ,    // BLE config: get TX power level
    APP_CMD_GET_BLE_SPP_STATUS                        ,    // BLE control: get BLE SPP service status
    APP_CMD_START_BLE_SPP                             ,    // BLE control: start BLE SPP service
    APP_CMD_STOP_BLE_SPP                              ,    // BLE control: stop BLE SPP service
    APP_CMD_SET_BLE_ENABLE                            ,    // BLE control: set the persisted BLE switch, uint8_t 0=off 1=on; starts/stops the stack at once; response = stored state
    APP_CMD_GET_BLE_ENABLE                            ,    // BLE control: get the persisted BLE switch, uint8_t 0=off 1=on

    /*
     * Other general commands; codes start at 5000.
     */
    APP_CMD_SEND_FORWARD_DATA                   = 5000,    // Send forward data; destination (BLE or WiFi) depends on current forward mode config

} app_command_code_t;

#endif
