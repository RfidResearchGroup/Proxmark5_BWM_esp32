#include "esp_log.h"
#include "app_wifi_netif_cfg.h"


#define GET_NETIF(_ifname) \
    esp_netif_t *_ifname = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); \
    if (NULL == _ifname) return ESP_ERR_INVALID_STATE; \


/**
 * @brief Get WiFi DHCP status
 * 
 * @param status DHCP client status
 * @return esp_err_t 
 */
esp_err_t app_wifi_cfg_get_dhcp_status(esp_netif_dhcp_status_t *status) {
    GET_NETIF(sta_netif);
    return esp_netif_dhcpc_get_status(sta_netif, status);
}

/**
 * @brief Set WiFi DHCP status
 * 
 * @param status DHCP client status
 * @return esp_err_t 
 */
esp_err_t app_wifi_cfg_set_dhcp_enable(bool enable) {
    GET_NETIF(sta_netif);
    if (enable) {
        return esp_netif_dhcpc_start(sta_netif);
    } else {
        return esp_netif_dhcpc_stop(sta_netif);
    }
}

/**
 * @brief Set WiFi IP address information
 * 
 * @param ipaddr IP address
 * @param gwaddr Gateway address
 * @param netmask Subnet mask address
 * @return esp_err_t 
 */
esp_err_t app_wifi_cfg_set_ipv4(uint32_t ipaddr, uint32_t gwaddr, uint32_t netmask) {
    GET_NETIF(sta_netif);
    esp_netif_ip_info_t info_t = { 0x00 };
    esp_netif_dhcpc_stop(sta_netif); // Configuring static address will disable DHCP
    // Set address, gateway and subnet mask
    info_t.ip.addr = ipaddr;
    info_t.gw.addr = gwaddr;
    info_t.netmask.addr = netmask;
    // Call lower layer interface to set
    esp_err_t err = esp_netif_set_ip_info(sta_netif, &info_t);
    if (err != ESP_OK || gwaddr == 0) {
        return err;
    }
    // No DHCP means no DNS server: use the gateway, as most routers forward DNS
    esp_netif_dns_info_t dns = { 0x00 };
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = gwaddr;
    return esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns);
}

/**
 * @brief Get WiFi IP address information
 * 
 * @param ipaddr IP address
 * @param gwaddr Gateway address
 * @param netmask Subnet mask address
 * @return esp_err_t 
 */
esp_err_t app_wifi_cfg_get_ipv4(uint32_t *ipaddr, uint32_t *gwaddr, uint32_t *netmask) {
    GET_NETIF(sta_netif);
    esp_netif_ip_info_t info_t;
    // Call lower layer interface to get
    esp_err_t err = esp_netif_get_ip_info(sta_netif, &info_t);
    if (err != ESP_OK) {
        return err;
    }
    *ipaddr = info_t.ip.addr;
    *gwaddr = info_t.gw.addr;
    *netmask = info_t.netmask.addr;
    return ESP_OK;
}

/**
 * @brief Set WiFi hostname
 * 
 * @param hostname 
 * @return esp_err_t 
 */
esp_err_t app_wifi_cfg_set_host_name(const char *hostname) {
    GET_NETIF(sta_netif);
    return esp_netif_set_hostname(sta_netif, hostname);
}

/**
 * @brief Get WiFi hostname
 * 
 * @param hostname Secondary pointer, output hostname pointer; if NULL no hostname set
 * @return esp_err_t 
 */
esp_err_t app_wifi_cfg_get_host_name(const char **hostname) {
    GET_NETIF(sta_netif);
    *hostname = NULL;
    return esp_netif_get_hostname(sta_netif, hostname);
}
