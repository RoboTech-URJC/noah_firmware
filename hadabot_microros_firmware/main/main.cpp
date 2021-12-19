#include "uxr/client/config.h"
#include "nvs_flash.h"
#include "WiFi.h"

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

extern "C" void microros_interface_init();
extern "C" void host_sntp_sync_time(void);

void wifi_init_sta()
{
  WiFi.begin(ESP_WIFI_SSID, ESP_WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

extern "C" void app_main(void)
{   
  // Start networkign if required
  #ifdef UCLIENT_PROFILE_UDP
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    //host_sntp_sync_time();
  #endif  // UCLIENT_PROFILE_UDP
	microros_interface_init(); // Micro-ros app enter point
}
