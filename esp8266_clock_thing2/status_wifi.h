//
// Decode wifi status and return string answer
//

char unknown_wifi_status[8];
char * wifi_status_string() {
  switch(WiFi.status()) {
    case WL_IDLE_STATUS:	return "idle";
    case WL_NO_SSID_AVAIL:	return "SSIDs unreachable";
    case WL_CONNECTED:		return "connected";
    case WL_CONNECT_FAILED:	return "connection failed";
    case WL_WRONG_PASSWORD:	return "wrong password";
    case WL_DISCONNECTED:	return "disconnected";
    default:			sprintf(unknown_wifi_status, "? %4d", WiFi.status());
				return unknown_wifi_status;
  }
}
