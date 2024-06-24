import wifi
import time

wifiName = "AIDA"
wifiPassKey = "aida9319"

def main():
    wifiLoop(wifiName, wifiPassKey)


def wifiLoop(SSID, psw):
    # scan for available WiFi networks
    wifi_scanner = wifi.Cell.all('wlan0')
    available_networks = [cell.ssid for cell in wifi_scanner]

    # print available networks
    print(f"Available Networks: {available_networks}")

    # connect to a WiFi network
    network_ssid = SSID
    network_pass = psw

    while(not search(network_ssid, network_pass, wifi_scanner)):
        print("Sleeping")
        time.sleep(5)
    
def search(SSID, psw, wifiScanner):
    for cell in wifiScanner:
        if cell.ssid == SSID:
            scheme = wifi.Scheme.for_cell('wlan0', cell.ssid, cell, psw)
            scheme.save()
            scheme.activate()
            print(f"Connected to network: {SSID}")
            return True
    return False

main()