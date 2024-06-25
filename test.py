import wifi
import time
import os

wifiName = "AIDA"
wifiPassKey = "aida9319"

def main():
    wifiLoop(wifiName, wifiPassKey)


def wifiLoop(SSID, psw):
    # scan for available WiFi networks

    #available_networks = [cell.ssid for cell in wifi_scanner]

    # print available networks
    #print(f"Available Networks: {available_networks}")

    # connect to a WiFi network
    network_ssid = SSID
    network_pass = psw
    
    
    command = '/etc/network/interfaces'
    command2 = "scheme.activate"

    while(True):
    	wifi_scanner = wifi.Cell.all('wlan0')
    	print("Hello")
    	print(isinstance(wifi_scanner, map))
    	#available_networkss = [cell.ssid for cell in wifi_scanner]
    	#print(f"Available Networks: {available_networkss}")
    	print(wifi_scanner)
    	for cell in wifi_scanner:
    	    print("in for")
    	    if cell.ssid == SSID:
    	        print("in if")
    	        scheme = wifi.Scheme.for_cell('wlan0', cell.ssid, cell, psw)
    	        #os.system(f"sudo chmod 666 {command})
    	        scheme.save()
    	        scheme.activate()
    	        
    	        print(f"Connected to network: {SSID}")
    	        return True
    	        
    	print("Sleeping")
    	time.sleep(5)
    
def search(SSID, psw, wifiScanner):
    print("Hello")
    print(isinstance(wifiScanner, map))
    available_networkss = [cell.ssid for cell in wifiScanner]
    print(f"Available Networks: {available_networkss}")
    for cell in wifiScanner:
    	print("in for")
    	if str(value.ssid) == SSID:
    	    print("in if")
    	    scheme = wifi.Scheme.for_cell('wlan0', cell.ssid, cell, psw)
    	    scheme.save()
    	    scheme.activate()
    	    print(f"Connected to network: {SSID}")
    	    return True
    return False

main()
