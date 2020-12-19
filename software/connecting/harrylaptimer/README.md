# Connecting to Harry's LapTimer

More info at [https://www.gps-laptimer.de/](https://www.gps-laptimer.de/), where you can also find a very good user and developer forum. It offers the largest set of customizable options to build your own external device and it is the only supported platform for iOS phones via Bluetooth Low Energy.

![Example](hlt_lagunaseca.png)

Features:

* NMEA parsing offered by HLT directly: RMC+GGA messages
* Enable or Disable GSV/GSA as needed, with a lower update frequency selectable for 1, 2, or 5 sec (as [instructed by Harry](http://forum.gps-laptimer.de/viewtopic.php?t=4359))
* tested with v24

Connections
 
* iOS and Android: TCP/IP port 8118. This is the easier option, yet it prevents connecting other devices to your mobile phone (e.g. an action camera or an ODBII device)
* iOS: BLE atested on iPhone (tested with 7). BLE handles 5Hz and 10Hz GPS update rates when `GSV`/`GSA` streaming disabled, ok with polling at low frequencies (e.g. every few seconds)
* Android: BT-SPP tested on Android
 
After configuring the connection, your unit shows up in the Sensors List as 'Custom [BTLE/TCP-IP] GNSS' device

![](hlt-ble-configured.png)

now you can open the '**GPS**' page and check that you are receiving valid GPS data

![](hlt-ble-running.png)

with a paid add-on, you can also see the satellites constellation map

![](hlt-gps-map.png)


## iOS Bluetooth Low Energy setup

Go to *Administration > Settings > Expert Settings* and

* within 'accessory tweaks', turn on 'Use Bluetooth LE Sensors'
* within 'custom btle gps adapter', set 'Peripheral Name' to `BonoGPS-XXXX` where `XXXX` is the id of your device as shown in the title of the configuration page
* within 'custom btle gps adapter', set 'Service UUID' to  **1819** (standard for `SERVICE_LOCATION_AND_NAVIGATION_UUID`)
* within 'custom btle gps adapter', set 'Read Characteristic UUID' **2A67** (standard for `CHARACTERISTIC_LOCATION_AND_SPEED_CHARACTERISTIC_UUID`)

![](hlt-ble.jpeg)


## iOS and Android TCP/IP setup

Go to *Administration > Settings > Expert Settings* and

* within 'accessory tweaks', turn on 'Use Wi-Fi Sensors'
* within 'custom wifi gps adapter', set Connection Type `TCP/IP`, 'Device IP Address' to `10.0.0.1`, TCP Port to `1818`

![](hlt-wifi.jpeg)


## Android BT-SPP Setup

Go to *Sensor List* and

* Pair your bluetooth device as any BT device on your phone
* Click on the _Bluetooth +_ icon at the top of the page
* Select your newly paired BonoGPS device
* Specify _use as GPS_

![Configuration panel](hlt-btspp-configured.png)

Use the (i) icon to check the messages being exchanged

![List of received messages](hlt-btspp-check.png)

Now head to *GPS* to verify it is correctly working

![GPS page showing status and information received](hlt-btspp-running.png)

## GPS Configuration for NMEA messages

According [to this good reference message](http://forum.gps-laptimer.de/viewtopic.php?f=19&t=4359) on the forum

> I recommend to enable RMC+GGA+GSA+GSV, with RMC+GGA at 10 Hz and GSA+GSV at 1 Hz or lower.


* `GxGSA` and `GxGSV` are polled every 5 seconds (2.5 seconds each) with `$EIGNQ,GSA*2D` and `$EIGNQ,GSV*3A` respectively
* `GxGGA`, `GxRMC`, and `GxGBS` are streaming

HLT is the only app that easily works with the GN talker id, so no translation of Talker ID was necessary.