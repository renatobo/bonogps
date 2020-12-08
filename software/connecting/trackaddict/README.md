# TrackAddict

Another very popular app, [https://www.hptuners.com/product/trackaddict-app/](https://www.hptuners.com/product/trackaddict-app/)

  - tested with v4.6.0 on Android
  - BT-SPP is the only option
  - it needs a specific set of NMEA messages configuration, available as option
  - NBP was coded, yet not going to matter for TrackAddict as reported in [the forum](https://forum.hptuners.com/showthread.php?78403-GPS-over-NBP&highlight=gps)
  - Notes for the Classic BT-SPP version to optimize transfer to TrackAddict on Android
    - Track Addict is similarly configured to only accept NMEA messages with a GPS talker ID (i.e., `$GPRMC` instead of `$GNRMC`) [as discussed in the forum](https://forum.hptuners.com/showthread.php?69123-Track-addict-external-GPS&highlight=gps)
    - Ideally, TrackAddict wants `$GPRMC`, `$GPGGA`, and `$GPGLL` messages, with `$GPRMC` and `$GPGGA` being the recommended minimum.

Please make sure that your BonoGPS unit is connected before recording a session, it will default back to internal GPS without informing you otherwise in a clear way.
    
## Configuration

In the main page, click on the gear icon, scroll down to "Input position and speed" and select Bluetooth GPS

![](trackaddict_setup.png)

Assuming everything well, the main page will now show the new refreshed rate of your BonoGPS unit

![](trackaddict_connected.png)

You can click on the green line at the bottom to find out more and validate the connection parameters

![](trackaddict_check.png)