# Comparison of complete PVT solutions

Right now we use `RMC`+`GGA`+`GBS` at high resolution, then `GSA`+`GSV` at lower resolution. Are there less expensive (in terms of bandwidth) options?

## NMEA standard messages `RMC`+`GGA`+`GSA`+`GSV`+`GBS`


```
22:23:20  R -> NMEA GNRMC,  Size  70,  'Recommended Minimum Specific GNSS Data'
22:23:20  R -> NMEA GNGGA,  Size  79,  'Global Positioning System Fix Data'
22:23:20  R -> NMEA GNGSA,  Size  66,  'GNSS DOP and Active Satellites'
22:23:20  R -> NMEA GNGSA,  Size  44,  'GNSS DOP and Active Satellites'
22:23:20  R -> NMEA GNGSA,  Size  44,  'GNSS DOP and Active Satellites'
22:23:20  R -> NMEA GPGSV,  Size  72,  'GNSS Satellites in View'
22:23:20  R -> NMEA GPGSV,  Size  72,  'GNSS Satellites in View'
22:23:20  R -> NMEA GPGSV,  Size  72,  'GNSS Satellites in View'
22:23:20  R -> NMEA GPGSV,  Size  31,  'GNSS Satellites in View'
22:23:20  R -> NMEA GLGSV,  Size  66,  'GNSS Satellites in View'
22:23:20  R -> NMEA GLGSV,  Size  64,  'GNSS Satellites in View'
22:23:20  R -> NMEA GLGSV,  Size  53,  'GNSS Satellites in View'
22:23:20  R -> NMEA GAGSV,  Size  70,  'GNSS Satellites in View'
22:23:20  R -> NMEA GAGSV,  Size  70,  'GNSS Satellites in View'
22:23:20  R -> NMEA GNGBS,  Size  39,  'Satellite fault Detection'
```

Total of 912 bytes for a complete set

## PUBX format

This format is interpreted by some libraries, such as [NeoGPS](https://github.com/SlashDevin/NeoGPS), perhaps with the exception of `PUBX03` as it's not listed

```
22:25:53  R -> NMEA PUBX00,  Size 110,  'Position Data'
22:25:53  R -> NMEA PUBX03,  Size 607,  'Satellite Data'
22:25:53  R -> NMEA PUBX04,  Size  68,  'Time of Day'
```

Total of 785 bytes for a complete set

## ublox Binary format

This format is interpreted by some libraries, such as [NeoGPS](https://github.com/SlashDevin/NeoGPS), with the exception of `NAV-SAT` not listed

```
22:14:15  R -> UBX NAV-PVT,  Size 100,  'Navigation PVT Solution'
22:14:15  R -> UBX NAV-SAT,  Size 388,  'Satellite Status and Information'
```

Total of 488 bytes for a complete set

- [https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavPVT.msg](https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavPVT.msg)
- [https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSAT.msg](https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSAT.msg)
- [https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSAT_SV.msg](https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSAT_SV.msg)

### Possible alternative to NAV-SAT: NAV-SVINFO (same size)

This is listed in [ublox support for NeoGPS](https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/ublox.md)

- [https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSVINFO.msg](https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSVINFO.msg)
- [https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSVINFO_SV.msg](https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavSVINFO_SV.msg)

### Likely not needed as included in NAV-PVT
```
22:27:15  R -> UBX NAV-TIMEUTC,  Size  28,  'Universal Time Coordinated'
```

- [https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavTIMEUTC.msg](https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavTIMEUTC.msg)




