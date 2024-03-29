# Offline software for extracting IMU data and GPS track logs

## Software environment

Ubuntu 20.04LTS amdx64 platform

## Downloading and extracting data and gps logs

Put the gpsvario into wifi AP+webserver mode, connect to the WiFi access point
`ESP32GpsVario`, and download the binary datalog file using the url  http://esp32.local/datalog

The datalog may contain a combination of high-speed IBG (IMU+Baro+GPS) data logs and 
normal GPS track logs. 

Use the `logsplit` program to split the file into separate IBG and GPS data logs.

`logsplit` separates ibg logs wherever the timestamp of two consecutive log entries differs by more 
than the specified value in milliseconds (e.g. 300). Also splits wherever there is a change from IBG log to GPS track log or vice versa.

```
gcc -o logsplit logsplit.c
./logsplit ~/Downloads/datalog 300     
```

`gpslog2gpx` will convert a GPS binary log file into a `.gpx` text file that you
can load in Google Earth or other GPS track visualization software.

In Google Earth, remember to uncheck the _clamp to ground_ option before opening a `.gpx` file if you want to see the track in 3D.

```
gcc -o gpslog2gpx gpslog2gpx.c
./gpslog2gpx ~/Downloads/datalog
```

The IBG datalogs also contain gps fix data @ 10Hz. `ibglog2gpx` can be used to convert the gps fix data into gpx files that you can open in Google Earth or other 3D track visualization software.  This
is of limited use as a GPS flight log - if you enable IBG data logging, the 16Mbyte spi flash will fill up in ~13 minutes. IBG data logging is meant for offline analysis of IMU+GPS data for algorithm development.


```
gcc -o ibglog2gpx ibglog2gpx.c
./ibglog2gpx ~/Downloads/datalog
```

Display the IBG data in text format

```
gcc -o parseIBG parseIBG.c
./parseIBG ~/Downloads/datalog
```

`ibglogsplit` is used to split the downloaded binary datalog file into separate logs.
In this example, timestamps that are separated by more than 300mS result in a
separate log file being generated. Output files are `~/Downloads/datalog_00, ...` 

```
gcc -o ibglogsplit ibglogsplit.c
./ibglogsplit ~/Downloads/datalog 300     
```
## Offline analysis : Visualizaton of orientation

Compute the real-time orientation quaternion from the IBG datalog. Use `quatsee.py`
to visualize the orientation

```
gcc -o ibglog2quat ibglog2quat.c imu.c -lm
./ibglog2quat ~/Downloads/datalog
```
## Offline analysis : Kalman filter sensor fusion to compute altitude and climbrate

`/kf/kf_compare.cpp` is an example of off-line analysis of a downloaded IBG data log. It compares
different kalman filter options running on the downloaded data. 

This [Jupyter notebook](kf/compare_kf2_kf3_kf4.ipynb) visualizes the results for KF2 (only pressure sensor data), KF3 and KF4 algorithms.


## Validation of FormatGEO route files

`testroute` tests the validity of a **FormatGEO** route file generated by [xcplanner](
https://github.com/dkm/xcplanner). 

**xcplanner** does not specify waypoint radii in the route file. You can edit the
`.wpt` file to add a waypoint radius (in meters) at the end of a waypoint description line. 
If there is no waypoint radius specified for a waypoint, the gpsvario will assign a 
user-configurable waypoint radius. Once you have validated the .wpt file, you can upload
it to the gpsvario using the webpage server `upload file` button.

The total route distance computed and displayed is NOT a competition-optimized 'grazing' distance from the waypoint circles. It computes the total distance between waypoint centre points, ignoring the waypoint radius.

```
gcc -o testroute testroute.c -lm
./testroute example.wpt
```