## Tested on Ubuntu 16.04LTS amdx64 platform

gcc -o gpslog2gpx gpslog2gpx.c
./gpslog2gpx ~/Downloads/datalog

gcc -o ibglog2gpx ibglog2gpx.c
./ibglog2gpx ~/Downloads/datalog

gcc -o parseIBG parseIBG.c
./parseIBG ~/Downloads/datalog

gcc -o ibglogsplit ibglogsplit.c
./ibglogsplit ~/Downloads/datalog 300     

// called with 300mS argument, so timestamps that are separated by more than 300mS result in a
// different log file being generated. Output files are ~/Downloads/datalog_00, ... 

gcc -o ibglog2quat ibglog2quat.c imu.c -lm
./ibglog2quat ~/Downloads/datalog

gcc -o logsplit logsplit.c
./logsplit ~/Downloads/datalog 300     

// separates ibg logs wherever the timestamp of two consecutive logs differs by more than the specified value
// milliseconds (e.g. 300). Also splits wherever there is a change from ibg log to gps track log or vice versa.

gcc -o testmar2 testmar2.c kalmanfilter3.c imu.c ringbuf.c -lm
./testmar2 ~/Downloads/datalog-mar2

gcc -o testroute testroute.c -lm
./testroute example.wpt
