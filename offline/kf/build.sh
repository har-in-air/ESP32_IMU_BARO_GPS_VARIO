g++ -o kf_run kf_run.cpp kalmanfilter2.cpp kalmanfilter3.cpp kalmanfilter4.cpp kalmanfilter4d.cpp ../../src/sensor/imu.cpp \
../../src/sensor/ringbuf.cpp -I../../src/sensor -I. -lm

