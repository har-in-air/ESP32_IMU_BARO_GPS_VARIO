g++ -o kf_run kf_run.cpp kalmanfilter2.cpp kalmanfilter3.cpp \
../../src/sensor/kalmanfilter4.cpp ../../src/sensor/imu.cpp \
../../src/sensor/ringbuf.cpp -I../../src/sensor -I. -lm

