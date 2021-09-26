# Offline analysis of pressure sensor raw data and altitude noise

To characterize the barometric pressure sensor noise, 512 pressure and pressure-derived altitude samples were logged with the gps-vario at rest. 

This [Jupyter notebook](pressure_sensor_noise_test.ipynb) demonstrates that both pressure sensor raw data and derived altitude readings display zero-mean Gaussian nature. 

Note that when computing noise variance for a barometric pressure sensor, it is recommended not to use more than a second or two of samples, due to environmental and sensor drift. At 50 samples/second, we are logging more than 10 seconds of samples. So we 'de-trend' the data before analyzing the noise characteristics.
