#ifndef CALIB_H_
#define CALIB_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CALIB_ {
	int16_t  axBias;
	int16_t  ayBias;
	int16_t  azBias;
	int16_t  gxBias;
	int16_t  gyBias;
	int16_t  gzBias;
	int16_t  mxBias;
	int16_t  myBias;
	int16_t  mzBias;
	int16_t  mxSens;
	int16_t  mySens;
	int16_t  mzSens;
} CALIB;	

#define MAX_CALIB_PARAMS 12

extern CALIB calib;
extern bool IsCalibrated;

int calib_init(void);
int calib_save();
void calib_setDefaults();

#ifdef __cplusplus
}
#endif
#endif
