
#ifndef CALIBRATION_H
#define CALIBRATION_H


enum CalibrateResult
{
	CLBT_SUCCESS_ONE_LASER,
        CLBT_SUCCESS_TWO_LASER,
	CLBT_OPEN_DX_FAILED,
	CLBT_LESS_THAN_FIVE_CIRCLES,	
	CLBT_MORE_OVERSPEED_DATA,
	CLBT_DX_DATA_LESS,
	CLBT_LASER_NUM_ZERO,
	CLBT_MATCH_PERCENT_LOW,
	CLBT_NO_MATCH_RESULT,
        CLBT_LASER_NUM_MODRE,

};




//上位机传入激光可以范围
extern "C"  void setRange(float minRange, float maxRange);

extern "C" int CalibrateLaser(char* dxFilePath,  double *res, int laserType);





#endif 
