#ifndef __LIS3MDL
#define __LIS3MDL

#include "stm32f4xx_hal.h"
#include "i2c_sensor.cpp"



#define LIS3MDL_I2C_ADDRESS 	0x38

#define LIS3MDL_CTRL_REG1       0x20
#define LIS3MDL_CTRL_REG2       0x21
#define LIS3MDL_CTRL_REG3       0x22
#define LIS3MDL_CTRL_REG4       0x23
#define LIS3MDL_CTRL_REG5       0x24

#define LIS3MDL_OUT_X_L           0x28
#define LIS3MDL_OUT_X_H           0x29
#define LIS3MDL_OUT_Y_L           0x2A
#define LIS3MDL_OUT_Y_H           0x2B
#define LIS3MDL_OUT_Z_L           0x2C
#define LIS3MDL_OUT_Z_H           0x2D


class LIS3MDL : public I2CSensor3AxisLPF
{
	public:
		Vector3D offset;
	  Vector3D scaleFactor[3];
	  Vector3D scaledResult;
	
		LIS3MDL(I2C_HandleTypeDef* _hi2c, float _lpfK):I2CSensor3AxisLPF(_hi2c, LIS3MDL_I2C_ADDRESS, LIS3MDL_OUT_X_L, _lpfK)
		{
			offset.Set(0,0,0);
			
			scaleFactor[0].Set(1,1,1);
			scaleFactor[1].Set(1,1,1);
			scaleFactor[2].Set(1,1,1);
			
			scaledResult.Set(1,1,1);
		}
		
		
		virtual void processResult()
		{
			processAxis(); 
			
			float xt_raw = axis[0] - offset.X;
			float yt_raw = axis[1] - offset.Y;
			float zt_raw = axis[2] - offset.Z;
			
			scaledResult.X = scaleFactor[0].X * xt_raw + scaleFactor[1].X * yt_raw + scaleFactor[2].X * zt_raw;
			scaledResult.Y = scaleFactor[0].Y * xt_raw + scaleFactor[1].Y * yt_raw + scaleFactor[2].Y * zt_raw;
			scaledResult.Z = scaleFactor[0].Z * xt_raw + scaleFactor[1].Z * yt_raw + scaleFactor[2].Z * zt_raw;
			
			result.X = (1 - lpfK) * result.X + lpfK * scaledResult.X;
			result.Y = (1 - lpfK) * result.Y + lpfK * scaledResult.Y;
			result.Z = (1 - lpfK) * result.Z + lpfK * scaledResult.Z;
		}
};



#endif /*__LIS3MDL*/
