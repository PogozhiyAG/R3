#ifndef __LIS331DLH
#define __LIS331DLH

#include "stm32f4xx_hal.h"
#include "i2c_sensor.cpp"


//адрес
#define LIS331DLH_I2C_ADDRESS       0x30
 
//регистры
#define WHO_AM_I_REG_LIS331DLH      0x0F
#define LIS331DLH_XOUT_H_REG 				0x29
#define LIS331DLH_XOUT_L_REG			  0x28
#define LIS331DLH_YOUT_H_REG 				0x2B
#define LIS331DLH_YOUT_L_REG 				0x2A
#define LIS331DLH_ZOUT_H_REG 				0x2D
#define LIS331DLH_ZOUT_L_REG				0x2C
 
#define LIS331DLH_CTRL_REG_1      	0x20
#define LIS331DLH_CTRL_REG_2      	0x21
#define LIS331DLH_CTRL_REG_3      	0x22
#define LIS331DLH_CTRL_REG_4      	0x23
#define LIS331DLH_CTRL_REG_5      	0x24
 
#define LIS331DLH_STATUS_REG      	0x27
 
 
//режимы
#define POWER_DOWN   								0x6F
#define NORMAL_50HZ   							0x27
#define NORMAL_100HZ  				 			0x2F
#define NORMAL_400HZ   							0x37
#define NORMAL_1000HZ   						0x3F
#define LOW_POWER_0_5HZ   					0x47
#define LOW_POWER_1HZ   						0x67
#define LOW_POWER_2HZ   						0x87
#define LOW_POWER_5HZ   						0xA7
#define LOW_POWER_10HZ   						0xC7


class LIS331DLH : public I2CSensor3AxisLPF
{			
	public:
        Vector3D offset;
        Vector3D scaleFactor[3];
        Vector3D scaledResult;
    
		LIS331DLH(I2C_HandleTypeDef* _hi2c, float _lpfK):I2CSensor3AxisLPF(_hi2c, LIS331DLH_I2C_ADDRESS, LIS331DLH_XOUT_L_REG, _lpfK)
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


#endif /*__LIS331DLH*/
