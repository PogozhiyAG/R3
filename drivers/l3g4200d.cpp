#ifndef __L3G4200D
#define __L3G4200D

#include "stm32f4xx_hal.h"
#include <stdint.h>

#include "i2c_sensor.cpp"
#include "geometry.cpp"


#define L3G4200D_ADDRESS       0xD0

#define L3G4200D_WHO_AM_I      0x0F
 
#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG2     0x21
#define L3G4200D_CTRL_REG3     0x22
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_CTRL_REG5     0x24
#define L3G4200D_REFERENCE     0x25
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27
 
#define L3G4200D_OUT_X_L       0x28
#define L3G4200D_OUT_X_H       0x29
#define L3G4200D_OUT_Y_L       0x2A
#define L3G4200D_OUT_Y_H       0x2B
#define L3G4200D_OUT_Z_L       0x2C
#define L3G4200D_OUT_Z_H       0x2D
 
#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_FIFO_SRC_REG  0x2F
 
#define L3G4200D_INT1_CFG      0x30
#define L3G4200D_INT1_SRC      0x31
#define L3G4200D_INT1_THS_XH   0x32
#define L3G4200D_INT1_THS_XL   0x33
#define L3G4200D_INT1_THS_YH   0x34
#define L3G4200D_INT1_THS_YL   0x35
#define L3G4200D_INT1_THS_ZH   0x36
#define L3G4200D_INT1_THS_ZL   0x37
#define L3G4200D_INT1_DURATION 0x38


#define L3G4200D_RANGE_250         0
#define L3G4200D_RANGE_500         1
#define L3G4200D_RANGE_2000        2



class L3G4200D : public I2CSensor3AxisLPF
{	
	public:		
	  uint8_t range;
	  double rpsRate;		
	  Vector3D offset;
      Vector3D prevResult;
	
	
		L3G4200D(I2C_HandleTypeDef* _hi2c, float _lpfK) : I2CSensor3AxisLPF(_hi2c, L3G4200D_ADDRESS, L3G4200D_OUT_X_L, _lpfK)
		{
			offset.X = offset.Y = offset.Z = 0.0f;
		}
		
		
		HAL_StatusTypeDef setRange(uint8_t _range, bool BDU, uint32_t timeout){
			range = _range;
			char value;
			
			switch (range)
			{
				case L3G4200D_RANGE_250:  
					value = 0x00;
					rpsRate = 0.00875 * 0.017453293;
					break;
			  case L3G4200D_RANGE_500:  
					value = 0x10;
					rpsRate = 0.0175 * 0.017453293;
				  break;
			  case L3G4200D_RANGE_2000: 
					value = 0x20;
					rpsRate = 0.07 * 0.017453293;
				  break;
			}
            
            //BDU
            value = (BDU << 7) | value; 
            
			return writeRegister(L3G4200D_CTRL_REG4, value, timeout);			
		}
		
		
		void calibrate(int times, int pause_ms, uint32_t timeout)
		{	
			Vector3D v;
			for(int i = 0; i < times; i++)
			{
				readAxis(timeout);
				
				v.X += axis[0] * 1.0f;
				v.Y += axis[1] * 1.0f;
				v.Z += axis[2] * 1.0f;
				
				HAL_Delay(pause_ms);
			}
			offset.X = v.X / times;
			offset.Y = v.Y / times;
			offset.Z = v.Z / times;
		}
		
		
		virtual void processResult()
        {
            //TODO: Пересмотреть методы
			processAxis();
            
			result.X = (1 - lpfK) * result.X + lpfK * (axis[0] - offset.X) * rpsRate;
			result.Y = (1 - lpfK) * result.Y + lpfK * (axis[1] - offset.Y) * rpsRate;
			result.Z = (1 - lpfK) * result.Z + lpfK * (axis[2] - offset.Z) * rpsRate;
            
		}
};




#endif /*__L3G4200D*/









