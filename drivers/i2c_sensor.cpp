#ifndef __I2CSENSOR
#define __I2CSENSOR

#include "stm32f4xx_hal.h"

#include "geometry.cpp"
#include "utils.cpp"


//датчик i2c
class I2CSensor
{
	public:
		I2C_HandleTypeDef* hi2c;	
		uint16_t address;		
	
	    I2CSensor(I2C_HandleTypeDef* _hi2c, uint16_t _address) : hi2c(_hi2c), address(_address){}
	
		//записать значение в регистр
		HAL_StatusTypeDef readRegister(uint16_t reg, uint8_t* pValue, uint32_t timeout)
		{	
            return HAL_I2C_Mem_Read(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, pValue, 1, timeout);
		}
		
		//прочитать значение из регистра
		HAL_StatusTypeDef writeRegister(uint16_t reg, uint8_t value, uint32_t timeout)
		{
			return HAL_I2C_Mem_Write(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, timeout);
		}	
};


//трехосевой датчик i2c
class I2CSensor3Axis : public I2CSensor
{	
	public:	
		uint16_t firstAxisRegisterAddress;
		uint8_t data[6];
		int16_t axis[3];
					
		
		I2CSensor3Axis(I2C_HandleTypeDef* _hi2c, uint16_t _address, uint16_t _firstAxisRegisterAddress) : I2CSensor(_hi2c, _address), firstAxisRegisterAddress(_firstAxisRegisterAddress)
		{
			axis[0] = axis[1] = axis[2] = 0;			
		}
		
		HAL_StatusTypeDef beginReadAxisDMA()
		{
			return HAL_I2C_Mem_Read_DMA(hi2c, address, firstAxisRegisterAddress | (1 << 7), I2C_MEMADD_SIZE_8BIT, data, 6);
		}
		
		HAL_StatusTypeDef readAxis(uint32_t timeout)
		{
			HAL_StatusTypeDef result = HAL_I2C_Mem_Read(hi2c, address, firstAxisRegisterAddress | (1 << 7), I2C_MEMADD_SIZE_8BIT, data, 6, timeout);
			if(result == HAL_OK)
			{
				processAxis();
			}
			return result;
		}
		
		void processAxis()
		{
				axis[0] = (int16_t) (data[1] << 8 | data[0]);
				axis[1] = (int16_t) (data[3] << 8 | data[2]);
				axis[2] = (int16_t) (data[5] << 8 | data[4]);
		}		
		
};


//трехосевой датчик i2c с фильтром низких частот
class I2CSensor3AxisLPF : public I2CSensor3Axis
{
	public:
		//коэффициент фильтра низких частот
		float lpfK;
		//результ
		Vector3D result;
		
		I2CSensor3AxisLPF(I2C_HandleTypeDef* _hi2c, uint16_t _address, uint16_t _firstAxisRegisterAddress, float _lpfK) : I2CSensor3Axis(_hi2c, _address, _firstAxisRegisterAddress), lpfK(_lpfK)
		{
			result.Set(0.0f, 0.0f, 0.0f);
		}
		
		virtual void processResult()
		{
			
		}
	
};


class I2CSensor3AxisRange : public I2CSensor3AxisLPF
{
	public:		
		Vector3D offsetMin;
		Vector3D offsetMax;
		Vector3D mappedAxis;
		float rangeMin;
		float rangeMax;
	
	
		I2CSensor3AxisRange(I2C_HandleTypeDef* _hi2c, uint16_t _address, uint16_t _firstAxisRegisterAddress, float _lpfK) : I2CSensor3AxisLPF(_hi2c, _address, _firstAxisRegisterAddress, _lpfK)
		{
			offsetMin.Set(0.0f, 0.0f, 0.0f);
			offsetMax.Set(0.0f, 0.0f, 0.0f);
			
			rangeMin = -1.0f;
			rangeMax =  1.0f;
		}
		
		
		virtual void  processResult()
		{
				processAxis(); 
			
				mappedAxis.X = mapf(axis[0] * 1.0f, offsetMin.X, offsetMax.X, rangeMin, rangeMax);
				mappedAxis.Y = mapf(axis[1] * 1.0f, offsetMin.Y, offsetMax.Y, rangeMin, rangeMax);
				mappedAxis.Z = mapf(axis[2] * 1.0f, offsetMin.Z, offsetMax.Z, rangeMin, rangeMax);
			
				result.X = (1 - lpfK) * result.X + lpfK * mappedAxis.X;
				result.Y = (1 - lpfK) * result.Y + lpfK * mappedAxis.Y;
				result.Z = (1 - lpfK) * result.Z + lpfK * mappedAxis.Z;
		}
	
	
};



#endif /*__I2CSENSOR*/
