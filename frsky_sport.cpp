#ifndef __FRSKY_SPORT
#define __FRSKY_SPORT

#include "stdint.h"
#include "stm32f4xx_hal.h"


#define FRSKY_TELEMETRY_START_FRAME 0x7E
#define FRSKY_SENSOR_DATA_FRAME 0x10
#define FRSKY_STUFFING 0x7D

#define FCS_CURR_DATA_ID 0x0200
#define FCS_VOLT_DATA_ID 0x0210


#define FRSKY_MESSAGE_BUFFER_LENGTH 16
 


class FrskySport{
	public:		
	//контрольная сумма
    uint16_t crc;
  
    uint8_t sensorDataPtr;

    //предыдущий прочитанный байт
    uint8_t prevData;
	  
    //буфер для отправки сообщения телеметрии
    uint8_t messageBuffer[FRSKY_MESSAGE_BUFFER_LENGTH];
    //указатель на конец сообщения в буфере
    uint8_t messageBufferEnd; 
    //указатель на текущий байт в буфере
    uint8_t messageBufferCurrent;


    float telemetry_current;
    float telemetry_vbat;

	//конструктор
	FrskySport()
	{		
		crc = 0;		
		sensorDataPtr = 0;
		prevData = 0;
		messageBufferEnd = 0;
		messageBufferCurrent = 0;
	}
	
	//сброс указателей буфера
    void beginMessage()
	{		
		messageBufferEnd = messageBufferCurrent = 0;
	}
	
	
	void addByteToBuffer(uint8_t byte)
	{
		if(messageBufferEnd >= FRSKY_MESSAGE_BUFFER_LENGTH) return;
		messageBuffer[messageBufferEnd++] = byte;
	}
	
	
	
	

	void addByte(uint8_t byte)
	{  
			if(byte == 0x7E)
			{
				addByteToBuffer(FRSKY_STUFFING);
				addByteToBuffer(0x5E); // 0x7E xor 0x20
			}
			else if(byte == 0x7D)
			{
				addByteToBuffer(FRSKY_STUFFING);
				addByteToBuffer(0x5D); // 0x7D xor 0x20
			}
			else
			{
				addByteToBuffer(byte);
			}
			crc += byte;
			crc += crc >> 8; crc &= 0x00ff;  
	}

	void addCrc()
	{
		// Send and reset CRC
		addByte(0xFF - crc);
		crc = 0;
	}


	void sportSetMessage(uint16_t dataTypeId, uint32_t data)
	{  
		beginMessage();
		
		addByte(FRSKY_SENSOR_DATA_FRAME);
		
		uint8_t *bytes = (uint8_t*)&dataTypeId;
		addByte(bytes[0]);
		addByte(bytes[1]);
		
		bytes = (uint8_t*)&data;
		addByte(bytes[0]);
		addByte(bytes[1]);
		addByte(bytes[2]);
		addByte(bytes[3]);
		
		addCrc();		
	}


	void sportSetEmptyMessage(uint16_t dataTypeId)
	{
		beginMessage();
		
		addByte(0x00);
		
		uint8_t *bytes = (uint8_t*)&dataTypeId;
		addByte(bytes[0]);
		addByte(bytes[1]);
		
		for(uint8_t i = 0; i < 4; i++) addByte(0x00);
			
		addCrc();		
	}



	bool process(uint8_t data)
	{	
		bool result = false;
					
		//начало запроса на телеметрию
		if(prevData == FRSKY_TELEMETRY_START_FRAME)
		{
			switch(data)
			{
				//currentSensorID 0x22
				case 0x22:
					if(sensorDataPtr == 0) sportSetMessage(FCS_CURR_DATA_ID, (uint32_t)(telemetry_current * 10));
					if(sensorDataPtr == 1) sportSetMessage(FCS_VOLT_DATA_ID, (uint32_t)(telemetry_vbat * 100));
                    if(sensorDataPtr == 2) sportSetMessage(0x0700, (uint32_t)(12345));
                    
					sensorDataPtr++;
					if(sensorDataPtr >= 3) sensorDataPtr = 0;
					result = true;
					break;
			}			
		}
		
		prevData = data;
		
		return result;
	}

};

#endif /*__FRSKY_SPORT*/

