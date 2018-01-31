#ifndef __FRSKY_SPORT
#define __FRSKY_SPORT

#include "stdint.h"
#include "stm32f4xx_hal.h"



#define FRSKY_TELEMETRY_START_FRAME 0x7E
#define FRSKY_SENSOR_DATA_FRAME 0x10
#define FRSKY_STUFFING 0x7D


#define	FRSKY_ALT_ID          0x0100
#define	FRSKY_VARIO_ID        0x0110
#define	FRSKY_CURR_ID         0x0200
#define	FRSKY_VFAS_ID         0x0210
#define	FRSKY_CELLS_ID        0x0300
#define	FRSKY_T1_ID           0x0400
#define	FRSKY_T2_ID           0x0410
#define	FRSKY_RPM_ID          0x0500
#define	FRSKY_FUEL_ID		  0x0600
#define	FRSKY_ACCX_ID         0x0700
#define	FRSKY_ACCY_ID         0x0710
#define	FRSKY_ACCZ_ID         0x0720
#define	FRSKY_GPS_LON_LAT_ID  0x0800
#define	FRSKY_GPS_ALT_ID      0x0820
#define	FRSKY_GPS_SPEED_ID    0x0830
#define	FRSKY_GPS_COURSE_ID   0x0840
#define	FRSKY_GPS_TIME_ID     0x0850
#define	FRSKY_ADC3_ID         0x0900
#define	FRSKY_ADC4_ID         0x0910
#define	FRSKY_AIR_SPEED_ID    0x0a00
#define	FRSKY_RSSI_ID         0xf101
#define	FRSKY_ADC1_ID         0xf102
#define	FRSKY_ADC2_ID         0xf103
#define	FRSKY_BATT_ID         0xf104
#define	FRSKY_SWR_ID          0xf105



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
    float telemetry_acc_x;
    float telemetry_acc_y;
    float telemetry_acc_z;
    float telemetry_alt;
    int32_t telemetry_gps_lon;
    int32_t telemetry_gps_lat;
    
    

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
					if(sensorDataPtr == 0) sportSetMessage(FRSKY_CURR_ID, (uint32_t)(telemetry_current * 10));
					if(sensorDataPtr == 1) sportSetMessage(FRSKY_VFAS_ID, (uint32_t)(telemetry_vbat * 100));
                    if(sensorDataPtr == 2) sportSetMessage(FRSKY_ACCX_ID, (int32_t)(telemetry_acc_x * 100));
                    if(sensorDataPtr == 3) sportSetMessage(FRSKY_ACCY_ID, (int32_t)(telemetry_acc_y * 100));
                    if(sensorDataPtr == 4) sportSetMessage(FRSKY_ACCZ_ID, (int32_t)(telemetry_acc_z * 100));
                    if(sensorDataPtr == 5) sportSetMessage(FRSKY_ALT_ID,  (int32_t)(telemetry_alt * 100 + 810)); //TODO: Шо за 810?
                    if(sensorDataPtr == 6) sportSetMessage(FRSKY_GPS_LON_LAT_ID, (int32_t)get_gps_lon_lat(true));
                    if(sensorDataPtr == 7) sportSetMessage(FRSKY_GPS_LON_LAT_ID, (int32_t)get_gps_lon_lat(false));
                                        
					sensorDataPtr++;
					if(sensorDataPtr >= 8) sensorDataPtr = 0;
					result = true;
					break;
			}			
		}
		
		prevData = data;
		
		return result;
	}
    
    
    uint32_t get_gps_lon_lat(bool lat)
    {
        uint32_t frsky_coord = 0;
        int32_t coord = 0;

        if (lat) {
            // lattitude
            coord = telemetry_gps_lat;
            if (coord >= 0)
                frsky_coord = 0;
            else
                frsky_coord = 1 << 30;
        } else {
            // longitude
            coord = telemetry_gps_lon;
            if (coord >= 0)
                frsky_coord = 2 << 30;
            else
                frsky_coord = 3 << 30;
        }
        if(coord < 0) coord = coord * -1; 
        frsky_coord |= (((uint64_t)coord * 6ull) / 100);

        return frsky_coord;
    }

};

#endif /*__FRSKY_SPORT*/

