#ifndef __TELEMETRY
#define __TELEMETRY

#include <stdint.h>
#include <stdlib.h>



//маркер начала сообщения
#define TELEMETRY_START    0x7E
//заменитель зарезервированных значений
#define TELEMETRY_STUFFING 0x7D




//кодировщик телеметрических сообщений
class TelemetryCoder
{
    public:
    //буфер сообщения
    uint8_t* p_data;
    //указатель на текущую позицию в буфере
    uint8_t* p_current;    
    //контрольная сумма
    uint16_t crc;
    
    
    TelemetryCoder(size_t buffer_size)
    {
        p_data = (uint8_t*)malloc(buffer_size);
        reset();
    }
    
    ~TelemetryCoder()
    {
        free(p_data);
    }
    
    
    //сброс
    void reset()
    {
        p_current = p_data;       
        crc = 0;
    }
    
    //длина пакета
    uint16_t getLength()
    {
        return p_current - p_data;
    }
    
    
    //добавить байт данных
    void addDataByte(uint8_t dataByte)
	{  
        //если данные совпадают с зарезервированным стартом, маскируем заменителем
        if(dataByte == TELEMETRY_START)
        {
            *p_current++ = TELEMETRY_STUFFING;
            *p_current++ = 0x5E; // 0x7E xor 0x20
        }
        //если данные совпадают с зарезервированным заменителем, маскируем заменителем
        else if(dataByte == TELEMETRY_STUFFING)
        {
            *p_current++ = TELEMETRY_STUFFING;
            *p_current++ = 0x5D; // 0x7D xor 0x20
        }
        //иначе просто добавляем
        else
        {
            *p_current++ = dataByte;
        }
              
        //досчитываем контрольную сумму
        crc += dataByte; 
        crc += crc >> 8; 
        crc &= 0x00FF;  
	}
    
    
    
    
    void addMessage(void* msgPtr, uint16_t length, uint16_t msgID)
    {   
        //сброс контрольной суммы
        crc = 0;
        //пишем маркер старта сообщения
        *p_current++ = TELEMETRY_START;
        
        //пишем длину сообщения
        uint8_t* ptr = (uint8_t*)&length;
        addDataByte(ptr[0]);
        addDataByte(ptr[1]);
        
        //пишем ID сообщения
        ptr = (uint8_t*)&msgID;
        addDataByte(ptr[0]);
        addDataByte(ptr[1]);
        
        
        //пишем сообщение
        ptr = (uint8_t*)msgPtr;
        for(uint16_t i = 0; i < length; i++)
        {
           addDataByte(ptr[i]);
        }
        
        //пишем контрольную сумму
        addDataByte((uint8_t)(0xFF - crc));    
    }
    
        
};



#endif /*__TELEMETRY*/


