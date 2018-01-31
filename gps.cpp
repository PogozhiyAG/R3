#ifndef __UBLOX_GPS
#define __UBLOX_GPS

#include <stdint.h>
#include <stdlib.h>




//Заголовок сообщения UBX
#define UBX_MSG_HEADER_L 0xB5
#define UBX_MSG_HEADER_H 0x62


//Коды сообщений
//UBX-NAV-PVT Navigation Position Velocity Time Solution
#define MSG_ID_UBX_NAV_PVT 0x0701


//Сообщение протокола UBX UBX-NAV-PVT (0x01 0x07) 
struct UBX_NAV_PVT
{
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t reserved1;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;    
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t reserved2;
    uint32_t reserved3;
    int32_t headVeh;
    uint32_t reserved4;
};





namespace GPSDecoderState
{
    enum GPSDecoderState
    {
        Header = 0, 
        ClassID = 1, 
        Length = 2, 
        Payload = 3, 
        Checksum = 4   
    };
}





class GPSDecoder
{
    public:
    GPSDecoderState::GPSDecoderState state;
    uint8_t prev_byte;
    uint8_t* data;
    uint16_t buffer_size;    
    uint8_t cnt;
    
    uint16_t msg_class_id;
    uint16_t msg_length;
    uint16_t msg_checksum;
    
    
    GPSDecoder(uint16_t _buffer_size): state(GPSDecoderState::Header), prev_byte(0), buffer_size(_buffer_size), cnt(0)
    {
        data = (uint8_t*)malloc(buffer_size);
    }
    
    
    uint16_t process(uint8_t byte)
    {
        uint16_t result = 0;
            
        if(byte == UBX_MSG_HEADER_H && prev_byte == UBX_MSG_HEADER_L)
        {
            state = GPSDecoderState::ClassID;
            cnt = 0;
        }
        else
        {
            if (state != GPSDecoderState::Header)
            {
                switch(state)
                {                    
                    case GPSDecoderState::ClassID :
                    {   
                        data[cnt++] = byte;
                        if(cnt >= 2)
                        {
                            msg_class_id = *((uint16_t*)data);
                            state = GPSDecoderState::Length;
                            cnt = 0;
                        }
                        break;
                    }
                    case GPSDecoderState::Length :
                    {
                        data[cnt++] = byte;
                        if(cnt >= 2)
                        {
                            msg_length = *((uint16_t*)data);
                            state = GPSDecoderState::Payload;
                            cnt = 0;
                        }
                        break;
                    }
                    case GPSDecoderState::Payload :
                    {
                        //защита от переполнения буфера
                        if(cnt >= buffer_size)
                        {
                            state = GPSDecoderState::Header;
                            break;
                        }
                            
                        data[cnt++] = byte;
                        if(cnt >= msg_length)
                        {
                            state = GPSDecoderState::Checksum;
                            cnt = 0;
                        }
                        break;
                    }
                    case GPSDecoderState::Checksum :
                    {
                        data[cnt++] = byte;
                        if(cnt >= 2)
                        {
                            msg_checksum = *((uint16_t*)data);
                            state = GPSDecoderState::Header;  
                            result = msg_class_id;
                        }
                        break;
                    }
                    default:
                    {
                        state = GPSDecoderState::Header;
                        break;
                    }
                }
            }
        }
        
        
        prev_byte = byte;
        return result;
    }
};


#endif /*__UBLOX_GPS*/
