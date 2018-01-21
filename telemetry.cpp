#ifndef __TELEMETRY
#define __TELEMETRY

#include <stdint.h>
#include <stdlib.h>



//������ ������ ���������
#define TELEMETRY_START    0x7E
//���������� ����������������� ��������
#define TELEMETRY_STUFFING 0x7D




//���������� ��������������� ���������
class TelemetryCoder
{
    public:
    //����� ���������
    uint8_t* p_data;
    //��������� �� ������� ������� � ������
    uint8_t* p_current;    
    //����������� �����
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
    
    
    //�����
    void reset()
    {
        p_current = p_data;       
        crc = 0;
    }
    
    //����� ������
    uint16_t getLength()
    {
        return p_current - p_data;
    }
    
    
    //�������� ���� ������
    void addDataByte(uint8_t dataByte)
	{  
        //���� ������ ��������� � ����������������� �������, ��������� �����������
        if(dataByte == TELEMETRY_START)
        {
            *p_current++ = TELEMETRY_STUFFING;
            *p_current++ = 0x5E; // 0x7E xor 0x20
        }
        //���� ������ ��������� � ����������������� �����������, ��������� �����������
        else if(dataByte == TELEMETRY_STUFFING)
        {
            *p_current++ = TELEMETRY_STUFFING;
            *p_current++ = 0x5D; // 0x7D xor 0x20
        }
        //����� ������ ���������
        else
        {
            *p_current++ = dataByte;
        }
              
        //����������� ����������� �����
        crc += dataByte; 
        crc += crc >> 8; 
        crc &= 0x00FF;  
	}
    
    
    
    
    void addMessage(void* msgPtr, uint16_t length, uint16_t msgID)
    {   
        //����� ����������� �����
        crc = 0;
        //����� ������ ������ ���������
        *p_current++ = TELEMETRY_START;
        
        //����� ����� ���������
        uint8_t* ptr = (uint8_t*)&length;
        addDataByte(ptr[0]);
        addDataByte(ptr[1]);
        
        //����� ID ���������
        ptr = (uint8_t*)&msgID;
        addDataByte(ptr[0]);
        addDataByte(ptr[1]);
        
        
        //����� ���������
        ptr = (uint8_t*)msgPtr;
        for(uint16_t i = 0; i < length; i++)
        {
           addDataByte(ptr[i]);
        }
        
        //����� ����������� �����
        addDataByte((uint8_t)(0xFF - crc));    
    }
    
        
};



#endif /*__TELEMETRY*/


