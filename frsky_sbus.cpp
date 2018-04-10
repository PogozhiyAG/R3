#ifndef __FRSKY_SBUS
#define __FRSKY_SBUS

#include "stdint.h"
#include "utils.cpp"

#include <math.h>


#define FRSKY_SBUS_FRAME_LENGTH     25
#define FRSKY_BUFFER_LENGTH         25

#define FRSKY_MIN_CHANNEL_VALUE    172
#define FRSKY_MAX_CHANNEL_VALUE   1811

#define FRSKY_CHANNEL_THROTTLE       0
#define FRSKY_CHANNEL_AXIS_X         2
#define FRSKY_CHANNEL_AXIS_Y         1
#define FRSKY_CHANNEL_AXIS_Z         3

#define FRSKY_SBUS_DATA_TIMOUT_US 200000 

class FrskySbus
{
    public:			
    uint8_t data[FRSKY_BUFFER_LENGTH];
    int16_t channels[16];
    bool    is_frame_lost;
    bool    is_failsafe_activated;
    
    int32_t ptr;	
    int32_t start;
    int32_t fill;
    //время последнего кадра
    uint32_t last_frame_time;
    uint32_t volatile frames_count;
    
    uint32_t failsafe_count;
    uint32_t frame_lost_count;
	
	FrskySbus()
	{		
		ptr = 0;
		start = 0;
		fill = 0;
        last_frame_time = 0;
        frames_count = 0;
        failsafe_count = 0;
        frame_lost_count = 0;
	}
	
	
	bool process(uint8_t byte, uint32_t time)
	{	
        bool result = false;
        data[ptr] = byte;
        if (fill < FRSKY_SBUS_FRAME_LENGTH) fill++;     

        if (fill >= FRSKY_SBUS_FRAME_LENGTH)
        {
                if (data[ptr] == 0x00 && data[start = complt(ptr - (FRSKY_SBUS_FRAME_LENGTH - 1))] == 0x0f)
                {
                    last_frame_time = time;
                    frames_count++;
                    fill = 0;
                    
                    
                    channels[0]  = ((data[complt(start + 1)]      | data[complt(start + 2)] << 8)                                 & 0x07FF); 
                    channels[1]  = ((data[complt(start + 2)] >> 3 | data[complt(start + 3)] << 5)                                 & 0x07FF); 
                    channels[2]  = ((data[complt(start + 3)] >> 6 | data[complt(start + 4)] << 2 | data[complt(start + 5)] << 10) & 0x07FF); 
                    channels[3]  = ((data[complt(start + 5)] >> 1 | data[complt(start + 6)] << 7)                                 & 0x07FF); 
                    channels[4]  = ((data[complt(start + 6)] >> 4 | data[complt(start + 7)] << 4)                                 & 0x07FF); 
                    channels[5]  = ((data[complt(start + 7)] >> 7 | data[complt(start + 8)] << 1 | data[complt(start + 9)] <<  9) & 0x07FF); 
                    channels[6]  = ((data[complt(start + 9)] >> 2 | data[complt(start + 10)]<< 6)                                 & 0x07FF); 
                    channels[7]  = ((data[complt(start + 10)]>> 5 | data[complt(start + 11)]<< 3) 								  & 0x07FF); 
                    channels[8]  = ((data[complt(start + 12)]     | data[complt(start + 13)]<< 8)                                 & 0x07FF); 
                    channels[9]  = ((data[complt(start + 13)]>> 3 | data[complt(start + 14)]<< 5)                                 & 0x07FF); 
                    channels[10] = ((data[complt(start + 14)]>> 6 | data[complt(start + 15)]<< 2 | data[complt(start + 16)]<< 10) & 0x07FF); 
                    channels[11] = ((data[complt(start + 16)]>> 1 | data[complt(start + 17)]<< 7)                                 & 0x07FF); 
                    channels[12] = ((data[complt(start + 17)]>> 4 | data[complt(start + 18)]<< 4)                                 & 0x07FF); 
                    channels[13] = ((data[complt(start + 18)]>> 7 | data[complt(start + 19)]<< 1 | data[complt(start + 20)]<<  9) & 0x07FF); 
                    channels[14] = ((data[complt(start + 20)]>> 2 | data[complt(start + 21)]<< 6)                                 & 0x07FF); 
                    channels[15] = ((data[complt(start + 21)]>> 5 | data[complt(start + 22)]<< 3)                                 & 0x07FF); 
                    
                    uint8_t flags = data[complt(start + 23)];                    
                    is_frame_lost         = flags & 0x04;
                    is_failsafe_activated = flags & 0x08;
                    
                    //statistics
                    if(is_frame_lost) 
                        frame_lost_count++;
                    if(is_failsafe_activated) 
                        failsafe_count++;
                    
                    result = true;
                }
        }
        
        ptr = complt(ptr + 1);
        return result;
	}
	
	bool is_data_done(uint32_t time)
    {
        return !is_failsafe_activated && (time - last_frame_time < FRSKY_SBUS_DATA_TIMOUT_US) && (frames_count > 0);
    }
	
	float get_channel_throttle()
	{
		return mapf(channels[FRSKY_CHANNEL_THROTTLE], FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, 0.0, 1.0);
	}
	
	float get_channel_axis_x(float dead_band = 0.0f)
	{
		return deadband(mapf(channels[FRSKY_CHANNEL_AXIS_X], FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, -1.0, 1.0), dead_band);
	}
	
	float get_channel_axis_y(float dead_band = 0.0f)
	{
		return deadband(mapf(channels[FRSKY_CHANNEL_AXIS_Y], FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, -1.0, 1.0), dead_band);
	}
	float get_channel_axis_z(float dead_band = 0.0f)
	{
		return deadband(mapf(channels[FRSKY_CHANNEL_AXIS_Z], FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, -1.0, 1.0), dead_band);
	}
	
	
	private:
	inline int complt(int value)
	{
			return (value < 0) ? FRSKY_BUFFER_LENGTH + (value % FRSKY_BUFFER_LENGTH) : (value % FRSKY_BUFFER_LENGTH);
	}
    
    inline float deadband(float value, float dead_band)
    {
        if(fabs(value) < fabs(dead_band))
            value = 0.0f;
        return value;
    }
};



#endif //__FRSKY_SBUS

