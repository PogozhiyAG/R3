#ifndef __PID
#define __PID

#include "stdint.h"
#include <math.h>

class PID
{
	public:
            
        float 
            maxValue,
            kP, maxP, 
            kI, maxI,
            kD, maxD;
        
        float P, I, D;
        float value;
		
        float prevError;
	
	
	PID
    (
         float _maxValue,
         float _kP, float _maxP, 
         float _kI, float _maxI, 
         float _kD, float _maxD
    ):
         maxValue(_maxValue),
         kP(_kP), maxP(_maxP), 
         kI(_kI), maxI(_maxI), 
         kD(_kD), maxD(_maxD)
	{		
		prevError = 0.0f;
        value = 0.0f;
	}
    
    
	
	float get(float error, float deltat)
	{
		P = bound(kP * error, maxP);		
        I = kI != 0.0f ? bound(I + kI * error * deltat, maxI) : 0.0f;				
		D = bound(kD * (error - prevError) / deltat, maxD);
		
		prevError = error;
		
		return value = bound(P + I + D, maxValue);
	}
    
    
    
    inline float bound(float v, float bound)
    {
        bound = abs(bound);
        if(v >  bound) return bound;
        if(v < -bound) return -bound;
        return v;
    }
};


#endif /*__PID*/
