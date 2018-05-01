#ifndef __AHRS_MAHONY
#define __AHRS_MAHONY

#include <math.h>
#include "geometry.cpp"



class AHRSMahony
{
    public:
    float twoKp;											// 2 * proportional gain (Kp)
    float twoKi;											// 2 * integral gain (Ki)
    float q0, q1, q2, q3;					// quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy, integralFBz;	// integral error terms scaled by Ki
    Quaternion Q;
    
    AHRSMahony(float _twoKp, float _twoKi) : twoKp(_twoKp), twoKi(_twoKi)
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        ResetIntegral();
    }
    
    
    void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat, bool useAcc = true) {
        float recipNorm;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
        float hx, hy, bx, bz;
        float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            //MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(useAcc && !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
        {

            // Normalise accelerometer measurement
            recipNorm = sqrt(ax * ax + ay * ay + az * az);
            ax /= recipNorm;
            ay /= recipNorm;
            az /= recipNorm;     

            // Normalise magnetometer measurement
            recipNorm = sqrt(mx * mx + my * my + mz * mz);
            mx /= recipNorm;
            my /= recipNorm;
            mz /= recipNorm;   

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;   

            // Reference direction of Earth's magnetic field
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrt(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5f + q3q3;
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
        
            // Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // Compute and apply integral feedback if enabled
            if(twoKi > 0.0f) {
                integralFBx += twoKi * halfex * deltat;	// integral error scaled by Ki
                integralFBy += twoKi * halfey * deltat;
                integralFBz += twoKi * halfez * deltat;                
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
        
        // apply integral feedback
        gx += integralFBx;	
        gy += integralFBy;
        gz += integralFBz;
        
        
        // Integrate rate of change of quaternion
        gx *= (0.5f * deltat);		// pre-multiply common factors
        gy *= (0.5f * deltat);
        gz *= (0.5f * deltat);
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx); 
        
        // Normalise quaternion
        recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 /= recipNorm;
        q1 /= recipNorm;
        q2 /= recipNorm;
        q3 /= recipNorm;
        
        Q.Set(q1, q2, q3, q0); 
    }
    
    
    
    void ResetIntegral()
    {
        integralFBx = integralFBy = integralFBz = 0.0f;
    }
    
};







#endif //__AHRS_MAHONY
