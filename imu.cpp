#ifndef __IMU
#define __IMU

#include <math.h>
#include "geometry.cpp"

//оси
static const Vector3D xAxis(1,0,0);
static const Vector3D yAxis(0,1,0);
static const Vector3D zAxis(0,0,1);


namespace Orientation
{
	typedef enum
	{
		CourseGravity = 0,
		CourseGravityAcc = 1,
		GravityAcc = 2,
		Gravity = 3		
	} Mode;
};


class IMU
{	
	public:
		//коэффициент фильтра коррекции гравитации
		float lpfAccelerometer;
		//коэффициент фильтра коррекции магнитного поля
		float lpfMagnetometer;
		//пороговое значение разницы длин вектора гравитации и текущего показания акселерометра,
	  //выше которого не будет происходить коррекция вектора гравитации акселерометром
		float accLengthThreshold;
	
		//режим ориентации
		Orientation::Mode orientationMode;
	
		//магнитное поле
		Vector3D vMagnet;
		//магнитный север (вектор магнитного поля, перпендикулярный гравитации)
		Vector3D vNorth;
		//гравитация с отрицательным знаком (то, что показывает акселеромер в состоянии покоя)
		Vector3D vMinusG;
		//гравитация
		Vector3D vG;
		//собственное ускорение
		Vector3D vA;
		//направление суммы гравитации и собственного ускорения
		Vector3D vGA;
	
	
	
	IMU(float _lpfAccelerometer, float _lpfMagnetometer, float _accLengthThreshold, Orientation::Mode _orientationMode):lpfAccelerometer(_lpfAccelerometer),lpfMagnetometer(_lpfMagnetometer), accLengthThreshold(_accLengthThreshold), orientationMode(_orientationMode)
	{		
	}
	
		
  //получить текущую ориентацию	
	void computeAttitude(Vector3D& gyroscope, Vector3D& accelerometer, Vector3D& magnetometer, float period)
	{
		//угловые скорости в радианах в секунду
		Vector3D angularVelocity = gyroscope * -period;
	
		//кватернионы поворотов по трем осям с апроксимацией малых углов
		Quaternion qx = Quaternion::FromAxisSmallAngle(xAxis, angularVelocity.X);
		Quaternion qy = Quaternion::FromAxisSmallAngle(yAxis, angularVelocity.Y);
		Quaternion qz = Quaternion::FromAxisSmallAngle(zAxis, angularVelocity.Z);
		
	  //кватернион поворота по трем осям
		Quaternion qRotate = qx * qy * qz;
		
		//вращаем вектор гравитации с отрицательным знаком
		vMinusG = qRotate.Rotate(vMinusG);
		//вращаем векторр магнитного поля
		vMagnet = qRotate.Rotate(vMagnet);
			
		
		
		//TODO: Нормализовать аксель и маг
		
		//правим гравитацию акселерометром, если он по модулю не сильно отличаеися от G
		float accLength = accelerometer.GetLength();						
		if(fabs(accLength - 1.0f) < accLengthThreshold)
		{
			vMinusG.X = vMinusG.X * lpfAccelerometer + accelerometer.X * (1.0f - lpfAccelerometer);
			vMinusG.Y = vMinusG.Y * lpfAccelerometer + accelerometer.Y * (1.0f - lpfAccelerometer);
			vMinusG.Z = vMinusG.Z * lpfAccelerometer + accelerometer.Z * (1.0f - lpfAccelerometer);
		}
		vMinusG.Normalize();	
		
	  
		//правим вектор магнитного поля магнетометром
		vMagnet.X = vMagnet.X * lpfMagnetometer + magnetometer.X * (1.0f - lpfMagnetometer);
		vMagnet.Y = vMagnet.Y * lpfMagnetometer + magnetometer.Y * (1.0f - lpfMagnetometer);
		vMagnet.Z = vMagnet.Z * lpfMagnetometer + magnetometer.Z * (1.0f - lpfMagnetometer);			

		//восстанавливаем магнитный север (проецируем магнитное поле перпендикулярно гравитации)
		vNorth =  (vMinusG * vMagnet) * vMinusG;
		vNorth.Normalize();
	
		
		//вычисляем вспомогательные векторы
		//гравитация
		vG = -vMinusG;
		//собственное ускорение
		vA = accelerometer - vMinusG;
		//направление суммы гравитации и собственного ускорения
		vGA = vG + vA;
		vGA.Normalize();
		
	}
	
	
	//получить кватернион поворота из текущей в целевую ориентацию
	Quaternion computeTargetQuaternion()
	{
		Quaternion result;
		switch(orientationMode)
		{
			case Orientation::CourseGravity :
			{
				Quaternion q1 = Quaternion::ShortestArc(zAxis, vMinusG);
				Vector3D vy1 = q1.Rotate(yAxis);				
				Quaternion q2 = Quaternion::ShortestArc(vy1, vNorth);
				result = q2 * q1;
				break;
			}
			case Orientation::CourseGravityAcc :
			{					
				Quaternion q1 = Quaternion::ShortestArc(vG, vGA);
				Vector3D vNA = q1.Rotate(vNorth);
				q1 = Quaternion::ShortestArc(-zAxis, vGA);
				Vector3D vy1 = q1.Rotate(yAxis);
				Quaternion q2 = Quaternion::ShortestArc(vy1, vNA);
				result = q2 * q1;
				break;
			}
			case Orientation::GravityAcc :
			{					
				Quaternion q1 = Quaternion::ShortestArc(-zAxis, vGA);					
				result = q1;
				break;
			}
			case Orientation::Gravity:
			{					
				Quaternion q1 = Quaternion::ShortestArc(-zAxis, vG);					
				result = q1;
				break;
			}
		}
		return result;
	}
	
	
};







class MadgwickAHRS
{
	public:
		float beta;							// 2 * proportional gain (Kp)
		float q0, q1, q2, q3;	 // quaternion of sensor frame relative to auxiliary frame
	
	MadgwickAHRS(float _beta)
	{
		q0 = 1.0f;
		q1 = 0.0f; 
		q2 = 0.0f; 
		q3 = 0.0f;
		beta = _beta;
	}
	
	
	//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float samplePeriod) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, samplePeriod);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * ((-q1 * gx) - (q2 * gy) - (q3 * gz));
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
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
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * samplePeriod;
	q1 += qDot2 * samplePeriod;
	q2 += qDot3 * samplePeriod;
	q3 += qDot4 * samplePeriod;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float samplePeriod) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * ((-q1 * gx) - (q2 * gy) - (q3 * gz));
	qDot2 = 0.5f * ((q0 * gx) + (q2 * gz) - (q3 * gy));
	qDot3 = 0.5f * ((q0 * gy) - (q1 * gz) + (q3 * gx));
	qDot4 = 0.5f * ((q0 * gz) + (q1 * gy) - (q2 * gx));

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt((ax * ax) + (ay * ay) + (az * az));
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = (_4q0 * q2q2) + (_2q2 * ax) + (_4q0 * q1q1) - (_2q1 * ay);
		s1 = (_4q1 * q3q3) - (_2q3 * ax) + (4.0f * q0q0 * q1) - (_2q0 * ay) - _4q1 + (_8q1 * q1q1) + (_8q1 * q2q2) + (_4q1 * az);
		s2 = (4.0f * q0q0 * q2) + (_2q0 * ax) + (_4q2 * q3q3) - (_2q3 * ay) - _4q2 + (_8q2 * q1q1) + (_8q2 * q2q2) + (_4q2 * az);
		s3 = (4.0f * q1q1 * q3) - (_2q1 * ax) + (4.0f * q2q2 * q3) - (_2q2 * ay);
		recipNorm = invSqrt((s0 * s0) + (s1 * s1) + (s2 * s2) + (s3 * s3)); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= (beta * s0);
		qDot2 -= (beta * s1);
		qDot3 -= (beta * s2);
		qDot4 -= (beta * s3);
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * samplePeriod;
	q1 += qDot2 * samplePeriod;
	q2 += qDot3 * samplePeriod;
	q3 += qDot4 * samplePeriod;

	// Normalise quaternion
	recipNorm = invSqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

	
	
	
	static float invSqrt(float x) 
	{
		/*float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));*/

		/*
		unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
		float tmp = *(float*)&i;
		float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
		return y;
		*/
		
		return 1/sqrtf(x);
	}
	
};






#endif /*__IMU*/












