#ifndef __GEOMETRY
#define __GEOMETRY


#include <math.h>

#define EPSILON     1.401298E-45f
#define PI			3.141592654f
#define PI2			6.283185308f
#define PI05		1.570796327f
#define RAD_TO_GRAD 57.29577951f

struct Vector3D
{
  public:
    float X;
    float Y;
    float Z;
	
		Vector3D() : X(0), Y(0), Z(0){}
    Vector3D(float _x, float _y, float _z) : X(_x), Y(_y), Z(_z){}
	
			
		inline void	Set(const float _x, const float _y, const float _z)
		{
				X = _x;
				Y = _y;
				Z = _z;
		}
		
		inline void Set(const Vector3D& v)
		{
				X = v.X;
				Y = v.Y;
				Z = v.Z;
		}

    inline float GetLength()
    {
      return sqrtf(X * X + Y * Y + Z * Z);
    }
        
    inline void Normalize(){     
      float length = GetLength();
			if(length > EPSILON)
			{
				X = X / length;
				Y = Y / length;
				Z = Z / length; 
			}
    }
		
		//operators
		inline void operator +=(const Vector3D& v)
		{
				X += v.X;
				Y += v.Y;
				Z += v.Z;
		}
		
		inline void operator -=(const Vector3D& v)
		{
				X -= v.X;
				Y -= v.Y;
				Z -= v.Z;
		}
		
		inline void operator *=(float s)
		{
				X *= s;
				Y *= s;
				Z *= s;
		}
		
		inline friend Vector3D operator +(const Vector3D& v0, const Vector3D& v1)
		{
				return Vector3D(v0.X + v1.X, v0.Y + v1.Y, v0.Z + v1.Z);
		}
		
		inline friend Vector3D operator -(const Vector3D& v0, const Vector3D& v1)
		{
				return Vector3D(v0.X - v1.X, v0.Y - v1.Y, v0.Z - v1.Z);
		}
		
    inline friend Vector3D operator -(const Vector3D& v)
		{
				return Vector3D(-v.X, -v.Y, -v.Z);
		}		
		
		inline friend float operator %(const Vector3D& v0, const Vector3D& v1)
		{
				return v0.X * v1.X + v0.Y * v1.Y + v0.Z * v1.Z;
		}
		
		inline friend Vector3D operator *(const Vector3D& v0, const Vector3D& v1)
		{
				return Vector3D(v0.Y * v1.Z - v0.Z * v1.Y,
												v0.Z * v1.X - v0.X * v1.Z,
												v0.X * v1.Y - v0.Y * v1.X);
		}
		
		inline friend Vector3D operator *(const Vector3D& v, const float s)
		{
				return Vector3D(v.X * s, v.Y * s, v.Z * s);
		}
    
};





struct Quaternion
{
	public:
		float X;
        float Y;
        float Z;
		float W;
	
		Quaternion():X(0), Y(0), Z(0), W(1){};		
		Quaternion(float _x, float _y, float _z, float _w):X(_x), Y(_y), Z(_z), W(_w){};
		
			
			
		inline static Quaternion FromAxisAngle(const Vector3D& axis, float angle)
		{
			float half_angle = angle / 2;
			float sin_a = sin(half_angle);			
			float cos_a = cos(half_angle);
			
			Quaternion result = Quaternion(
			  axis.X * sin_a,
			  axis.Y * sin_a,
			  axis.Z * sin_a,
			  cos_a
			);
			
			result.Normalize();
			return result;
		}
		
		
			
		inline static Quaternion FromAxisSmallAngle(const Vector3D& axis, float angle)
		{
			float half_angle = angle / 2;
			float sin_a = half_angle;			
			float cos_a = 1 - half_angle * half_angle / 2;
			
			Quaternion result = Quaternion(
				axis.X * sin_a,
			    axis.Y * sin_a,
			    axis.Z * sin_a,
				cos_a
			);
			
			result.Normalize();
			return result;
		}
		
		
		inline static Quaternion ShortestArc(const Vector3D& from, const Vector3D& to )
        {
            Vector3D c =  from * to;
            Quaternion result(c.X, c.Y, c.Z, from % to);
            result.Normalize();
            result.W += 1.0f;
            if(result.W <= EPSILON)
            {
                if((from.Z * from.Z) > (from.X * from.X))
                    result.Set(0, from.Z, - from.Y, result.W);
                else 
                    result.Set(from.Y, - from.X, 0, result.W);
            }
            result.Normalize(); 
            return result;
        }
		
				
		inline void Set(float x_, float y_, float z_, float w_) 
		{
            X = x_; Y = y_; Z = z_; W = w_;
        }
		
        inline void Set(const Quaternion& q) 
		{
            X = q.X; Y = q.Y; Z = q.Z; W = q.W;
        }
		
		inline void Ident(void) 
		{
            X = 0.0f; Y = 0.0f; Z = 0.0f; W = 1.0f;
        };
		
		inline void Conjugate(void) 
		{
            X = -X; Y = -Y; Z = -Z;
        };

        inline Quaternion Conjugated(void) const
		{
            return Quaternion( -X, -Y, -Z, W );
        }
		
		inline void Scale(float s) 
		{
            X *= s; Y *= s; Z *= s; W *= s;
        };

        inline float Norm() const 
		{
            return X*X + Y*Y + Z*Z + W*W;
        };

        inline float Magnitude(void) const 
		{
            return sqrtf(Norm());
        }
		
		inline void StabilizeLength()
        {
           float cs = (float)(fabs(X) + fabs(Y) + fabs(Z) + fabs(W));
           if( cs > 0.0f )
               Set( X/=cs,  Y/=cs,  Z/=cs,  W/=cs ); 
           else
               Ident();
        }
		
		inline void Normalize(void) 
		{
            float m = Magnitude();
            if( m < EPSILON )
            {
                StabilizeLength();
                m = Magnitude();
            }
            Scale( 1.0f/m );
        }
		
		
		inline Quaternion operator-()const 
		{
            return Quaternion(-X, -Y, -Z, -W);
        }

        inline Quaternion operator+(const Quaternion& q)const 
		{
            return Quaternion(X + q.X, Y + q.Y, Z + q.Z, W + q.W);
        }

        inline Quaternion operator-(const Quaternion& q)const 
		{
            return Quaternion(X - q.X, Y - q.Y, Z - q.Z, W - q.W);
        }

        inline Quaternion operator*(const float& s)const 
		{
            return Quaternion(X*s, Y*s, Z*s, W*s);
        }
   
        inline Quaternion operator*(const Quaternion& q)const 
		{
            return Quaternion
            (
                W*q.X + X*q.W + Y*q.Z - Z*q.Y,
                W*q.Y + Y*q.W + Z*q.X - X*q.Z,
                W*q.Z + Z*q.W + X*q.Y - Y*q.X,
                W*q.W - X*q.X - Y*q.Y - Z*q.Z
            );       
        }
		
		
			
		inline Vector3D Rotate(const Vector3D& v)
		{
            Quaternion q
            (
              v.X * W + v.Z * Y - v.Y * Z,
              v.Y * W + v.X * Z - v.Z * X,
              v.Z * W + v.Y * X - v.X * Y,
              v.X * X + v.Y * Y + v.Z * Z
            );
				
            return Vector3D
            (
              W * q.X + X * q.W + Y * q.Z - Z * q.Y,
              W * q.Y + Y * q.W + Z * q.X - X * q.Z,
              W * q.Z + Z * q.W + X * q.Y - Y * q.X
            ) * (1.0f / Norm());
        }
		
		
		inline void ToAxisAngle(Vector3D& axis, float& angle)const 
		{
            float vl = sqrtf( X*X + Y*Y + Z*Z );
            if( vl > EPSILON )
            {
                float ivl = 1.0f/vl;
                axis.Set( X*ivl, Y*ivl, Z*ivl );
                if( W < 0 )
                    angle = 2.0f * atan2f(-vl, -W); //-PI,0 
                else
                    angle = 2.0f * atan2f( vl,  W); //0,PI 
            }else{
                axis = Vector3D(0, 0, 0);
                angle = 0;
            }
        };

};


#endif /*__GEOMETRY*/








