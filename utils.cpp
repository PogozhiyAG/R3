#ifndef __UTILS
#define __UTILS

#include <stdint.h>

//отображение значения с одного диапазонв на другой
inline float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline uint32_t mapi(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return static_cast<uint32_t>((x - in_min) * (out_max - out_min) * 1.0f / (in_max - in_min) + out_min);
}

#endif /*__UTILS*/
