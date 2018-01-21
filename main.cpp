/*
TODO:
01. Напряжение
    - мерить опору

02. Барометр
    - драйвер
    - DMA
    
03. Температура
    - с барометра
    - с контроллера
    
04. SBUS
    - декодировка с помощью *uint32_t
    - режим синхронизации и приема кадра
    
05. SPORT
    - передавать RPY
    
*/

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"


#include "frsky_sport.cpp"
#include "frsky_sbus.cpp"

#include "l3g4200d.cpp"
#include "lis3mdl.cpp"
#include "lis331dlh.cpp"

#include "utils.cpp"
#include "ahrs_Mahony.cpp"
#include "telemetry.cpp"
#include "gps.cpp"
#include "pid.cpp"

#include <math.h>





//переферия
GPIO_InitTypeDef  	hgpio;
TIM_HandleTypeDef 	htim_timer;
TIM_HandleTypeDef 	htim_motor;
TIM_OC_InitTypeDef  htim_oc;
UART_HandleTypeDef 	sport_huart;
DMA_HandleTypeDef 	sport_hdma_tx;
UART_HandleTypeDef 	sbus_huart;
UART_HandleTypeDef 	modem_huart;
DMA_HandleTypeDef 	modem_hdma_tx;
UART_HandleTypeDef 	pc_huart;
DMA_HandleTypeDef 	pc_hdma_tx;
UART_HandleTypeDef 	gps_huart;
DMA_HandleTypeDef 	gps_hdma_tx;
I2C_HandleTypeDef 	hi2c;
I2C_HandleTypeDef 	hi2c_2;
DMA_HandleTypeDef 	i2c_hdma_rx;
DMA_HandleTypeDef 	i2c_hdma_rx_2;
ADC_HandleTypeDef   hadc;
ADC_ChannelConfTypeDef hadc_channel;




struct Timing
{
    public:    
    uint32_t prev_t;
    uint32_t max_t;
    float avg_period;
    float avg_rithm;
    
    void start(uint32_t time)
    {
        if(prev_t != 0)
        {
            uint32_t period = time - prev_t;
            avg_rithm = avg_rithm * 0.99f + period * 0.01f;
        }
        
        prev_t = time;
    }
    
    void stop(uint32_t time)
    {
        uint32_t period = time - prev_t;
        
        if(period > max_t) max_t = period;        
        avg_period = avg_period * 0.99f + period * 0.01f;
    }
};
Timing timings[10];


struct I2CStat
{
    uint32_t masterHangs;
    uint32_t slaveHangs;
    uint32_t error;
    uint32_t busy;
    uint32_t timeout;    
};
I2CStat i2c_stats[2];



//forward declaration
uint8_t SetSysClock_PLL_HSI(void);
uint32_t get_time();
void delay_us(uint32_t us);
void init(void);
void init_sensors(void);
void reinit_I2C(I2C_HandleTypeDef* h);
void ensure_I2C(I2C_HandleTypeDef* h, GPIO_TypeDef* p_port, uint16_t scl_pin, uint16_t sda_pin, I2CStat* p_stat);
uint32_t getThrottlePulseWidth(float throttle);

//DEBUG:forward declaration
void sport_t(bool v);
void taskAHRS();
void taskControl();
void taskTelemetry();
void taskStabilization(float deltat);
void taskReport();
void taskInitAHRS();




//приемник
FrskySport frsky_sport;
FrskySbus  frsky_sbus;
bool       new_rc_command = false;
float      rc_axis_x;
float      rc_axis_y;
float      rc_axis_z;
Vector3D   rc_axis_pitch_roll;
float      rc_angle;
Quaternion rc_target_orientation;

//ориентация
Quaternion targetQuaternion;
Vector3D targetAxis;
float targetAngle;
Vector3D targetErrors;
Vector3D targetPIDs;

//ось Z
Vector3D zAxis(0, 0, 1.0f);

//PID коэффициенты
 float
        maxValue = 0.30f,
		kP = 0.30f, maxP = 0.30f,
		kI = 1.00f, maxI = 0.04f,
		kD = 0.05f, maxD = 0.10f;

//PIDы по осям
PID pidX(maxValue, kP, maxP, kI, maxI, kD, maxD);
PID pidY(maxValue, kP, maxP, kI, maxI, kD, maxD);
PID pidZ(maxValue * 0.3f, kP, maxP * 0.3f, kI, maxI, kD, maxD); 


//Мин и макс сигналы моторов
uint32_t minMotors = 900,
         maxMotors = 1700;

//таблица микширования моторов
float mixMotor[4][4] = 
{
	{-1,-1,-1, 1},
	{ 1,-1, 1, 1},
	{ 1, 1,-1, 1},
	{-1, 1, 1, 1}
};



//сигналы моторов
float motors[4] = {0,0,0,0};

//газ				 
float Throttle = 0.0f;



//сенсоры
L3G4200D        gyroscope       (&hi2c_2, 1.00f);
LIS331DLH       accelerometer   (&hi2c_2, 1.00f);
LIS3MDL         magnetometer    (&hi2c, 1.00f);
I2CSensor3Axis  barometer       (&hi2c_2, 0xB9, 0x27);

//фильтр ориентации
AHRSMahony ahrs_Mahony(1, 0);

//кодировщик телеметрии
TelemetryCoder telemetryCoder(1000);

//декодер GPS
GPSDecoder gpsDecoder(500);

//сообщение gps UBX_NAV_PVT
UBX_NAV_PVT ubx_nav_pvt;



//целевой курс
float targetCourse = 0.0f;
//целевая ориентация
Quaternion targetOrientation();


//статус инициализации фильтра ориентации
static uint8_t ahrs_state = 0;



         uint8_t  sport_byte    = 0;
volatile bool     sport_rx_cplt = false;
volatile bool     sport_tx_cplt = false;
volatile bool     sport_err     = false;
         
         uint8_t  sbus_byte    = 0;
volatile bool     sbus_rx_cplt = false;
volatile bool     sbus_tx_cplt = false;
volatile bool     sbus_err     = false;

         uint8_t  modem_byte    = 0;
volatile bool     modem_rx_cplt = false;
volatile bool     modem_tx_cplt = false;
volatile bool     modem_err     = false;

         uint8_t  gps_byte      = 0;
volatile bool     gps_rx_cplt   = false;
volatile bool     gps_tx_cplt   = false;
volatile bool     gps_err       = false;

         uint8_t  pc_byte      = 0;
volatile bool     pc_rx_cplt   = false;
volatile bool     pc_tx_cplt   = false;
volatile bool     pc_err       = false;

volatile bool i2c_cplt;
volatile bool i2c_error;

volatile bool i2c_cplt_2;
volatile bool i2c_error_2;



uint16_t telemetry_packet_length = 0;

uint32_t adc_value = 0;
float    vbat = 0;


HAL_StatusTypeDef hal_status;





char report_data[1000];

//DEBUG
int32_t presure;
float presure_mBar;



HAL_StatusTypeDef I2CStatStatus(HAL_StatusTypeDef status, I2CStat* p_stat)
{
    switch(status)
    {
        case HAL_ERROR:   p_stat->error++; break;
        case HAL_BUSY:    p_stat->busy++;  break;
        case HAL_TIMEOUT: p_stat->timeout++; break;
        default: break;
    }
    return status;
}





int main()
{   
    
    //настройка переферии
	init();
    
    //таймер моторов 
	HAL_TIM_PWM_Start(&htim_motor, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_motor, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim_motor, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim_motor, TIM_CHANNEL_4);
    
    
    
    
    //стоп моторы
    htim_motor.Instance->CCR1 = minMotors;
    htim_motor.Instance->CCR2 = minMotors;
    htim_motor.Instance->CCR3 = minMotors;
    htim_motor.Instance->CCR4 = minMotors;            
    
    
    
    //запуск uart'ов    
	sport_t(1);
    HAL_UART_Receive_IT(&sport_huart, &sport_byte, 1);
    HAL_UART_Receive_IT(&sbus_huart, &sbus_byte, 1);
    HAL_UART_Receive_IT(&modem_huart, &modem_byte, 1);
    HAL_UART_Receive_IT(&gps_huart, &gps_byte, 1);
    HAL_UART_Receive_IT(&pc_huart, &pc_byte, 1);
    
    //запуск АЦП
    HAL_ADC_Start(&hadc);
    
    
    //настройка сенсоров
    init_sensors();       
    
    
   
    //рабочий цикл
	while(1)
	{   
        taskInitAHRS();
        taskControl();
        taskAHRS();        
        taskTelemetry();
	}
	
	
}


void taskInitAHRS()
{
    static uint32_t time  = 0;
    
    switch(ahrs_state)
    {
        case 0:
        {
            time = get_time();
            
            ahrs_Mahony.twoKp = 20.0f;
            ahrs_Mahony.twoKi = 0.0f;
            
            ahrs_state = 1;
            break;
        }
        case 1:
        {
            if(get_time() - time >= 5000000)
            {
                ahrs_Mahony.twoKp = 0.6f;
                ahrs_Mahony.twoKi = 0.003f;
                ahrs_state = 2;
            }
            break;
        }
    }
}







void taskAHRS()
{    
	static uint32_t time   = 0;
    static uint32_t deltat = 0;
    
    static uint32_t timeB  = 0;
    
    
    static int state   = 0;
    
    static int step    = -1;       
    static int step1   = -1;
    
    
    
    //машина состояний опроса гироскопа и акселерометра по i2c1
    switch(step)
	{    
        //запрос гироскопа 
        case 0:
		{
            //маленькая задержка перед следующей транзакцией
            delay_us(3);
            //проверка i2c           
            ensure_I2C(&hi2c_2, GPIOB, GPIO_PIN_10, GPIO_PIN_3, &i2c_stats[1]);
            //запрос гироскопа по i2c2 
            if(I2CStatStatus(gyroscope.beginReadAxisDMA(), &i2c_stats[1]) == HAL_OK) 
            {
                step++;
            }
			
			break;
        }
		//ждем гироскоп; 
		case 1:
		{   
            if(hi2c_2.State == HAL_I2C_STATE_READY)
            {   
                //если ошибка, возвращаемся назад к запросу гироскопа
                if(hi2c_2.ErrorCode != HAL_I2C_ERROR_NONE)
                {   
                    step--;
                }
                else
                {                    
                    step++;
                }
            }           
            
			break;            
			
		}
        //запрос акселерометра; обработка гироскопа; 
        case 2:
        {            
            //маленькая задержка перед следующей транзакцией
            delay_us(3);
            //проверка i2c 
            ensure_I2C(&hi2c_2, GPIOB, GPIO_PIN_10, GPIO_PIN_3, &i2c_stats[1]);            
            //запрос акселерометра i2c2
            if(I2CStatStatus(accelerometer.beginReadAxisDMA(), &i2c_stats[1]) == HAL_OK)
            {
                //обработка гироскопа
                gyroscope.processResult();
                step++;
            }
        }
        //обработка акселерометра;
        case 3:
        {   
            if(hi2c_2.State == HAL_I2C_STATE_READY)
            {                
                //если ошибка, возвращаемся назад к запросу акселерометра
                if(hi2c_2.ErrorCode != HAL_I2C_ERROR_NONE)
                {   
                    step--;
                }
                else
                {     
                    
                    //обработка акселерометра
                    accelerometer.processResult();
                    //защелкиваем машину состояний
                    step = -1;
                }
            }              
            break;
        }
        
        default: break;
    }
    
    
    
    switch(step1)
    {
        case 0:
        {
            //маленькая задержка перед следующей транзакцией
            delay_us(3);
            //проверка i2c             
            ensure_I2C(&hi2c, GPIOB, GPIO_PIN_8, GPIO_PIN_9, &i2c_stats[0]);
            //запрос магнетометра i2c2
            if(I2CStatStatus(magnetometer.beginReadAxisDMA(), &i2c_stats[0]) == HAL_OK)
            {
                //обработка акселерометра
                accelerometer.processResult();
                step1++;
            }
            break;
        }
        case 1:
        {
            if(hi2c.State == HAL_I2C_STATE_READY)
            {                
                //если ошибка, возвращаемся назад к запросу магнетометра
                if(hi2c.ErrorCode != HAL_I2C_ERROR_NONE)
                {   
                    step1--;
                }
                else
                {     
                    //обработка магнетометра
                    magnetometer.processResult();
                    step1 = -1;
                }
            }              
            break;
        }
        
        default: break;
    }
    
    
    
    //основная машина состояний
    switch(state)
    {
        //пауза
        case 0:
        {
            if((deltat = (get_time() - time)) >= 2000)
			{                
				time = get_time();
                //ограничиваем сверху (TODO: надо?)
                if(deltat > 3000) deltat = 3000; 
                //запуск опроса датчиков
                step  = 0;
                step1 = 0;
                //на ожидание окончания опроса датчиков
                state++;
                
                timings[0].start(get_time());
            }
            break;
        }       
        //ожидание окончания опроса датчиков
        case 1:
        {
            if(step == -1 && step1 == -1)
            {   
                timings[0].stop(get_time());
                
                //расчет текущей ориентации
                ahrs_Mahony.Update
                (
                    gyroscope.result.X, gyroscope.result.Y, gyroscope.result.Z,
                    accelerometer.result.X, accelerometer.result.Y, accelerometer.result.Z,                    
                    magnetometer.result.X, magnetometer.result.Y, magnetometer.result.Z,
                    deltat * 0.000001f
                );
                
                
                //... и сразу стабилизация
                taskStabilization(deltat * 0.000001f);
                   
                
                //DEBUG Baro
                if(get_time() - timeB >= 40000)
                {                
                    timeB = get_time(); 
                    ensure_I2C(&hi2c_2, GPIOB, GPIO_PIN_10, GPIO_PIN_3, &i2c_stats[1]);
                    barometer.readAxis(10);
                    
                    presure = (int32_t)(int8_t)barometer.data[3] << 16 | (uint16_t)barometer.data[2] << 8 | barometer.data[1];
                    presure_mBar = presure_mBar * 0.97f + ((float)presure / 4096) * 0.03f;
                }
                
                
                                                
                state = 0;
            }
            break;
        }
        default: 
        {
            state = 0;
            break;
        }
    }
    
}








void taskStabilization(float deltat)
{       
    timings[1].start(get_time());

    if(frsky_sbus.is_data_done(get_time())) //DEBUG
    {
        //читаем приемник
        Throttle = frsky_sbus.get_channel_throttle();
        rc_axis_x = frsky_sbus.get_channel_axis_x();
        rc_axis_y = frsky_sbus.get_channel_axis_y();
        rc_axis_z = frsky_sbus.get_channel_axis_z();
    }
    else
    {
        rc_axis_x  = rc_axis_y = rc_axis_z = 0.0f; 
    }
    
    
    //интегрируем угол курса
    if(fabs(rc_axis_z) > 0.02f)
    {
        targetCourse += -rc_axis_z * 0.003f;
        if(targetCourse >  PI2) targetCourse -= PI2;
        if(targetCourse < -PI2) targetCourse += PI2;
    }
    
    
    //ось крена-тангажа
    rc_axis_pitch_roll.Set(rc_axis_y, rc_axis_x, 0.0); 
    rc_axis_pitch_roll.Normalize();
    //угол крена-тангажа
    rc_angle = 0.50f * (fabs(rc_axis_x) > fabs(rc_axis_y) ? fabs(rc_axis_x) : fabs(rc_axis_y));								
    //кватернион крена тангажа
    Quaternion qPR = Quaternion::FromAxisAngle(rc_axis_pitch_roll, rc_angle);
    
    //кватернион курса
    Quaternion qC = Quaternion::FromAxisAngle(zAxis, targetCourse);
    //целевой кватернион
    rc_target_orientation = qC * qPR;
    
    
    //кватернион поворота из текущей ориентации в целевую
    targetQuaternion = ahrs_Mahony.Q.Conjugated() * rc_target_orientation;
    
    //выбираем кратчайшее направление
    if(targetQuaternion.W < 0)
    {
        targetQuaternion = -targetQuaternion;
    }
    
    //переводим в ось и угол
    targetQuaternion.ToAxisAngle(targetAxis, targetAngle);
    
    //текущие ошибки по осям
    targetErrors.Set(
        targetAxis.X * targetAngle / PI,
        targetAxis.Y * targetAngle / PI,
        targetAxis.Z * targetAngle / PI
    );
    
    //значения регуляторов
    targetPIDs.Set(
        pidX.get(targetErrors.X, deltat),
        pidY.get(targetErrors.Y, deltat),
        pidZ.get(targetErrors.Z, deltat)
    );
    
    
    
    
    float min, max;
    for(int i = 0; i < 4; i++)
    {
        motors[i] = 
            //газ
            Throttle +
            //pidы по осям
            mixMotor[i][0] * targetPIDs.X +
            mixMotor[i][1] * targetPIDs.Y +
            mixMotor[i][2] * targetPIDs.Z;
        
        //определяем минимальный и максимальный сигнал
        if(i == 0)
        {
            max = min = motors[0];
        }
        else
        {
            if(motors[i] < min) min = motors[i];
            if(motors[i] > max) max = motors[i];
        }
    }
    
    //корректирум сигналы, чтобы значения не выходили за 100%
    float overdrive = max - 1.0f;
    if(overdrive > 0.0f)
    {
        for(int i = 0; i < 4; i++)
        {
            motors[i] -= overdrive;
        }
    }
    
    
    
    
    if(Throttle < 0.02f)
    {
        htim_motor.Instance->CCR1 = minMotors;
        htim_motor.Instance->CCR2 = minMotors;
        htim_motor.Instance->CCR3 = minMotors;
        htim_motor.Instance->CCR4 = minMotors;
    }
    else
    {
        htim_motor.Instance->CCR4 = getThrottlePulseWidth(motors[0]);
        htim_motor.Instance->CCR2 = getThrottlePulseWidth(motors[1]);
        htim_motor.Instance->CCR1 = getThrottlePulseWidth(motors[2]);
        htim_motor.Instance->CCR3 = getThrottlePulseWidth(motors[3]);
    }

    timings[1].stop(get_time());
}








//DEBUG
void taskControl()
{
    static uint32_t time = 0;
    
    if(get_time() - time >= 10000)
    {
        time = get_time();
        
        if(frsky_sbus.is_data_done(time))
        {
            if(ahrs_state == 2)
            {
           
                //ahrs_Mahony.twoKp = mapf((frsky_sbus.channels[12] > 0 ? frsky_sbus.channels[12] : FRSKY_MIN_CHANNEL_VALUE) * 1.0f, FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, 0.00000f, 2.000f); 
                //ahrs_Mahony.twoKi = mapf((frsky_sbus.channels[13] > 0 ? frsky_sbus.channels[13] : FRSKY_MIN_CHANNEL_VALUE) * 1.0f, FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, 0.00000f, 0.200f); 
               
            }
        
            //пиды
            float p     = mapf((frsky_sbus.channels[12] > 0 ? frsky_sbus.channels[12] : FRSKY_MIN_CHANNEL_VALUE) * 1.0f, FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, 0.00f, 0.50f); 
            float i     = mapf((frsky_sbus.channels[14] > 0 ? frsky_sbus.channels[14] : FRSKY_MIN_CHANNEL_VALUE) * 1.0f, FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, 0.00f, 3.00f); 
            float _maxI = mapf((frsky_sbus.channels[15] > 0 ? frsky_sbus.channels[15] : FRSKY_MIN_CHANNEL_VALUE) * 1.0f, FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, 0.00f, 0.07f); 
            float d     = mapf((frsky_sbus.channels[13] > 0 ? frsky_sbus.channels[13] : FRSKY_MIN_CHANNEL_VALUE) * 1.0f, FRSKY_MIN_CHANNEL_VALUE, FRSKY_MAX_CHANNEL_VALUE, 0.00f, 0.50f); 
            
            
            //pidZ.kP = (pidX.kP = pidY.kP = p) * 3.0f;
            
            pidX.kP = pidY.kP = pidZ.kP = p ;
            pidX.kI = pidY.kI = pidZ.kI = i;
            pidX.maxI = pidY.maxI = pidZ.maxI = _maxI;            
            pidX.kD = pidY.kD = pidZ.kD = d;
        }
    }
}



void taskTelemetry()
{    
    static uint32_t reportTime = 0;
    
    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
    {
        adc_value = HAL_ADC_GetValue(&hadc);
        vbat = vbat * 0.9999f  + (adc_value * 3.3f * 11.86f / 4095) * 0.0001f;
    }
    
    
    frsky_sport.telemetry_current = 23.4f + ((HAL_GetTick() % 3) * 0.3f);
    frsky_sport.telemetry_vbat = vbat;
    
    
    
    if((get_time() - reportTime >= 65000)  && (modem_huart.gState == HAL_UART_STATE_READY) && (__HAL_UART_GET_FLAG(&modem_huart, UART_FLAG_TC)))
	{        
        reportTime = get_time();
        
        telemetryCoder.reset();        
        //telemetryCoder.addMessage(&reportTime, sizeof(uint32_t), 0x0001);
        
        telemetryCoder.addMessage(&vbat, sizeof(float), 0x0005);
        telemetryCoder.addMessage(&rc_target_orientation, sizeof(Quaternion), 0x0006);
        telemetryCoder.addMessage(&targetQuaternion, sizeof(Quaternion), 0x0007);        
        telemetryCoder.addMessage(&pidX.kP, sizeof(float), 0x0008);
        telemetryCoder.addMessage(&pidX.kI, sizeof(float), 0x0009);
        telemetryCoder.addMessage(&pidX.kD, sizeof(float), 0x000A);
        telemetryCoder.addMessage(&pidX.maxI, sizeof(float), 0x000B);
        telemetryCoder.addMessage(&ubx_nav_pvt, sizeof(UBX_NAV_PVT), 0x0101);
        
        telemetryCoder.addMessage(&gyroscope.axis[0], sizeof(int16_t) * 3, 0x0200);        
        telemetryCoder.addMessage(&accelerometer.axis[0], sizeof(int16_t) * 3, 0x0300);        
        telemetryCoder.addMessage(&magnetometer.axis[0], sizeof(int16_t) * 3, 0x0400);
        telemetryCoder.addMessage(&presure_mBar, sizeof(float), 0x0500);
        
        //DEBUG
        telemetryCoder.addMessage(&ahrs_Mahony.Q, sizeof(Quaternion), 0x0602);        
        telemetryCoder.addMessage(&ahrs_Mahony.twoKp, sizeof(float), 0x0603);
        telemetryCoder.addMessage(&ahrs_Mahony.twoKi, sizeof(float), 0x0604);
        
        telemetry_packet_length = telemetryCoder.getLength();
        
        HAL_UART_Transmit_DMA(&modem_huart, telemetryCoder.p_data, telemetry_packet_length);
            
    }
}




void taskReport()
{
    static uint32_t reportTime = 0;
    static uint32_t reportPtr = 0;
    
	if(get_time() - reportTime >= 10000 && pc_huart.gState == HAL_UART_STATE_READY)
	{
		reportTime = get_time();
		reportPtr = 0;	
		
		
		//reportPtr += sprintf(&report_data[reportPtr], "%f;%f;%f;%f \n ", Throttle, accelerometer.result.X, accelerometer.result.Y, accelerometer.result.Z);
		
		/*
		reportPtr += sprintf(&report_data[reportPtr], "clr \n ");			
		reportPtr += sprintf(&report_data[reportPtr], "angle: %f \n ", targetAngle * RAD_TO_GRAD);
		*/
		
		
		reportPtr += sprintf(&report_data[reportPtr], "%d %d %d \n ", magnetometer.axis[0], magnetometer.axis[1], magnetometer.axis[2]);				
		/*
        reportPtr += sprintf(&report_data[reportPtr], "m result: %f %f %f \n ", magnetometer.result.X, magnetometer.result.Y, magnetometer.result.Z);
		reportPtr += sprintf(&report_data[reportPtr], "imu magnet: %f %f %f \n ", imu.vMagnet.X, imu.vMagnet.Y, imu.vMagnet.Z);
		*/
		
		
		HAL_UART_Transmit_DMA(&pc_huart, (uint8_t*)report_data, reportPtr);
	}
}
















/////////////////////////////////////////
///////////// AUX FUNCTIONS /////////////
/////////////////////////////////////////

//получить время в микросекундах
uint32_t get_time()
{
	return TIM5->CNT;
}

//задержка в микросекундах
void delay_us(uint32_t us)
{
    uint32_t t = get_time();
    while(get_time() - t < us);
}

//настройка сенсоров
void init_sensors()
{
    //задержка, чтобы успеть убрать руки
    delay_us(3000000);
    
    //проверка i2c 
    ensure_I2C(&hi2c_2, GPIOB, GPIO_PIN_10, GPIO_PIN_3, &i2c_stats[1]);
    
     //включение, скорость выходных данных
	gyroscope.writeRegister(L3G4200D_CTRL_REG1, 0xCF, 10);
	//диапазон
	gyroscope.setRange(L3G4200D_RANGE_500, 1, 10);
	//калибровка
	//gyroscope.calibrate(500, 1, 10);
    Vector3D v; 
    uint32_t times = 5000;
    for(int i = 0; i < times; i++)
    {
        ensure_I2C(&hi2c_2, GPIOB, GPIO_PIN_10, GPIO_PIN_3, &i2c_stats[1]);
        gyroscope.readAxis(1);
        
        v.X += gyroscope.axis[0] * 1.0f;
        v.Y += gyroscope.axis[1] * 1.0f;
        v.Z += gyroscope.axis[2] * 1.0f;
        
        delay_us(3);
    }
    gyroscope.offset.X = v.X / times;
    gyroscope.offset.Y = v.Y / times;
    gyroscope.offset.Z = v.Z / times;

    
	
	
	//акселерометр
	accelerometer.writeRegister(LIS331DLH_CTRL_REG_1, 0x3F, 10);
	accelerometer.writeRegister(LIS331DLH_CTRL_REG_4, 0x90, 10);
	accelerometer.offset.Set(56.89f, -477.30f, 497.13f);	
	accelerometer.scaleFactor[0].Set(0.000121f, 0.000003f, 0.000000f);
	accelerometer.scaleFactor[1].Set(0.000002f, 0.000121f, 0.000000f);
	accelerometer.scaleFactor[2].Set(-0.000000f, 0.000000f, 0.000121f);
	
		
	//магнетометр    
	magnetometer.writeRegister(LIS3MDL_CTRL_REG1, 0x22, 10);
    magnetometer.writeRegister(LIS3MDL_CTRL_REG3, 0x00, 10);
    magnetometer.writeRegister(LIS3MDL_CTRL_REG4, 0x04, 10);
    magnetometer.writeRegister(LIS3MDL_CTRL_REG5, 0x40, 10);
	magnetometer.offset.Set(-298.040395f, 576.143406f, -1795.183563);	
	magnetometer.scaleFactor[0].Set(0.000361f, -0.000011f, -0.000030f);
	magnetometer.scaleFactor[1].Set(-0.000011f, 0.000345f, -0.000012f);
	magnetometer.scaleFactor[2].Set(-0.000030f, -0.000012f, 0.000318f);
    
  
    
    //барометр
    barometer.writeRegister(0x10, 0x6A, 10);
    barometer.writeRegister(0x20, 0xF4, 10);
}


//разрешить передачу SPORT
void sport_t(bool v)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, v ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void I2C_manual_slave_clocking(GPIO_TypeDef* p_port, uint16_t scl_pin, uint16_t sda_pin)
{
    //настраиваем порт, как выход общего назначения с открытым коллетором
    hgpio.Pin = scl_pin | sda_pin;
	hgpio.Mode = GPIO_MODE_OUTPUT_OD;
	hgpio.Pull = GPIO_NOPULL;
	hgpio.Speed = GPIO_SPEED_FAST;	
	HAL_GPIO_Init(p_port, &hgpio);
    
    
    //отпускаем линии SCL и SDA
    HAL_GPIO_WritePin(p_port, scl_pin | sda_pin, GPIO_PIN_SET);
    
    //клокаем SCL 8 (байт) + 1 (NACK) раз
    for(int i = 0; i < 9; i++)
    {
        //притягиваем к земле
        HAL_GPIO_WritePin(p_port, scl_pin, GPIO_PIN_RESET);
        delay_us(3);
        //отпускаем
        HAL_GPIO_WritePin(p_port, scl_pin, GPIO_PIN_SET);
        delay_us(3);
    }
    
    
    //настраиваем порт, для работы с I2C
    hgpio.Pin = scl_pin | sda_pin;
	hgpio.Mode = GPIO_MODE_AF_OD;
	hgpio.Pull = GPIO_NOPULL;
	hgpio.Speed = GPIO_SPEED_FAST;	
    hgpio.Alternate = 4;
	HAL_GPIO_Init(p_port, &hgpio);
  
}

//провека зависания мастера I2C
//TODO: проверить, что отвис слэйв
void ensure_I2C(I2C_HandleTypeDef* h, GPIO_TypeDef* p_port, uint16_t scl_pin, uint16_t sda_pin, I2CStat* p_stat)
{    
    if(__HAL_I2C_GET_FLAG(h, I2C_FLAG_BUSY) == SET)
    {
        reinit_I2C(h);        
        
        if(__HAL_I2C_GET_FLAG(h, I2C_FLAG_BUSY) == SET)
        {        
            I2C_manual_slave_clocking(p_port, scl_pin, sda_pin);
            reinit_I2C(h);
            p_stat->slaveHangs++;
        }else
        {
            p_stat->masterHangs++;
        }    
    }
    
}

//сброс i2c
void reinit_I2C(I2C_HandleTypeDef* h)
{   
    h->Instance->CR1 |= I2C_CR1_SWRST;
    h->Instance->CR1 &= ~I2C_CR1_SWRST;
	HAL_I2C_Init(h);    
}



//DEBUG
uint32_t getThrottlePulseWidth(float throttle){
	if(throttle < 0.0f) return minMotors;
	if(throttle > 1.0f) return maxMotors;
	
	return static_cast<uint32_t>(minMotors + (maxMotors - minMotors) * throttle);
}







//initialize perpheria
void init()
{
	//set system clock
	SetSysClock_PLL_HSI();	
	//HAL init
	HAL_Init();
	
	
	
	
	//clock	
	//set timer's clock full speed prescaler bit 
	__HAL_RCC_TIMCLKPRESCALER(1);
	
	//GPIO	
	__HAL_RCC_GPIOA_CLK_ENABLE();	
    __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
    
    //DMA
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    
	//UART
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_UART5_CLK_ENABLE();   
    
    
    //I2C
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    
    
    //ADC
    __HAL_RCC_ADC1_CLK_ENABLE();
    
	
	//TIM
	__HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
	
	
	
	
	/**************************************/
	/*************** SPORT ****************/
	/**************************************/	
		
	/*************** SPORT::GPIO::Allow TX *****************/	
	hgpio.Pin = GPIO_PIN_8;
	hgpio.Mode = GPIO_MODE_OUTPUT_PP;
	hgpio.Pull = GPIO_NOPULL;
	hgpio.Speed = GPIO_SPEED_HIGH;
		
	HAL_GPIO_Init(GPIOA, &hgpio);
	
	/**************** SPORT::GPIO::TX ****************/
	hgpio.Pin = GPIO_PIN_12;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_PULLUP;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF8_UART5;
		
	HAL_GPIO_Init(GPIOC, &hgpio);
	
	/**************** SPORT::GPIO::RX ****************/
	hgpio.Pin = GPIO_PIN_2;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_PULLUP;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF8_UART5;
		
	HAL_GPIO_Init(GPIOD, &hgpio);
	
	
	/**************** SPORT::UART ****************/
	sport_huart.Instance = UART5;
	sport_huart.Init.BaudRate = 57600;
	sport_huart.Init.WordLength = UART_WORDLENGTH_8B;
	sport_huart.Init.Parity = UART_PARITY_NONE;
	sport_huart.Init.StopBits = UART_STOPBITS_1;
	sport_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    sport_huart.Init.OverSampling = UART_OVERSAMPLING_16;
	sport_huart.Init.Mode = UART_MODE_TX_RX;
	
	HAL_UART_Init(&sport_huart);
	
	HAL_NVIC_SetPriority(UART5_IRQn, 0, 7);
	HAL_NVIC_EnableIRQ(UART5_IRQn);
	
    
    
	/**************** SPORT::DMA::TX **************/
	sport_hdma_tx.Instance                 = DMA1_Stream7; 
    
	sport_hdma_tx.Init.Channel             = DMA_CHANNEL_4; 
	sport_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH; 
	sport_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE; 
	sport_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE; 
	sport_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; 
	sport_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE; 
	sport_hdma_tx.Init.Mode                = DMA_NORMAL; 
	sport_hdma_tx.Init.Priority            = DMA_PRIORITY_LOW; 
	sport_hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; 
	sport_hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; 
	sport_hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE; 
	sport_hdma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
	

	HAL_DMA_Init(&sport_hdma_tx);
	
	__HAL_LINKDMA(&sport_huart, hdmatx, sport_hdma_tx);
	
	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 6);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	
	
    
    
    
    /**************************************/
	/*************** SBUS *****************/
	/**************************************/
	
	/**************** SBUS::GPIO ****************/	
	hgpio.Pin = GPIO_PIN_1;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_PULLUP;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF8_UART4;
		
	HAL_GPIO_Init(GPIOA, &hgpio);
	
	
	/**************** SBUS::UART ****************/	
	sbus_huart.Instance =  UART4;
	sbus_huart.Init.BaudRate = 100000;
	sbus_huart.Init.WordLength = UART_WORDLENGTH_9B;
	sbus_huart.Init.Parity = UART_PARITY_EVEN;
	sbus_huart.Init.StopBits = UART_STOPBITS_2;
	sbus_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    sbus_huart.Init.OverSampling = UART_OVERSAMPLING_16;
	sbus_huart.Init.Mode = UART_MODE_RX;
	
	HAL_UART_Init(&sbus_huart);
	
	HAL_NVIC_SetPriority(UART4_IRQn, 0, 5);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	
	
    
    
    
    
    /**************************************/
	/*************** MODEM ****************/
	/**************************************/	
			
	/**************** MODEM::GPIO ****************/
	hgpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_PULLUP;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF7_USART3;
    
    HAL_GPIO_Init(GPIOC, &hgpio);
    
    
    /**************** MODEM::UART ****************/	
	modem_huart.Instance =  USART3;
	modem_huart.Init.BaudRate = 57600;
	modem_huart.Init.WordLength = UART_WORDLENGTH_8B;
	modem_huart.Init.Parity = UART_PARITY_NONE;
	modem_huart.Init.StopBits = UART_STOPBITS_1;
	modem_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    modem_huart.Init.OverSampling = UART_OVERSAMPLING_16;
	modem_huart.Init.Mode = UART_MODE_TX | UART_MODE_RX;
	
	HAL_UART_Init(&modem_huart);
	
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 8);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
    
    
    /**************** MODEM::DMA::TX **************/
	modem_hdma_tx.Instance                 = DMA1_Stream3; 
    
	modem_hdma_tx.Init.Channel             = DMA_CHANNEL_4; 
	modem_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH; 
	modem_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE; 
	modem_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE; 
	modem_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; 
	modem_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE; 
	modem_hdma_tx.Init.Mode                = DMA_NORMAL; 
	modem_hdma_tx.Init.Priority            = DMA_PRIORITY_LOW; 
	modem_hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; 
	modem_hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; 
	modem_hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE; 
	modem_hdma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
	

	HAL_DMA_Init(&modem_hdma_tx);
	
	__HAL_LINKDMA(&modem_huart, hdmatx, modem_hdma_tx);
	
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 9);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    
    
    
    
    /**************************************/
	/**************** PC ******************/
	/**************************************/
	
	/**************** PC::GPIO ****************/	
	hgpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_PULLUP;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF7_USART2;
		
	HAL_GPIO_Init(GPIOA, &hgpio);
	
	/**************** PC::UART ****************/	
	pc_huart.Instance =  USART2;
	pc_huart.Init.BaudRate = 230400;
	pc_huart.Init.WordLength = UART_WORDLENGTH_8B;
	pc_huart.Init.Parity = UART_PARITY_NONE;
	pc_huart.Init.StopBits = UART_STOPBITS_1;
	pc_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	pc_huart.Init.Mode = UART_MODE_TX_RX;
	
	HAL_UART_Init(&pc_huart);
	
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	
	/**************** PC::UART::DMA ****************/
	pc_hdma_tx.Instance                 = DMA1_Stream6; 
    
	pc_hdma_tx.Init.Channel             = DMA_CHANNEL_4; 
	pc_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH; 
	pc_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE; 
	pc_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE; 
	pc_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; 
	pc_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE; 
	pc_hdma_tx.Init.Mode                = DMA_NORMAL; 
	pc_hdma_tx.Init.Priority            = DMA_PRIORITY_HIGH; 
	pc_hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; 
	pc_hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; 
	pc_hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE; 
	pc_hdma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
	

	HAL_DMA_Init(&pc_hdma_tx);
	
	__HAL_LINKDMA(&pc_huart, hdmatx, pc_hdma_tx);
	
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 12);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    
    
    /**************************************/
	/*************** GPS ****************/
	/**************************************/	
			
	/**************** GPS::GPIO ****************/
	hgpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_PULLUP;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF7_USART1;
    
    HAL_GPIO_Init(GPIOA, &hgpio);
    
    
    
    /**************** GPS::UART ****************/	
	gps_huart.Instance = USART1;
	gps_huart.Init.BaudRate = 9600;
	gps_huart.Init.WordLength = UART_WORDLENGTH_8B;
	gps_huart.Init.Parity = UART_PARITY_NONE;
	gps_huart.Init.StopBits = UART_STOPBITS_1;
	gps_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    gps_huart.Init.OverSampling = UART_OVERSAMPLING_16;
	gps_huart.Init.Mode = UART_MODE_TX | UART_MODE_RX;
	
	HAL_UART_Init(&gps_huart);
	
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 10);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
    
    
    
    /**************** GPS::DMA::TX **************/
	gps_hdma_tx.Instance                 = DMA2_Stream7; 
    
	gps_hdma_tx.Init.Channel             = DMA_CHANNEL_4; 
	gps_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH; 
	gps_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE; 
	gps_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE; 
	gps_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; 
	gps_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE; 
	gps_hdma_tx.Init.Mode                = DMA_NORMAL; 
	gps_hdma_tx.Init.Priority            = DMA_PRIORITY_LOW; 
	gps_hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; 
	gps_hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; 
	gps_hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE; 
	gps_hdma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
	

	HAL_DMA_Init(&gps_hdma_tx);
	
	__HAL_LINKDMA(&gps_huart, hdmatx, gps_hdma_tx);
	
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 11);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    
    
    
    
    
    
    /**************************************/
	/**************** I2C *****************/
	/**************************************/
	
	/************** I2C::GPIO *************/
	hgpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	hgpio.Mode = GPIO_MODE_AF_OD;
	hgpio.Pull = GPIO_NOPULL;
	hgpio.Speed = GPIO_SPEED_FAST;
	hgpio.Alternate = GPIO_AF4_I2C1;
	
	HAL_GPIO_Init(GPIOB, &hgpio);
	
    
	/************** I2C::I2C *************/
	hi2c.Instance = I2C1;
	hi2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.ClockSpeed      = 400000;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
    hi2c.Init.OwnAddress1     = 0;
    hi2c.Init.OwnAddress2     = 0;    
	
    //программный сброс и инициализация
    reinit_I2C(&hi2c);
    //проверка i2c
    ensure_I2C(&hi2c, GPIOB, GPIO_PIN_8, GPIO_PIN_9, &i2c_stats[0]);
	
	
	/************** I2C::DMA *************/
	i2c_hdma_rx.Instance                 = DMA1_Stream0;
	i2c_hdma_rx.Init.Channel             = DMA_CHANNEL_1; 
	i2c_hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY; 
	i2c_hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE; 
	i2c_hdma_rx.Init.MemInc              = DMA_MINC_ENABLE; 
	i2c_hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; 
	i2c_hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE; 
	i2c_hdma_rx.Init.Mode                = DMA_NORMAL; 
	i2c_hdma_rx.Init.Priority            = DMA_PRIORITY_LOW; 
	i2c_hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; 
	i2c_hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; 
	i2c_hdma_rx.Init.MemBurst            = DMA_MBURST_SINGLE; 
	i2c_hdma_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 

	HAL_DMA_Init(&i2c_hdma_rx);
	
	__HAL_LINKDMA(&hi2c, hdmarx, i2c_hdma_rx);
	
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    
    
    
    
    
    
    
     /**************************************/
	/**************** I2C2 *****************/
	/**************************************/
	
	/************** I2C2::GPIO *************/
	hgpio.Pin = GPIO_PIN_10 | GPIO_PIN_3;
	hgpio.Mode = GPIO_MODE_AF_OD;
	hgpio.Pull = GPIO_NOPULL;
	hgpio.Speed = GPIO_SPEED_FAST;
	hgpio.Alternate = GPIO_AF4_I2C2;
	
	HAL_GPIO_Init(GPIOB, &hgpio);
    
   	
	/************** I2C2::I2C *************/
	hi2c_2.Instance = I2C2;
	hi2c_2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c_2.Init.ClockSpeed      = 400000;
    hi2c_2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c_2.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
    hi2c_2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c_2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
    hi2c_2.Init.OwnAddress1     = 0;
    hi2c_2.Init.OwnAddress2     = 0;
	
    
    //программный сброс и инициализация
    reinit_I2C(&hi2c_2);
    //проверка i2c
    ensure_I2C(&hi2c_2, GPIOB, GPIO_PIN_10, GPIO_PIN_3, &i2c_stats[1]);
    
    
	
	
	/************** I2C::DMA *************/
	i2c_hdma_rx_2.Instance                 = DMA1_Stream2;
	i2c_hdma_rx_2.Init.Channel             = DMA_CHANNEL_7; 
	i2c_hdma_rx_2.Init.Direction           = DMA_PERIPH_TO_MEMORY; 
	i2c_hdma_rx_2.Init.PeriphInc           = DMA_PINC_DISABLE; 
	i2c_hdma_rx_2.Init.MemInc              = DMA_MINC_ENABLE; 
	i2c_hdma_rx_2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; 
	i2c_hdma_rx_2.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE; 
	i2c_hdma_rx_2.Init.Mode                = DMA_NORMAL; 
	i2c_hdma_rx_2.Init.Priority            = DMA_PRIORITY_LOW; 
	i2c_hdma_rx_2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; 
	i2c_hdma_rx_2.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; 
	i2c_hdma_rx_2.Init.MemBurst            = DMA_MBURST_SINGLE; 
	i2c_hdma_rx_2.Init.PeriphBurst         = DMA_PBURST_SINGLE; 

	HAL_DMA_Init(&i2c_hdma_rx_2);
	
	__HAL_LINKDMA(&hi2c_2, hdmarx, i2c_hdma_rx_2);
	
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 3);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    
    
    
    
    
    
    
	
	/**************************************/
	/************** Motors ****************/
	/**************************************/

    /************** Motors::Timer *****************/
	htim_motor.Instance = TIM3;
	htim_motor.Init.Period = 4000; 
    htim_motor.Init.Prescaler = (uint32_t) ((SystemCoreClock / 1000000) - 1); 
    htim_motor.Init.ClockDivision = 0; 
    htim_motor.Init.CounterMode = TIM_COUNTERMODE_UP; 
	
	HAL_TIM_PWM_Init(&htim_motor);
	
	/*********** Motors::Timer Channels ************/
	htim_oc.OCMode       = TIM_OCMODE_PWM1;
	htim_oc.Pulse        = 0;
    htim_oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    htim_oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    htim_oc.OCFastMode   = TIM_OCFAST_DISABLE;
    htim_oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    htim_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	HAL_TIM_PWM_ConfigChannel(&htim_motor, &htim_oc, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim_motor, &htim_oc, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim_motor, &htim_oc, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim_motor, &htim_oc, TIM_CHANNEL_4);
	
	
	/**************** Motors::GPIO *****************/
	hgpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_NOPULL;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF2_TIM3;
		
	HAL_GPIO_Init(GPIOB, &hgpio);
	
	
	hgpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	hgpio.Mode = GPIO_MODE_AF_PP;
	hgpio.Pull = GPIO_NOPULL;
	hgpio.Speed = GPIO_SPEED_HIGH;
	hgpio.Alternate = GPIO_AF2_TIM3;
		
	HAL_GPIO_Init(GPIOC, &hgpio);
    
    
    
    
    /**************************************/
    /**************** ADC *****************/
    /**************************************/
    hgpio.Pin = GPIO_PIN_0;
	hgpio.Mode = GPIO_MODE_ANALOG;
	hgpio.Pull = GPIO_NOPULL;
    
    HAL_GPIO_Init(GPIOC, &hgpio);
    
    
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;   
    hadc.Init.Resolution = ADC_RESOLUTION_12B;    
    hadc.Init.ScanConvMode = DISABLE;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.NbrOfDiscConversion = 0;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion = 1;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = DISABLE;

    HAL_ADC_Init(&hadc);

    hadc_channel.Channel = ADC_CHANNEL_10;
    hadc_channel.Rank = 1;
    hadc_channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    hadc_channel.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc, &hadc_channel);
    
    
    
    
    /**************************************/
	/************** Timer *****************/
	/**************************************/	
	htim_timer.Instance = TIM5;
	htim_timer.Init.Period = 0xFFFFFFFF; 
    htim_timer.Init.Prescaler = (uint32_t) ((SystemCoreClock / 1000000) - 1); 
    htim_timer.Init.ClockDivision = 0; 
    htim_timer.Init.CounterMode = TIM_COUNTERMODE_UP; 
	HAL_TIM_Base_Init(&htim_timer);
	__HAL_TIM_ENABLE(&htim_timer);
	
	
}







/////////////////////////////////////////
////////////// HAL callbacks ////////////
/////////////////////////////////////////

//HAL callbacks::UART::Tx
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	//SPORT
	if(UartHandle->Instance == UART5)
	{
        sport_tx_cplt = true;
	}
    //SBUS
    else if(UartHandle->Instance == UART4)
    {
        sbus_tx_cplt = true;
    }
	//modem
	else if(UartHandle->Instance == USART3)
	{
        modem_tx_cplt = true;
	}
    //GPS
    else if(UartHandle->Instance == USART1)
	{
        gps_tx_cplt = true;
	}
    //PC
    else if(UartHandle->Instance == USART2)
	{
        pc_tx_cplt = true;
	}
}

//HAL callbacks::UART::Rx
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	//SPORT
	if(UartHandle->Instance == UART5)
	{
        sport_rx_cplt = true;     
	}
    //SBUS
    else if(UartHandle->Instance == UART4)
    {
        sbus_rx_cplt = true;
    }
	//modem
	else if(UartHandle->Instance == USART3)
	{
        modem_rx_cplt = true;
	}
    //GPS
	else if(UartHandle->Instance == USART1)
	{
        gps_rx_cplt = true;
	}
    //PC
	else if(UartHandle->Instance == USART2)
	{
        pc_rx_cplt = true;
	}
}
	
//HAL callbacks::UART::Error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	//SPORT
	if(UartHandle->Instance == UART5)
	{
        sport_err = true;
	}
    //SBUS
    else if(UartHandle->Instance == UART4)
    {
        sbus_err = true;
    }
	//modem
	else if(UartHandle->Instance == USART3)
	{
        modem_err = true;
	}
    //GPS
	else if(UartHandle->Instance == USART1)
	{
        gps_err = true;
	}
    //PC
	else if(UartHandle->Instance == USART2)
	{
        pc_err = true;
	}
}


//HAL callbacks::I2C::RX
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h)
{
    if(h == &hi2c)
    {
        i2c_cplt = true;
    }
    else if(h == &hi2c_2)
    {
        i2c_cplt_2 = true;
    }
}

//HAL callbacks::I2C::Error
void HAL_I2C_ErrorCallback (I2C_HandleTypeDef *h)
{
    if(h == &hi2c)
    {
        i2c_error = true;
    }
    else if(h == &hi2c_2)
    {
        i2c_error_2 = true;
    }
}






/////////////////////////////////////////
/////////// INTERRUPT HANDLERS //////////
/////////////////////////////////////////
extern "C" 
{
	//HAL tick
	//TODO: create TIM5 as system timer
	void SysTick_Handler(void)
    {
       HAL_IncTick();
    }
        
	
    
    
	//INTERRUPT HANDLERS::SPORT UART
	void UART5_IRQHandler(void)	
	{
		sport_rx_cplt = false;
		sport_tx_cplt = false;
		sport_err     = false;
		
		HAL_UART_IRQHandler(&sport_huart);
		
		
		if(sport_rx_cplt)			
		{
			if(frsky_sport.process(sport_byte))
			{
                sport_t(0);
				__HAL_UART_DISABLE_IT(&sport_huart, UART_IT_RXNE);
				HAL_UART_Transmit_DMA(&sport_huart, frsky_sport.messageBuffer, frsky_sport.messageBufferEnd);
			}else
			{
                sport_t(1);
				HAL_UART_Receive_IT(&sport_huart, &sport_byte, 1);
			}
		}
		
		if(sport_err)
		{
			uint16_t dummy = sport_huart.Instance->DR;
            sport_t(1);
			HAL_UART_Receive_IT(&sport_huart, &sport_byte, 1);
		}
		
		if(sport_tx_cplt)
		{
			uint16_t dummy = sport_huart.Instance->DR;
            sport_t(1);
			HAL_UART_Receive_IT(&sport_huart, &sport_byte, 1);
		}		
	}
    
    
    //INTERRUPT HANDLERS::SPORT UART::DMA
    void DMA1_Stream7_IRQHandler(void) 
	{ 
		HAL_DMA_IRQHandler(sport_huart.hdmatx);		
	} 
    
    
    
    //INTERRUPT HANDLERS::SBUS UART
	void UART4_IRQHandler(void)	
	{
        sbus_rx_cplt = false;
		sbus_tx_cplt = false;
		sbus_err     = false;
        
        HAL_UART_IRQHandler(&sbus_huart);
        
        if(frsky_sbus.process(sbus_byte, get_time()))
        {
            timings[2].start(get_time());
            new_rc_command = true;
        }
        
        HAL_UART_Receive_IT(&sbus_huart, &sbus_byte, 1);
    }
    
    
    
    
	//INTERRUPT HANDLERS::I2C::DMA
	void DMA1_Stream0_IRQHandler(void) 
	{ 
		HAL_DMA_IRQHandler(hi2c.hdmarx);
	} 
    
    //INTERRUPT HANDLERS::I2C2::DMA
	void DMA1_Stream2_IRQHandler(void) 
	{ 
		HAL_DMA_IRQHandler(hi2c_2.hdmarx);
	} 
	
	
    
    
	
	//INTERRUPT HANDLERS::MODEM UART
	void USART3_IRQHandler(void)	
	{
        modem_rx_cplt = false;
		modem_tx_cplt = false;
		modem_err     = false;
        
		HAL_UART_IRQHandler(&modem_huart);
        
        HAL_UART_Receive_IT(&modem_huart, &modem_byte, 1);
        
        
        if(modem_rx_cplt)
        {
            //DEBUG
            //HAL_UART_Transmit_IT(&pc_huart, &modem_byte, 1);
            
            HAL_UART_Receive_IT(&modem_huart, &modem_byte, 1);
        }
        if(modem_err)
        {
            HAL_UART_Receive_IT(&modem_huart, &modem_byte, 1);            
        }
        
        
	}
	
    //INTERRUPT HANDLERS::MODEM UART::DMA
	void DMA1_Stream3_IRQHandler()
    {
        HAL_DMA_IRQHandler(modem_huart.hdmatx);		
    }
    
    
    
    
	//INTERRUPT HANDLERS::PC UART	
	void USART2_IRQHandler(void)	
	{	
        pc_rx_cplt = false;
		pc_tx_cplt = false;
		pc_err     = false;
        
		HAL_UART_IRQHandler(&pc_huart);
        
        HAL_UART_Receive_IT(&pc_huart, &pc_byte, 1);
        
        
        if(pc_rx_cplt)
        {
            
            //DEBUG
            //HAL_UART_Transmit_IT(&modem_huart, &pc_byte, 1);
            
            HAL_UART_Receive_IT(&pc_huart, &pc_byte, 1);
        }
        if(pc_err)
        {
            HAL_UART_Receive_IT(&pc_huart, &pc_byte, 1);            
        }
        
        
	}
	
    //INTERRUPT HANDLERS::PC UART DMA
	void DMA1_Stream6_IRQHandler(void) 
	{ 
		HAL_DMA_IRQHandler(pc_huart.hdmatx);
	} 
    
    
    
    
    //INTERRUPT HANDLERS::GPS UART
	void USART1_IRQHandler(void)	
	{
        gps_rx_cplt = false;
		gps_tx_cplt = false;
		gps_err     = false;
        
		HAL_UART_IRQHandler(&gps_huart);
        
        //HAL_UART_Receive_IT(&gps_huart, &gps_byte, 1);
        
        
        if(gps_rx_cplt)
        {
            //DEBUG           
            //HAL_UART_Transmit_IT(&pc_huart, &gps_byte, 1);
            
            switch(gpsDecoder.process(gps_byte))
            {
                case MSG_ID_UBX_NAV_PVT:
                {
                    ubx_nav_pvt = *((UBX_NAV_PVT*)gpsDecoder.data);
                }
                
            }
            
            HAL_UART_Receive_IT(&gps_huart, &gps_byte, 1);
        }
        if(gps_err)
        {
            HAL_UART_Receive_IT(&gps_huart, &gps_byte, 1);            
        }
        
        
        
	}
	
    //INTERRUPT HANDLERS::GPS UART::DMA
	void DMA2_Stream7_IRQHandler()
    {
        HAL_DMA_IRQHandler(gps_huart.hdmatx);		
    }
}








/******************************************************************************/
/*            PLL (clocked by HSI) used as System clock source                */
/******************************************************************************/
uint8_t SetSysClock_PLL_HSI(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet. */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
 
  // Enable HSI oscillator and activate PLL with HSI as source
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;   
  RCC_OscInitStruct.PLL.PLLM            = 16;            // VCO input clock = 1 MHz (16 MHz / 16)
  RCC_OscInitStruct.PLL.PLLN            = 360;           // VCO output clock = 360 MHz (1 MHz * 360)
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2; // PLLCLK = 180 MHz (360 MHz / 2)
  RCC_OscInitStruct.PLL.PLLQ            = 7;             //
  RCC_OscInitStruct.PLL.PLLQ            = 6;             //
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return 0; // FAIL
  }
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK; // 180 MHz
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;         // 180 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;           //  45 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;           //  90 MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    return 0; // FAIL
  }

  // Output clock on MCO1 pin(PA8) for debugging purpose
#if DEBUG_MCO == 1
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // 16 MHz
#endif

  return 1; // OK
}
