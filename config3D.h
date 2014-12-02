/*
project_Quad 3d Fixed Pitch   
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
https://www.facebook.com/tinnakonza
*/
//Parameter system Quadrotor
#define m_quad 1.1 //kg
#define L_quad 0.225 //m quad
//The type of multicopter   
#define Quad_X
//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
#define tarremote 0.062  //0.092 slow 0.12 0.02 0.08 remote 
//#define tar 0.011 //0.012 0.015
#define tar 0.01

#define MINTHROTTLE 1064 
#define MAXTHROTTLE 1860
#define MINCOMMAND 1460
#define MIN_FWD_THROTTLE 1514//+54
#define MIN_REV_THROTTLE 1406//-54
#define MAXCOMMAND 1860
#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900

//PID-------------Rate
float K_Rate = 0.35;//0.25
float K_Rate_yaw = 0.36;//0.45

float Kp_rateRoll = 1.42;//1.05 1.12
float Ki_rateRoll = 0.85;//0.45
float Kd_rateRoll = 0.026;//0.028 0.041 0.051

float Kp_ratePitch = 1.42;//1.12
float Ki_ratePitch = 0.85;//0.45
float Kd_ratePitch = 0.026;//0.041

float Kp_rateYaw = 2.06;//2.07 2.6
float Ki_rateYaw = 1.35;//0.45
float Kd_rateYaw = 0.038;//0.052 0.022

#define TASK_100HZ 2
#define TASK_50HZ 3 //4
#define TASK_20HZ 10
#define TASK_10HZ 20
#define TASK_5HZ 40
#define TASK_2HZ 100
#define TASK_1HZ 200
#define TASK_noHZ 220
#define RAD_TO_DEG 57.295779513082320876798154814105

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;
byte armed = 0;
float G_Dt = 0.01; 
long Dt_sensor = 1000;
long Dt_roop = 10000;
int Status_LED = LOW;
int ESC_calibra = 0;
