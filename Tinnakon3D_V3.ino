/*
project_Quad 3d Fixed Pitch   
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
https://www.facebook.com/tinnakonza

date: 27-11-2557(2014)  Tinnakon3D_V1
date: 29-11-2557(2014)  Tinnakon3D_V2
date: 30-11-2557(2014)  Tinnakon3D_V3

support:  Board MWC-S V1.5
• Atmega328p
• MPU6050C Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2

Quad=> X
           D3             D10
              \         / 
                \ --- /
                 |   |
                / --- \
              /         \ 
           D11            D9
---------motor---------
FontLeft  => D3
FontRight => D10
BackLeft  => D11
BackRight => D9          
----------rx-----------           
Throttle  => D2
Aileron   => D4
Elevator  => D5
Ruder     => D6
Aux       => D7 
*/
#include <Arduino.h>
#include <Wire.h>
#include "config3D.h"
#include "multi_328prx.h"
#include "mpu6050.h"
#include "Control_PIDA.h"
#include "motorX4.h"

void setup()
{
  Serial.begin(57600);//38400
  Serial.print("TK_Quad3D_Run_Roop_200Hz");Serial.println("\t");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  configureReceiver();
  motor_initialize();
  Wire.begin();
  delay(10);
  mpu6050_initialize();
  delay(10); 
  digitalWrite(13, HIGH);
  TWBR = ((F_CPU / 400000L) - 16) / 2;//change the I2C clock rate to 400kHz 
  delay(10);
     for(uint8_t i=0; i<50; i++) 
    {
     mpu6050_Gyro_Values();
     delay(20);
    }
    digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  RC_Calibrate();//"multi_rxPPM2560.h"
  Serial.print("TK_Quad3D_Run_Roop_200Hz");Serial.println("\t");
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop()
{
  Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
  if(Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
    if(Dt_sensor >= 1000 && gyroSamples < 4)////Collect 3 samples = 2760 us  && gyroSamples < 5  && gyroSamples < 5
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
    }
   Dt_roop = micros() - previousTime;// 200 Hz task loop (10 ms)  , 5000 = 0.02626 ms
   if(Dt_roop <= 0)
   {
    Dt_roop = 5001; 
   }   
    if (Dt_roop >= 5000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
      mpu6050_Get_gyro();
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      GyroZf = (GyroZ + GyroZ2)/2.0;
      GyroX2 = GyroX;GyroY2 = GyroY;GyroZ2 = GyroZ;//gyro Old1  
//PID modeControl///////////
     Control_PIDRate();
//////Out motor///////////
//armed = 1;
     motor_Mix();
/////////////////////////
     motor_command(); 
////////end Out motor//////
 if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
  computeRC();
  if(CH_THR > 1450 && CH_THR < 1550 && AUX_1 > 1900 && armed == 0){
     armed = 1;
     digitalWrite(13, HIGH);//B
  } 
  if(AUX_1 < 1900){
     armed = 0;
     digitalWrite(13, LOW);
      }
 //end  ARM and DISARM your helicopter 3D///////////////      
}//end roop 50 Hz 
         if (frameCounter % TASK_noHZ == 0)//roop print  ,TASK_5HZ  TASK_10HZ
        {
            //Serial.print(CH_THR);Serial.print("\t");
            //Serial.print(CH_AIL);Serial.print("\t");  
            //Serial.print(CH_ELE);Serial.print("\t");
            //Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            Serial.print(uAltitude);Serial.print("\t");
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 

            //Serial.print(GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(roll_D_rate);Serial.print("\t");
            //Serial.print(GyroY*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroYf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            //Serial.print(roll_I_rate);Serial.print("\t");
            
            Serial.print(motor_FrontL);Serial.print("\t");     
            Serial.print(motor_FrontLf);Serial.print("\t");
            Serial.print(motor_FrontRf);Serial.print("\t");
            Serial.print(motor_BackLf);Serial.print("\t");
            Serial.print(motor_BackRf);Serial.print("\t");
            //Serial.print(motor_Left);Serial.print("\t");
            //Serial.print(motor_Right);Serial.print("\t");
            Serial.print(gyroSamples2);Serial.print("\t");
            Serial.print(G_Dt*1000);Serial.print("\t");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
        }//end roop 5 Hz 
        if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 100 HZ 
}
