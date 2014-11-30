/*
project_Quad 3d Fixed Pitch   
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
https://www.facebook.com/tinnakonza
*/
//motor Arduino boards 328P works on pins 3, 10, 11, 9
uint8_t MOTOR_FrontL_PIN = 3;//PD3
uint8_t MOTOR_FrontR_PIN = 10;//PB2
uint8_t MOTOR_BackL_PIN = 11;//PB3
uint8_t MOTOR_BackR_PIN = 9;//PB1
    
#define PWM_FREQUENCY 400   //400 in Hz and 300 Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

float motor_FrontL = MINCOMMAND;
float motor_FrontR = MINCOMMAND;
float motor_BackL = MINCOMMAND;
float motor_BackR = MINCOMMAND;

//lag filter motor/
float motor_FrontLf = MINCOMMAND;
float motor_FrontLold = MINCOMMAND;
float motor_FrontLold2 = MINCOMMAND;
float motor_FrontRf = MINCOMMAND;
float motor_FrontRold = MINCOMMAND;
float motor_FrontRold2 = MINCOMMAND;
float motor_BackLf = MINCOMMAND;
float motor_BackLold = MINCOMMAND;
float motor_BackLold2 = MINCOMMAND;
float motor_BackRf = MINCOMMAND;
float motor_BackRold = MINCOMMAND;
float motor_BackRold2 = MINCOMMAND;
///////////////////
void motor_command_all() 
{
  for (int j = 0 ; j <= 50 ; j++)
  {
   motor_FrontL = MINCOMMAND;
   motor_FrontR = MINCOMMAND;
   motor_BackL = MINCOMMAND;
   motor_BackR = MINCOMMAND;
   
   OCR2B = motor_FrontL / 16 ;//PD3  1000-2000 to 128-256
   OCR1A = motor_BackR * 2 ;//PB1
   OCR1B = motor_FrontR * 2 ;//PB2
   OCR2A = motor_BackL / 16 ;//PB3
   delay(20);
  }
}
void motor_initialize() 
{
    DDRB = DDRB | B00001110;   // Set ports to output PB1-3
    DDRD = DDRD | B00001000;   // Set port to output PD3
        // Init PWM Timer 1  16 bit
    TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = PWM_COUNTER_PERIOD;
    // Init PWM Timer 2   8bit                                 // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
    TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);    // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
    TCCR2B = (1<<CS22)|(1<<CS21);                              // Prescaler set to 256, that gives us a resolution of 16us
    // TOP is fixed at 255                                     // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))
  motor_command_all();         // Initialise motors to MINCOMMAND (stopped)
}
void motor_command() 
{
   OCR2B = motor_FrontLf / 16 ;//PD3  1000-2000 to 128-256
   OCR1A = motor_BackRf * 2 ;//PB1  set PWM data (1000 - 2000)*2 to (2000 - 4000)
   OCR1B = motor_FrontRf * 2 ;//PB2 set PWM data (1000 - 2000)*2 to (2000 - 4000)
   OCR2A = motor_BackLf / 16 ;//PB3
}
void motor_Lag(){
  ////////lag lead compensator filter motor////////////////////////////////////
     float k_Lag = 2.95;//3.95 5.85 1.85 0.45 1.078 1.0 1.25
     float k_Lead = 25.5;//35.25
     float diff_motor_FrontL = (motor_FrontL - motor_FrontLold2)/0.02;
     float temp_motor_FrontL = diff_motor_FrontL + (motor_FrontL - motor_FrontLf)*k_Lead;//35.5
     motor_FrontLf = motor_FrontLf + (temp_motor_FrontL*G_Dt*k_Lag);
     float diff_motor_FrontR = (motor_FrontR - motor_FrontRold2)/0.02;
     float temp_motor_FrontR = diff_motor_FrontR + (motor_FrontR - motor_FrontRf)*k_Lead;//35.5
     motor_FrontRf = motor_FrontRf + (temp_motor_FrontR*G_Dt*k_Lag);         
     float diff_motor_BackL = (motor_BackL - motor_BackLold2)/0.02;
     float temp_motor_BackL = diff_motor_BackL + (motor_BackL - motor_BackLf)*k_Lead;//35.5
     motor_BackLf = motor_BackLf + (temp_motor_BackL*G_Dt*k_Lag);
     float diff_motor_BackR = (motor_BackR - motor_BackRold2)/0.02;
     float temp_motor_BackR = diff_motor_BackR + (motor_BackR - motor_BackRf)*k_Lead;//35.5
     motor_BackRf = motor_BackRf + (temp_motor_BackR*G_Dt*k_Lag);
     motor_FrontLold2 = motor_FrontLold;
     motor_FrontLold = motor_FrontL;//store PWM for next diff
     motor_FrontRold2 = motor_FrontRold;
     motor_FrontRold = motor_FrontR;
     motor_BackLold2 = motor_BackLold;
     motor_BackLold = motor_BackL;
     motor_BackRold2 = motor_BackRold;
     motor_BackRold = motor_BackR;
}
void motor_Mix(){
      motor_FrontL = uAltitude + u3_pitch*0.7071 + u2_roll*0.7071 + u4_yaw;//Front L , cos45 = 0.7071
      motor_FrontR = uAltitude + u3_pitch*0.7071 - u2_roll*0.7071 - u4_yaw;//Front R
      motor_BackL = uAltitude - u3_pitch*0.7071 + u2_roll*0.7071 - u4_yaw;//Back L
      motor_BackR = uAltitude - u3_pitch*0.7071 - u2_roll*0.7071 + u4_yaw;////Back R
      
       if (uAltitude >= 1460){
          motor_FrontLf = constrain((motor_FrontL + 54), MIN_FWD_THROTTLE, MAXTHROTTLE);
          motor_FrontRf = constrain((motor_FrontR + 54), MIN_FWD_THROTTLE, MAXTHROTTLE);
          motor_BackLf = constrain((motor_BackL + 54), MIN_FWD_THROTTLE, MAXTHROTTLE);
          motor_BackRf = constrain((motor_BackR + 54), MIN_FWD_THROTTLE, MAXTHROTTLE);
       }
        else{
          motor_FrontLf = constrain((motor_FrontL - 54), MINTHROTTLE, MIN_REV_THROTTLE);
          motor_FrontRf = constrain((motor_FrontR - 54), MINTHROTTLE, MIN_REV_THROTTLE);
          motor_BackLf = constrain((motor_BackL - 54), MINTHROTTLE, MIN_REV_THROTTLE);
          motor_BackRf = constrain((motor_BackR - 54), MINTHROTTLE, MIN_REV_THROTTLE);
        }
        if(armed == 1)
        {
         //motor_Lag();
         //motor_FrontLf = constrain(motor_FrontLf, MINTHROTTLE, MAXCOMMAND);//set PWM data (1000 - 2000)*2 to (2000 - 4000)
         //motor_FrontRf = constrain(motor_FrontRf, MINTHROTTLE, MAXCOMMAND);
         //motor_BackLf = constrain(motor_BackLf, MINTHROTTLE, MAXCOMMAND);
         //motor_BackRf = constrain(motor_BackRf, MINTHROTTLE, MAXCOMMAND);
        }
        else
        {
          roll_I_rate = 0.0;
          pitch_I_rate = 0.0;
          yaw_I_rate = 0.0;
          motor_FrontLf = MINCOMMAND;//set PWM data (1000 - 2000)*2 to (2000 - 4000)
          motor_FrontRf = MINCOMMAND;
          motor_BackLf = MINCOMMAND;
          motor_BackRf = MINCOMMAND;
        }
}
