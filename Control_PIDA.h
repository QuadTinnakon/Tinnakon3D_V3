/*
project_Quad 3d Fixed Pitch   
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
https://www.facebook.com/tinnakonza
*/

float roll_I_rate;
float roll_D_rate;
float err_roll_rate;
float err_roll_ant_rate;

float pitch_I_rate;
float pitch_D_rate;
float err_pitch_rate;
float err_pitch_ant_rate;

float yaw_I_rate;
float yaw_D_rate;
float err_yaw_rate=0;
float err_yaw_ant_rate=0;

//PID mode control
float u2_roll = 0.0;
float u3_pitch = 0.0;
float u4_yaw = 0.0;
float uAltitude = 1460.0;

void Control_PIDRate(){
            // ROLL CONTROL
            float Set_roll = ((CH_AIL-CH_AIL_Cal)*K_Rate);
            applyDeadband(Set_roll, 6.5);//10 = 2.5
            err_roll_rate = (Set_roll - GyroXf*RAD_TO_DEG);
            roll_I_rate += err_roll_rate*Ki_rateRoll*G_Dt; 
            roll_I_rate = constrain(roll_I_rate, -500, 500); 
            roll_D_rate = (tar*roll_D_rate/(tar+G_Dt))+((err_roll_rate - err_roll_ant_rate)/(tar+G_Dt)); 
            err_roll_ant_rate = err_roll_rate;        
            u2_roll = Kp_rateRoll*err_roll_rate + roll_I_rate + Kd_rateRoll*roll_D_rate; 
            u2_roll = constrain(u2_roll, -600, 600);//300 +-400
            // PITCH CONTROL
            float Set_pitch = ((CH_ELE-CH_ELE_Cal)*-K_Rate);
            applyDeadband(Set_pitch, 6.5);//10 = 2.5
            err_pitch_rate = (Set_pitch - GyroYf*RAD_TO_DEG);   
            pitch_I_rate += err_pitch_rate*Ki_ratePitch*G_Dt;  
            pitch_I_rate = constrain(pitch_I_rate, -500, 500);
            pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt))+((err_pitch_rate - err_pitch_ant_rate)/(tar+G_Dt));       
            err_pitch_ant_rate = err_pitch_rate;   
            u3_pitch = Kp_ratePitch*err_pitch_rate + pitch_I_rate + Kd_ratePitch*pitch_D_rate;   
            u3_pitch = constrain(u3_pitch, -600, 600);//300 //+-400
            // YAW CONTROL
            float Set_yaw = ((CH_RUD-CH_RUD_Cal)*-K_Rate);
            applyDeadband(Set_yaw, 6.5);//10 = 2.5
            err_yaw_rate = (Set_yaw - GyroYf*RAD_TO_DEG);  
            yaw_I_rate += err_yaw_rate*Ki_rateYaw*G_Dt;     
            yaw_I_rate = constrain(yaw_I_rate, -100, 100);        
            yaw_D_rate = (tar*yaw_D_rate/(tar+G_Dt))+((err_yaw_rate - err_yaw_ant_rate)/(tar+G_Dt));    
            err_yaw_ant_rate = err_yaw_rate;         
            u4_yaw = (Kp_rateYaw*err_yaw_rate) + yaw_I_rate + (Kd_rateYaw*yaw_D_rate); 
            u4_yaw = constrain(u4_yaw, -250, 250);//250 +-400
            //Altitude CONTROL
            uAltitude = map(CH_THR, 1050, 1950, MINTHROTTLE, MAXTHROTTLE);
            uAltitude = constrain(uAltitude, MINTHROTTLE, MAXTHROTTLE);   
}
