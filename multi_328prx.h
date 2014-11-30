/*
project_Quad 3d Fixed Pitch   
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
https://www.facebook.com/tinnakonza
*/
int CH_THR;
int CH_AIL;
int CH_ELE;
int CH_RUD;
int AUX_1;

int CH_AIL_Cal = 1500;
int CH_ELE_Cal = 1500;
int CH_RUD_Cal = 1500;

//RX PIN assignment inside the port //for PORTK
#define THROTTLEPIN                2  //
#define ROLLPIN                    4  //
#define PITCHPIN                   5  //
#define YAWPIN                     6  //
#define AUX1PIN                    7  //
#define ISR_UART                   ISR(USART0_UDRE_vect)

#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4     

#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

static uint8_t pinRcChannel[8] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,5,6,7};
volatile uint16_t rcPinValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
static int16_t rcData[8] ;
static int16_t rcCommand[4] ; 
static int16_t rcHysteresis[8] ;
static int16_t rcData4Values[8][4];

volatile uint16_t rcValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500};

unsigned long timer=0;

void configureReceiver() {
    for (uint8_t chan = 0; chan < 5; chan++){
      for (uint8_t a = 0; a < 4; a++){
        rcData4Values[chan][a] = 1500;
      }  
    }    
      PORTD   = (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7); 
      PCMSK2 |= (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
      PCICR   = 1<<2;
}
ISR(PCINT2_vect) { 
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;
  pin = PIND;            
  mask = pin ^ PCintLast;   
  sei();                   
  PCintLast = pin;     
  cTime = micros();        
  if (mask & 1<<2)          
    if (!(pin & 1<<2)) {    
      dTime = cTime-edgeTime[2]; if (900<dTime && dTime<2200) rcPinValue[2] = dTime; 
    } else edgeTime[2] = cTime;   
  if (mask & 1<<4)   
    if (!(pin & 1<<4)) {
      dTime = cTime-edgeTime[4]; if (900<dTime && dTime<2200) rcPinValue[4] = dTime;
    } else edgeTime[4] = cTime;
  if (mask & 1<<5)
    if (!(pin & 1<<5)) {
      dTime = cTime-edgeTime[5]; if (900<dTime && dTime<2200) rcPinValue[5] = dTime;
    } else edgeTime[5] = cTime;
  if (mask & 1<<6)
    if (!(pin & 1<<6)) {
      dTime = cTime-edgeTime[6]; if (900<dTime && dTime<2200) rcPinValue[6] = dTime;
    } else edgeTime[6] = cTime;
  if (mask & 1<<7)
    if (!(pin & 1<<7)) {
      dTime = cTime-edgeTime[7]; if (900<dTime && dTime<2200) rcPinValue[7] = dTime;
    } else edgeTime[7] = cTime;
}
uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  data = rcPinValue[pinRcChannel[chan]];
  SREG = oldSREG;
  return data; 
}
void computeRC() {
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  rc4ValuesIndex++;
  for (chan = 0; chan < 5; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcData[chan] = 0;
    for (a = 0; a < 4; a++){
      rcData[chan] += rcData4Values[chan][a];
    }
    rcData[chan]= (rcData[chan]+2)/4;
    if ( rcData[chan] < rcHysteresis[chan] -3)  rcHysteresis[chan] = rcData[chan]+2;
    if ( rcData[chan] > rcHysteresis[chan] +3)  rcHysteresis[chan] = rcData[chan]-2;
  }
    CH_THR = rcHysteresis[THROTTLE];
    CH_AIL = rcHysteresis[ROLL];
    CH_ELE = rcHysteresis[PITCH];
    CH_RUD = rcHysteresis[YAW];
    AUX_1 = rcHysteresis[AUX1];
}
void RC_Calibrate(){
  Serial.print("RC_Calibrate");Serial.println("\t");
  for (int i = 0; i < 10; i++) {
    computeRC();
    delay(20);
  }
  CH_AIL_Cal = CH_AIL;
  CH_ELE_Cal = CH_ELE;
  CH_RUD_Cal = CH_RUD;
    Serial.print(CH_AIL_Cal);Serial.print("\t");
    Serial.print(CH_ELE_Cal);Serial.print("\t");
    Serial.print(CH_RUD_Cal);Serial.println("\t");
}
