#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "controller.h"

#include "driver/pcnt.h"
#include "floodfill.h"


/*
//test the timing make sure everything finishes within 1ms and leave at least 30% extra time for other code to excute in main routing
void systickHandler(void)
{
  Millis++;  //keep track of the system time in millisecond
  if(bUseIRSensor)
    readIRSensor();
  if(bUseGyro)
    readGyro();
  if(bUseSpeedProfile)
    speedProfile
}
*/



//ps. if you want to change speed, you simply change the targetSpeedX and targetSpeedW in main routing(int main) at any time.
//PUBLIC:
int targetSpeedX = 0;
int targetSpeedW = 0;
short onlyUseGyroFeedback = 0;
short useFrontSensor = 0;
short useEncGyroFeedback = 0;
short onlyUseEncoderFeedback = 0;
int DLSensor = 0; //Last IR - DL sensor mesure
int DRSensor = 0; //Last IR - DR sensor mesure
int LFSensor = 0; //Last IR - LF sensor mesure
int RFSensor = 0; //Last IR - RF sensor mesure
int SRSensor = 0; //Last IR - SR sensor mesure
int SLSensor = 0; //Last IR - SL sensor mesure
int aSpeed = 0;
int angle = 0;
//TO TELEMETRY:
int leftBaseSpeed = 0;
int rightBaseSpeed = 0;
double curSpeedX = 0;
double curSpeedW = 0;
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;



//PRIVATE:
double posErrorX = 0;
double posErrorW = 0;
double oldPosErrorX = 0;
double oldPosErrorW = 0;
int posPwmX = 0;
int posPwmW = 0;

long oldEncoderCount = 0; //used in move_one_cell

long leftEncoderChange=0;
long rightEncoderChange = 0;
long encoderChange = 0;

//long leftEncoderOld = 0;
//long rightEncoderOld = 0;

long leftEncoderCount = 0;
long rightEncoderCount = 0;
long encoderCount = 0;

int distanceLeft = 0;

int sensorError = 0;

//CONSTANTS - CONFIG
double kpX = 1.5, kdX = 28; //0.55 ; 28
double kpW = 0.9, kdW = 23;
//10:1                  double kpX = 0.6, kdX = 25;
//10:1                  double kpW = 1.9, kdW = 60;
//double kpW = 0.55, kdW = 19;//used in straight

//double kpW1 = 1;//used for T1 and T3 in curve turn
//double kdW1 = 26;
//double kpW2 = 1;//used for T2 in curve turn
//double kdW2 = 36;
double accX = 0.3;//0.15;//6m/s/s
double decX = 0.3;//0.15;
double accW = 0.4; //cm/s^2
double decW = 0.4;

double moveSpeed = 0;//speed_to_counts(0.3*2);;//speed_to_counts(0*2);  //1m/s  // x2 porque la velocidad es doble encoder_l+encoder_r
double turnSpeed = speed_to_counts(0.27*2);  //1m/s
double frontAligmentSpeed = speed_to_counts(0.05*2);
//int returnSpeed = speed_to_counts(1.0*2); //1m/s
//int stopSpeed = speed_to_counts(0.2*2); //0.2m/s
double maxSpeed = speed_to_counts(0.2*2);     //4m/s

double gyroFeedbackRatio = 12.25;//5700;//5900;
int a_scale = 25; //configured
int f_scale = 400;//configured

int DLMiddleValue = 835; //configured
int DRMiddleValue = 835; //configured

int SLMiddleValue = 1350; //configured
int SRMiddleValue = 1680; //configured

int thRightWall = 500;
int thLeftWall = 500;

//Longitudinal calibration
#define ALIGMENT_ONE_WALL
#define NO_DEC_IF_STRAIGHT
#define LONGITUDINAL_CALIBRATION
int hasWallDRSensorValue = 1300; //to be configured //min value to determine that there is wall in the DR sensor
int hasWallDLSensorValue = 1200; //to be configured //min value to determine that there is wall in the DL sensor
int diaRightWallFadingOffValue = 700; //to be configured //max value to determine if there is not wall in the DR sensor
int diaLeftWallFadingOffValue = 560; //to be configured //max value to determine if there is not wall in the DL sensor

//LFvalues1 and RFvalues1 are the front wall sensor threshold when the center of mouse between the boundary of the cells.
int LFvalue2 = 2410; //configured
int RFvalue2 = 1800; //configured
//LFvalues2 and RFvalues2 are the front wall sensor threshold when the center of the mouse staying half cell farther than LFvalues1 and 2
//LF/RFvalues2 are usually the threshold to determine if there is a front wall or not. You should probably move this 10mm closer to front wall when collecting these thresholds just in case the readings are too weak.
int LFvalue1 = 3600; //to be configured
int RFvalue1 = 3440; //to be configured

//volatile long count_enc_r = 0;
//volatile long count_enc_l = 0;

// void changeEnc1(){
//     static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
//     static uint8_t enc1_val = 0;

//     uint8_t v = (digitalRead(ENC_1_A)<<1) | digitalRead(ENC_1_B);

//     enc1_val = enc1_val << 2;
//     enc1_val = enc1_val | (v & 0b11);
//     count_enc_l = count_enc_l + lookup_table[enc1_val & 0b1111];
// }

//Encoder read idea: http://makeatronics.blogspot.com.es/2013/02/efficiently-reading-quadrature-with.html
// void changeEnc2(){
//     static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
//     static uint8_t enc2_val = 0;

//     int v = (digitalRead(ENC_2_A)<<1) | digitalRead(ENC_2_B);

//     enc2_val = enc2_val << 2;
//     enc2_val = enc2_val | (v & 0b11);
//     count_enc_r = count_enc_r+lookup_table[enc2_val & 0b1111];
// }

// void changeEnc11(){
//     static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
//     static uint8_t enc11_val = 0;

//     uint8_t v = (digitalRead(ENC_1_A)<<1) | digitalRead(ENC_1_B);

//     enc11_val = enc11_val << 2;
//     enc11_val = enc11_val | (v & 0b11);
//     count_enc_l = count_enc_l - lookup_table[enc11_val & 0b1111];
// }

// //Encoder read idea: http://makeatronics.blogspot.com.es/2013/02/efficiently-reading-quadrature-with.html
// void changeEnc22(){
//     static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
//     static uint8_t enc22_val = 0;

//     int v = (digitalRead(ENC_2_A)<<1) | digitalRead(ENC_2_B);

//     enc22_val = enc22_val << 2;
//     enc22_val = enc22_val | (v & 0b11);
//     count_enc_r = count_enc_r - lookup_table[enc22_val & 0b1111];
// }

// void encoder_setup(){
//     pinMode(ENC_1_B,INPUT_PULLUP);
//     pinMode(ENC_1_A,INPUT_PULLUP);
//     pinMode(ENC_2_B,INPUT_PULLUP);
//     pinMode(ENC_2_A,INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(ENC_1_B), changeEnc1, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(ENC_2_B), changeEnc2, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(ENC_1_A), changeEnc11, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(ENC_2_A), changeEnc22, CHANGE);
// }


void configQEI_1(){
    pinMode(ENC_1_B,INPUT_PULLUP);
    pinMode(ENC_1_A,INPUT_PULLUP);

  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = ENC_1_A; //set gpio4 as pulse input gpio
  pcnt_config.ctrl_gpio_num = ENC_1_B; //set gpio5 as control gpio
  pcnt_config.channel = PCNT_CHANNEL_0; //use unit 0 channel 0
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; //when control signal is low, reverse the primary counter mode(inc->dec/dec->inc)
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP; //when control signal is high, keep the primary counter mode
  pcnt_config.pos_mode = PCNT_COUNT_INC; //increment the counter
  pcnt_config.neg_mode = PCNT_COUNT_DEC; //keep the counter value
  pcnt_config.counter_h_lim = 10000;
  pcnt_config.counter_l_lim = -10000;
  pcnt_config.unit = PCNT_UNIT_0;

 pcnt_unit_config(&pcnt_config);

 pcnt_config.pulse_gpio_num = ENC_1_A; //set gpio4 as pulse input gpio
 pcnt_config.ctrl_gpio_num = ENC_1_B;
 pcnt_config.unit = PCNT_UNIT_1;
 pcnt_unit_config(&pcnt_config);
}

void configQEI_2(){
      pinMode(ENC_2_B,INPUT_PULLUP);
      pinMode(ENC_2_A,INPUT_PULLUP);

  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = ENC_2_A; //set gpio4 as pulse input gpio
  pcnt_config.ctrl_gpio_num = ENC_2_B; //set gpio5 as control gpio
  pcnt_config.channel = PCNT_CHANNEL_1; //use unit 0 channel 0
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; //when control signal is low, reverse the primary counter mode(inc->dec/dec->inc)
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP; //when control signal is high, keep the primary counter mode
  pcnt_config.pos_mode = PCNT_COUNT_INC; //increment the counter
  pcnt_config.neg_mode = PCNT_COUNT_DEC; //keep the counter value
  pcnt_config.counter_h_lim = 10000;
  pcnt_config.counter_l_lim = -10000;
  pcnt_config.unit = PCNT_UNIT_2;

 pcnt_unit_config(&pcnt_config);

 pcnt_config.pulse_gpio_num = ENC_2_A; //set gpio4 as pulse input gpio
 pcnt_config.ctrl_gpio_num = ENC_2_B;
 pcnt_config.unit = PCNT_UNIT_3;
 pcnt_unit_config(&pcnt_config);
}

inline int16_t read_clear_encoder1(){
   int16_t count1=0;
  pcnt_get_counter_value(PCNT_UNIT_0, &count1);
  pcnt_counter_clear(PCNT_UNIT_0);

  int16_t count2=0;
  pcnt_get_counter_value(PCNT_UNIT_1, &count2);
  pcnt_counter_clear(PCNT_UNIT_1);

  return count1+count2;
}

inline int16_t read_clear_encoder2(){
   int16_t count1=0;
  pcnt_get_counter_value(PCNT_UNIT_2, &count1);
  pcnt_counter_clear(PCNT_UNIT_2);

  int16_t count2=0;
  pcnt_get_counter_value(PCNT_UNIT_3, &count2);
  pcnt_counter_clear(PCNT_UNIT_3);

  return count1+count2;
}

void elapseMicros(uint32_t targetTime, uint32_t oldt){
 while((micros()-oldt)<targetTime);
}

void readGyro(void)
{                           //k=19791(sum for sample in 1 second)    101376287 for 50 seconds with 5000 samples
      int i;
      int sampleNum = 10;
      aSpeed = 0;
      for(i=0;i<sampleNum;i++)
            aSpeed += analogRead(GYRO);
      aSpeed *= 50000/sampleNum;
      aSpeed -= 95595700;//92980000;
      aSpeed /= 50000;
      aSpeed /= 4;
      angle += aSpeed;
}

void readIRsensors(){

  unsigned long  curt=micros();



//diagonal sensors
  SRSensor = analogRead(I_IR_LAT_L);
  DLSensor = analogRead(I_IR_DIA_R);
  digitalWrite(O_IR_DIA,HIGH);
  elapseMicros(70,curt);
  SRSensor = analogRead(I_IR_LAT_L) - SRSensor;
  DLSensor = analogRead(I_IR_DIA_R) - DLSensor;
  digitalWrite(O_IR_DIA,LOW);

  elapseMicros(170,curt);

//right front sensor
  RFSensor = analogRead(I_IR_F_R);
  digitalWrite(O_IR_F_R,HIGH);
  elapseMicros(240,curt);
  RFSensor = analogRead(I_IR_F_R) - RFSensor;
  digitalWrite(O_IR_F_R,LOW);

  elapseMicros(340,curt);


//side sensors
  DRSensor = analogRead(I_IR_DIA_L);
  SLSensor = analogRead(I_IR_LAT_R);
  digitalWrite(O_IR_LAT,HIGH);
  elapseMicros(420,curt);
  DRSensor = analogRead(I_IR_DIA_L) - DRSensor;
  SLSensor = analogRead(I_IR_LAT_R) - SLSensor;
  digitalWrite(O_IR_LAT,LOW);

  elapseMicros(520,curt);

//left front sensor
  LFSensor = analogRead(I_IR_F_L);
  digitalWrite(O_IR_F_L,HIGH);
  elapseMicros(600,curt);
  LFSensor = analogRead(I_IR_F_L) - LFSensor;
  digitalWrite(O_IR_F_L,LOW);

  //elapseMicros(680,curt);
  //
  //
  //
  //
  if(DLSensor < 0)
    DLSensor = 0;
  if(SRSensor < 0)
    SRSensor = 0;
  if(LFSensor < 0)//error check
    LFSensor = 0;
  if(SLSensor < 0)
    SLSensor = 0;
  if(DRSensor < 0)
    DRSensor = 0;
  if(RFSensor < 0)
    RFSensor = 0;
}

void getEncoderStatus(void)
{
   leftEncoderChange = read_clear_encoder1();//count_enc_l;
   rightEncoderChange = read_clear_encoder2(); //count_enc_r;

  //leftEncoderChange = 1.0323*leftEncoder - 1.0323*leftEncoderOld;
  //rightEncoderChange = rightEncoder - rightEncoderOld;
  encoderChange = (leftEncoderChange + rightEncoderChange)/2;

  //leftEncoderOld = leftEncoder;
  //rightEncoderOld = rightEncoder;

  leftEncoderCount += leftEncoderChange;
  rightEncoderCount += rightEncoderChange;
  encoderCount =  (leftEncoderCount+rightEncoderCount);

  distanceLeft -= (leftEncoderChange + rightEncoderChange);// update distanceLeft
}



void updateCurrentSpeed(void)
{
  if(curSpeedX < targetSpeedX)
  {
    curSpeedX += (double)(speed_to_counts(accX*2)/100);
    if(curSpeedX > targetSpeedX)
      curSpeedX = targetSpeedX;
  }
  else if(curSpeedX > targetSpeedX)
  {
    curSpeedX -= (double)speed_to_counts(decX*2)/100;
    if(curSpeedX < targetSpeedX)
      curSpeedX = targetSpeedX;
  }
  if(curSpeedW < targetSpeedW)
  {
    curSpeedW += accW;
    if(curSpeedW > targetSpeedW)
      curSpeedW = targetSpeedW;
  }
  else if(curSpeedW > targetSpeedW)
  {
    curSpeedW -= decW;
    if(curSpeedW < targetSpeedW)
      curSpeedW = targetSpeedW;
  }
}


void getSensorEror(){
  if(SLSensor > SLMiddleValue && SRSensor < SRMiddleValue && LFSensor<LFvalue1*0.4){
    sensorError = SLMiddleValue - SLSensor;
    digitalWrite(LED_L,HIGH);
  }else if(SRSensor > SRMiddleValue && SLSensor < SLMiddleValue && RFSensor<RFvalue1*0.4){
    sensorError = SRSensor - SRMiddleValue;
      digitalWrite(LED_R,HIGH);
  }else{
      #ifdef ALIGMENT_ONE_WALL
            if( SRSensor < thRightWall &&  SLSensor > thLeftWall && DRSensor>150 && LFSensor<LFvalue1*0.4){ //pared en la izq y NO en la der
                  sensorError = (SLMiddleValue - SLSensor)/4;
            }else if( SLSensor < thLeftWall &&  SRSensor > thRightWall && DLSensor>150 && RFSensor<RFvalue1*0.4){ //pared en la derecha y NO en la izq
                  sensorError = (SRSensor - SRMiddleValue)/4;
            }else
      #endif
                  sensorError = 0;
                  digitalWrite(LED_R,LOW);
                  digitalWrite(LED_L,LOW);
  }
}

void setLeftPwm(int p){
      if(p < 0){
            digitalWrite(LMOTOR_DIR, LOW);
            p = -p;
      }else{
            digitalWrite(LMOTOR_DIR, HIGH);
      }
      if(p>1024)p=1024;
      double pow = map(p, 0,1024,0,1000);
      ledcWrite(LMOTOR_CH, pow);
}

void setRightPwm(int p){
      if(p < 0){
            digitalWrite(RMOTOR_DIR, HIGH);
            p = -p;
      }else{
            digitalWrite(RMOTOR_DIR, LOW);
      }
      if(p>1024)p=1024;
      double pow = map(p, 0,1024,0,1000);
      ledcWrite(RMOTOR_CH, pow);
}

int countData=0;
long countSpeedProfile=0;
double dataTime[N_TEL];
double data1[N_TEL];
double data2[N_TEL];
double data3[N_TEL];
double data4[N_TEL];
double data5[N_TEL];
double data6[N_TEL];
double data7[N_TEL];
double data8[N_TEL];
//double data9[N_TEL];
//double data10[N_TEL];

void calculateMotorPwm(void) // encoder PD controller
{
  double gyroFeedback;
  double rotationalFeedback;
  double sensorFeedback;

    /* simple PD loop to generate base speed for both motors */
  encoderFeedbackX = rightEncoderChange + leftEncoderChange;
  encoderFeedbackW = rightEncoderChange - leftEncoderChange;

  readGyro();
  gyroFeedback = -aSpeed/gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture
  readIRsensors();
  getSensorEror();
  sensorFeedback = -sensorError/a_scale;//have sensor error properly scale to fit the system

  if(onlyUseGyroFeedback)
    rotationalFeedback = gyroFeedback;
  else if(onlyUseEncoderFeedback)
    rotationalFeedback = encoderFeedbackW;
  else if(useFrontSensor){
      double frontSensorFeedback = (RFSensor - RFvalue2 + LFvalue2 - LFSensor)/f_scale;
      rotationalFeedback = (encoderFeedbackW+frontSensorFeedback);
  }else if(useEncGyroFeedback){
      rotationalFeedback = (encoderFeedbackW + gyroFeedback)/2;
  }
  else
    rotationalFeedback = (1*encoderFeedbackW + 3*gyroFeedback + 3*sensorFeedback)/7;
      //if you use IR sensor as well, the line above will be rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;
      //make sure to check the sign of sensor error.


  posErrorX += curSpeedX - encoderFeedbackX;
  posErrorW += curSpeedW - rotationalFeedback;

  posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
  posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);

  oldPosErrorX = posErrorX;
  oldPosErrorW = posErrorW;

  leftBaseSpeed = posPwmX - posPwmW;
  rightBaseSpeed = posPwmX + posPwmW;

  setLeftPwm(leftBaseSpeed); //setLeftPwm( (LFvalue2 - LFSensor)/7);
  setRightPwm(rightBaseSpeed); //setRightPwm( (RFvalue2 - RFSensor)/ 7);

if(countSpeedProfile%5)
     if(countData < N_TEL){
       dataTime[countData]=micros();
       data1[countData]=rightEncoderChange;
       data2[countData]=leftEncoderChange;
       data3[countData]=curSpeedW;
       data4[countData]=curSpeedX;
       data5[countData]=gyroFeedback;
       data6[countData]= DRSensor;//sensorFeedback;//SRSensor;//leftBaseSpeed;
       data7[countData]= DLSensor;//rotationalFeedback;//SLSensor; //rightBaseSpeed;
       data8[countData]=encoderFeedbackW;
 //      data9[countData]=rightBaseSpeed;


       countData++;
     }

   countSpeedProfile++;
}


void speedProfile(void)
{
  getEncoderStatus();
  adcStart(GYRO); //comienza la lectura analogica asincrona del gyro para ser leido el valor finalmente en la funcion calculateMotorPwm()
  updateCurrentSpeed();
  calculateMotorPwm();
}

void resetSpeedProfile(void)
{
  //resetEverything;

  //disable sensor data collecting functions running in 1ms interrupt
  setLeftPwm(0);
  setRightPwm(0);

  curSpeedX = 0;
  curSpeedW = 0;
  targetSpeedX = 0;
  targetSpeedW = 0;
  posErrorX = 0;
  posErrorW = 0;
  oldPosErrorX = 0;
  oldPosErrorW = 0;
//  leftEncoderOld = 0;
//  rightEncoderOld = 0;

  leftEncoderCount = 0;
  rightEncoderCount = 0;
  encoderCount = 0;
  oldEncoderCount = 0;
  leftBaseSpeed = 0;
  rightBaseSpeed = 0;

//  count_enc_l = 0;//reset left encoder count
//  count_enc_r = 0;//reset right encoder count
}


double needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)//speed are in encoder counts/ms, dist is in encoder counts
{
  if (curSpd<0) curSpd = -curSpd;
  if (endSpd<0) endSpd = -endSpd;
  if (dist<0) dist = 1;//-dist;
  if (dist == 0) dist = 1;  //prevent divide by 0
  int t_ms = ceil(((curSpd-endSpd)*1.0)/(double)(speed_to_counts(decX*2)/100));
  return (curSpd*t_ms - ((double)(speed_to_counts(decX*2)/100)*t_ms*t_ms)/2. )*1.2;


//  return ( abs(  (((double)(curSpd))*((double)(curSpd)) - ((double)(endSpd))*((double)(endSpd)))/(dist)/2.0)); //dist_counts_to_mm(dist)/2);

  //calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
  //use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
  //because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
}

void aligmentFrontWall(){
      while( (LFSensor < LFvalue1 && LFSensor > LFvalue2*0.8)//if has front wall, make the mouse finish this 180mm with sensor threshold only
        || (RFSensor < RFvalue1 && RFSensor > RFvalue2*0.8)
  ){
      digitalWrite(LED_F,HIGH);
      targetSpeedX = frontAligmentSpeed;
      useFrontSensor = 1;
  }
  // if(useFrontSensor){
  //     int b_f_scale= f_scale;
  //     while( abs(LFSensor - LFvalue1)>25 || abs(RFSensor - RFvalue1)>25){

  //        f_scale=100;
  //     }
  //     f_scale = b_f_scale;
  // }
  targetSpeedX=moveSpeed;
  useFrontSensor = 0;
  digitalWrite(LED_F,LOW);
}
/*
sample code for straight movement
*/


void moveOneCell(short isExploring){
bool hasRightWall=false;
bool hasLeftWall=false;
  //enable_sensor(),
  //enable_gyro();
  //enable_PID();

  targetSpeedW = 0;
  targetSpeedX = moveSpeed;
  distanceLeft= ONE_CELL_DISTANCE - (encoderCount-oldEncoderCount);
  int ffRunned=0;
 // digitalWrite(LED_F,LOW);
  do{
    /*you can call int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)
    here with current speed and distanceLeft to decide if you should start to decelerate or not.*/
    /*sample*/
    delay(1);
    // if(countData<N_TEL-1){
    //   dataTime[countData]=micros();
    //   data1[countData]=distanceLeft;
    //   data2[countData]=((DLSensor-0)>0 ? (DLSensor-0):0); //Last IR - DL sensor mesure
    //   data3[countData]=DRSensor; //Last IR - DR sensor mesure
    //   data4[countData]=curSpeedX; //LFSensor; //sensorError/a_scale;
    //   data5[countData]=SLSensor; //encoderFeedbackW; //Last IR - RF sensor mesure
    //   data6[countData]=SRSensor; //Last IR - SR sensor mesure
    //   data7[countData]= encoderFeedbackX;//SLSensor;
    //   data8[countData]=needToDecelerate(distanceLeft, curSpeedX, moveSpeed); //posErrorW;
    //   countData++;
    // }
    if( distanceLeft < 1500 && ffRunned==0 && isExploring==1){
            ffRunned=1;
            xTaskCreatePinnedToCore(FFtask, "FF",10000,NULL, 0,NULL,0);
    }

#ifdef LONGITUDINAL_CALIBRATION
      if(SRSensor > hasWallDRSensorValue)
             hasRightWall = true;
      if(SLSensor > hasWallDLSensorValue)
             hasLeftWall = true;
      if(SRSensor < diaRightWallFadingOffValue &&  hasRightWall == true){
             hasRightWall = false;
             playNoteNoDelay(600,150);
             distanceLeft = speed_to_counts(150*2);
      }
      if(SLSensor < diaLeftWallFadingOffValue &&  hasLeftWall == true){
             hasLeftWall = false;
             playNoteNoDelay(850,150);
             distanceLeft = speed_to_counts(155*2);
      }
#endif
//    if(needToDecelerate(distanceLeft, curSpeedX, moveSpeed) < ((double)speed_to_counts(decX*2)/100.0)){
    if(needToDecelerate(distanceLeft, curSpeedX, moveSpeed) < distanceLeft){
            targetSpeedX = maxSpeed;
    }else{

      #ifdef NO_DEC_IF_STRAIGHT
      if( ((ffRunned!=0 && runningFF==0) || !isExploring) && getNextMovement(xRobot,yRobot,robotDir)==STRAIGHT){
            targetSpeedX = maxSpeed;
      }else{
            targetSpeedX = moveSpeed;
      }
      #else
            targetSpeedX = moveSpeed;
      #endif

    }

    //there is something else you can add here. Such as detecting falling edge of post to correct longitudinal position of mouse when running in a straight path
  }while( ( distanceLeft > 0 && LFSensor < LFvalue1+1 && RFSensor < RFvalue1+1));//use encoder to finish 180mm movement if no front walls

      aligmentFrontWall();


  targetSpeedX = moveSpeed;
  //LFvalues1 and RFvalues1 are the front wall sensor threshold when the center of mouse between the boundary of the cells.
  //LFvalues2 and RFvalues2 are the front wall sensor threshold when the center of the mouse staying half cell farther than LFvalues1 and 2
  //and LF/RFvalues2 are usually the threshold to determine if there is a front wall or not. You should probably move this 10mm closer to front wall when collecting
  //these thresholds just in case the readings are too weak.

  oldEncoderCount = encoderCount; //update here for next movement to minimized the counts loss between cells.
}

void giro180(){
     onlyUseGyroFeedback=1;
     targetSpeedX = 0;
     targetSpeedW = turnSpeed;

     delay(374);
     targetSpeedW = 0;
     targetSpeedX = 0;
     delay(150);
     onlyUseGyroFeedback=0;
     oldEncoderCount = encoderCount;
}

void giro90l(){
      onlyUseGyroFeedback=1;
     targetSpeedX = 0;
     targetSpeedW = turnSpeed;

     delay(184);
     targetSpeedW = 0;
     targetSpeedX = 0;
     delay(100);
     onlyUseGyroFeedback=0;
     oldEncoderCount = encoderCount;
}

void giro90r(){
      onlyUseGyroFeedback=1;
     targetSpeedX = 0;
     targetSpeedW = -turnSpeed;

     delay(184);
     targetSpeedW = 0;
     targetSpeedX = 0;
     delay(100);
     onlyUseGyroFeedback=0;
     oldEncoderCount = encoderCount;
}


void turn(uint16_t t1,uint16_t t2,uint16_t t3,double maxAngularVelocity, double maxAccW, double maxDecW, double kpW1, double kdW1, double kpW2, double kdW2){
double BaccW = accW;
double BdecW = decW;

double Straight_kpW= kpW;
double Straight_kdW= kdW;

      accW=maxAccW;
      decW=maxDecW;
      long curl = millis();
      while(millis()< (curl +t1)){
            kpW = kpW1;
            kdW = kdW1;
            targetSpeedW = maxAngularVelocity;
            delay(1);
      }
      curl = millis();
      while(millis()< (curl +t2)){
            kpW = kpW2;
            kdW = kdW2;
            targetSpeedW = maxAngularVelocity;
            delay(1);
      }

      curl = millis();
      while(millis()< (curl+t3)){
            kpW = kpW1;
            kdW = kdW1;
            targetSpeedW = 0;
            delay(1);
      }
accW = BaccW; //cm/s^2
decW = BdecW;

kpW = Straight_kpW;
kdW = Straight_kdW;

}

void L90(){
      useEncGyroFeedback=1;
//      targetSpeedX = 0;//30.52644626;
      //delay(250);
      //turn(25,433,25,7.12, 0.2898, 0.2898, 0.5,30,0.5,30);
      turn(40,387,40,8, 0.2, 0.2, 0.85,35,0.85,40);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;
}

void R90(){
      useEncGyroFeedback=1;
//      targetSpeedX = 0;//30.52644626;
      //delay(250);
      turn(40,387,40,-8, 0.2, 0.2, 0.85,35,0.85,40);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;
}

void R180(){
      onlyUseEncoderFeedback=1;
//      targetSpeedX = 0;//30.52644626;
      //delay(250);
      //turn(25,891,25,-7.12, 0.2898, 0.2898, 0.5,30,0.5,30);
      turn(40,812,40,-8, 0.2, 0.2, 0.85,35,0.85,40);
      onlyUseEncoderFeedback=0;
      oldEncoderCount = encoderCount;
}


int readButton(){
      return !digitalRead(BUTTON);
}

void allLedsOn(){
      digitalWrite(LED_F,HIGH);
      digitalWrite(LED_R,HIGH);
      digitalWrite(LED_L,HIGH);
}

void allLedsOff(){
      digitalWrite(LED_F,LOW);
      digitalWrite(LED_R,LOW);
      digitalWrite(LED_L,LOW);
}


void L90ff(){
      double old_kpX = kpX; //= 0.55,
      double old_kdX = kdX; // = 19;
      kpX = 1;//2.3;
      kdX = 25;//55;
      useEncGyroFeedback=1;
      countData=0;
      turn(30,186,30,15, 0.5, 0.5, 2.8,55,1.8,60);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;

      kpX = old_kpX;
      kdX = old_kdX;
}
void R90ff(){
      double old_kpX = kpX; //= 0.55,
      double old_kdX = kdX; // = 19;
      kpX = 1;//2.3;
      kdX = 25;//55;
      useEncGyroFeedback=1;
      countData=0;
      turn(30,186,30,-15, 0.5, 0.5, 2.8,55,1.8,60);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;

      kpX = old_kpX;
      kdX = old_kdX;
}

void L180ff(){
      double old_kpX = kpX; //= 0.55,
      double old_kdX = kdX; // = 19;
      kpX = 1;//2.3;
      kdX = 25;//55;
      useEncGyroFeedback=1;
      countData=0;
      turn(30,405,30,15, 0.5, 0.5, 2.8,55,1.9,60);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;

      kpX = old_kpX;
      kdX = old_kdX;
}


void L90f(){
      useEncGyroFeedback=1;
      countData=0;
      turn(50,174,50,15, 0.3, 0.3, 1.6,22,1.6,40);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;
}
void R90f(){
      useEncGyroFeedback=1;
      countData=0;
      turn(50,174,50,-15, 0.3, 0.3, 1.6,22,1.6,40);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;
}

void L180f(){
      useEncGyroFeedback=1;
      countData=0;
      turn(50,403,50,15, 0.3, 0.3, 1.6,22,1.6,40);
      useEncGyroFeedback=0;
      oldEncoderCount = encoderCount;
}







