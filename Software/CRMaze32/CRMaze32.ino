#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <pgmspace.h>

#include "controller.h"
#include "floodfill.h"
//#include "telemetry.h"

#define PWM_FREQ 5000 //frequency in Hz of PWM

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//volatile int interruptCounter;
//int totalInterruptCounter;

#define BUZZER_PIN 16
#define BUTTON 0

#define DO 262
#define RE 294
#define MI 330
#define FA 349
#define SOL 392
#define LA 440
#define SI 494



// void coreTask( void * pvParameters ){
//    pinMode(ENC_1_B,INPUT_PULLUP);
//    pinMode(ENC_1_A,INPUT_PULLUP);
//    pinMode(ENC_2_B,INPUT_PULLUP);
//    pinMode(ENC_2_A,INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(ENC_1_B), changeEnc1, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(ENC_2_B), changeEnc2, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(ENC_1_A), changeEnc11, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(ENC_2_A), changeEnc22, CHANGE);
//    while(1)
//       delay(1000);
// }
//

void configMode(){
  int mode1=0;
  long curl = millis();
  while(curl+1000 > millis()){
      allLedsOn();
      delay(100);
      if(readButton()==1){ //button presed
            mode1=1;
      }
      allLedsOff();
      delay(100);
  }

  if(mode1){
      EEPROM_load_maze();
      playNoteNoDelay(1000,500);
      initializeMinDistances();
      delay(1000);
      // Serial.begin(115200);
      // initializeDistancesIniGoal();
      // printMaze(0,0,1);
      // while(1){

      // }
  }else{
      initializeDistancesCenterGoal();
  }
}

void startBuzzer(int freq){
      ledcWriteTone(0, freq);
      ledcWrite(0, 125);
}

void stopBuzzer(){
      ledcWrite(0, 0);
}

void playNote(int freq, int dur){
      startBuzzer(freq);
      delay(dur);
      stopBuzzer();
}

typedef struct{
      int freq;
      int dur_ms;
}sound;

void playNoteTask(void *ss){
      sound *s = (sound *)ss;
      startBuzzer(s->freq);
      delay(s->dur_ms);
      stopBuzzer();
      vTaskDelete(NULL);

}
sound s;
void playNoteNoDelay(int freq, int dur){
      s.freq=freq;
      s.dur_ms = dur;
      xTaskCreatePinnedToCore(playNoteTask, "buzzer",10000,(void *)&s, 0,NULL,0);
}

void setup() {

  pinMode(O_IR_F_L, OUTPUT); digitalWrite(O_IR_F_L,LOW);
  pinMode(O_IR_F_R, OUTPUT); digitalWrite(O_IR_F_R,LOW);
  pinMode(O_IR_DIA, OUTPUT); digitalWrite(O_IR_DIA,LOW);
  pinMode(O_IR_LAT, OUTPUT); digitalWrite(O_IR_LAT,LOW);

  // put your setup code here, to run once:
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &speedProfile, true);
  timerAlarmWrite(timer, 1000, true); //interrupt each ms


  //motor pins setup
  ledcSetup(LMOTOR_CH, PWM_FREQ, 10);
  ledcAttachPin(LMOTOR_EN, LMOTOR_CH);
  ledcSetup(RMOTOR_CH, PWM_FREQ, 10);
  ledcAttachPin(RMOTOR_EN, RMOTOR_CH);
  pinMode(RMOTOR_DIR, OUTPUT);
  pinMode(LMOTOR_DIR, OUTPUT);
  digitalWrite(LMOTOR_EN,LOW);digitalWrite(LMOTOR_DIR,LOW);
  digitalWrite(RMOTOR_EN,LOW);digitalWrite(RMOTOR_DIR,LOW);
  //gyro pin setup
  adcAttachPin(GYRO);


  //IR pins setup
  adcAttachPin(I_IR_F_L);
  adcAttachPin(I_IR_F_R);
  adcAttachPin(I_IR_DIA_L);
  adcAttachPin(I_IR_DIA_R);
  adcAttachPin(I_IR_LAT_L );
  adcAttachPin(I_IR_LAT_R);



   // Serial.begin(115200);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_L, OUTPUT);
  pinMode(LED_F, OUTPUT);

  pinMode(ENC_1_B,INPUT_PULLUP);
  pinMode(ENC_1_A,INPUT_PULLUP);
  pinMode(ENC_2_B,INPUT_PULLUP);
  pinMode(ENC_2_A,INPUT_PULLUP);

  pinMode(BUTTON, INPUT_PULLUP);
  //init_buzzer();
  pinMode(BUZZER_PIN,OUTPUT);
  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER_PIN, 0);



  playNoteNoDelay(DO,1000);
  for(int _=0;_<4;_++){allLedsOn();delay(125);allLedsOff();delay(125);}
  configMode();

  //encoder_setup();
  //xTaskCreatePinnedToCore(coreTask, "encoders",10000,NULL, 0,NULL,0);
  configQEI_1();
  configQEI_2();


  timerAlarmEnable(timer);
}
void sendTelData(){
  Serial.begin(115200);
  Serial.println("COMIENZO TELEMETRIA");
  int i;
  for(i=0;i<countData;i++){
    Serial.print(dataTime[i]);
    Serial.print(" ");
    Serial.print(data1[i]);
    Serial.print(" ");
    Serial.print(data2[i]);
    Serial.print(" ");
    Serial.print(data3[i]);
    Serial.print(" ");
    Serial.print(data4[i]);
    Serial.print(" ");
    Serial.print(data5[i]);
    Serial.print(" ");
    Serial.print(data6[i]);
    Serial.print(" ");
    Serial.print(data7[i]);
    Serial.print(" ");
    // Serial.print(data8[i]);
    // Serial.print(" ");
    //Serial.print(data9[i]);
    //Serial.print(" ");
    Serial.println(data8[i]);
  }
  delay(5000);
}


void printIRSensors(){
      readIRsensors();
      Serial.print(RFSensor);Serial.print(" ");
      Serial.print(DRSensor);Serial.print(" ");
      Serial.print(SRSensor);Serial.print(" ");
      Serial.print(SLSensor);Serial.print(" ");
      Serial.print(DLSensor);Serial.print(" ");
      Serial.print(LFSensor);Serial.print(" ");
      getSensorEror();
      Serial.print(aSpeed);
      Serial.println("");
}



int getParedes(int robotDir, int w){
      if(robotDir==RIGHT_DIR){ //si mira a la derecha roto los bits a la izq 1 posicion
            if(w & LEFT_DIR)
                  w = w << 1 | UP_DIR;
            else
                  w = w << 1;
       }else if(robotDir==LEFT_DIR){ //si mira a la izq roto los bits a la der 1 posicion
            if(w & UP_DIR)
                  w = w >> 1 | LEFT_DIR;
            else
                  w = w >> 1;
      }else if(robotDir==DOWN_DIR){ //si mira a abajo roto los bits "a la der" 2 posiciones
            if(w & UP_DIR)
                  w = w >> 1 | LEFT_DIR;
            else
                  w = w >> 1;
            if(w & UP_DIR)
                  w = w >> 1 | LEFT_DIR;
            else
                  w = w >> 1;
      }
      return w & 0x0F;
}

int readParedes(int robotDir){
      int w=0;
      if(LFSensor > 400 && RFSensor > 400){
            w |= UP_DIR;
      }
      if(SLSensor > 650){
            w |= LEFT_DIR;
      }
      if(SRSensor > 650){
            w |= RIGHT_DIR;
      }

      return getParedes(robotDir,  w);
}

short isExploring=1;

void loop() {
  // put your main code here, to run repeatedly:
//delay(1000);

// while(1){
//       Serial.begin(115200);
//       printIRSensors();
//       delay(100);
// }

while(1){
      FFloop();
      //aligmentFrontWall();
      //countData=0;
      //moveOneCell(0);
      //moveOneCell(0);
      //moveOneCell(0);
      //delay(500);
      // countData=0;
       onlyUseGyroFeedback = 1;
       L90();
      // delay(500);
      // moveOneCell(0);
      // delay(500);
      // R90();
      // moveOneCell(0);
      delay(100);
           while(1){
            timerAlarmDisable(timer);
            resetSpeedProfile();
            delay(300);
            sendTelData();
      }

}
//  Serial.begin(115200);
// while(1){

//       Serial.print(readParedes(DOWN_DIR));
//       delay(1000);
// }

while(1){

      //Serial.begin(115200);

      //FFloop();
      L180f();delay(1000);
      R90f();delay(1000);
      //L180f();delay(1000);
      //L90f();delay(1000);

      //printIRSensors();
      while(1){
            timerAlarmDisable(timer);
            resetSpeedProfile();
            delay(300);
            sendTelData();
      }
}

// countData=0;
// while(1){
//       Serial.begin(115200);
//       printIRSensors();
//       delay(100);
// }

//onlyUseGyroFeedback=1;
// for (int k=0;k<5;k++){
      //targetSpeedX = speed_to_counts(0.7*2);
      //delay(300);
      //targetSpeedX = speed_to_counts(0);
      //delay(100);
 //     targetSpeedW = speed_to_counts(0.0*2); //0.5m/s
  //Serial.print(count_enc_l);Serial.print(" ");Serial.println(count_enc_r);

//  moveOneCell();
//  getParedes(RIGHT_DIR);
countData=0;
//onlyUseEncoderFeedback=1;
 while(1){


      moveOneCell(isExploring);
      //moveOneCell();
      R90();
      moveSpeed=0;
      moveOneCell(isExploring);
 delay(50000);
 }


  //moveOneCell();
  // giro90l();

  // delay(200);
  // targetSpeedW = speed_to_counts(0);
  // moveOneCell();
  // targetSpeedW = speed_to_counts(0);
  // delay(200);
  /*long m  = millis();
  while(m+2000 > millis()){
    long ini = micros();
    speedProfile();
    elapseMicros( ini,  1000);
  }*/

 // targetSpeedX= speed_to_counts(0);
 // targetSpeedW= speed_to_counts(0);
  /*m  = millis();
  while(m+2000 > millis()){
    long ini = micros();
    speedProfile();
    elapseMicros( ini,  1000);
  }*/



delay(500);
//}
  while(1){
      timerAlarmDisable(timer);
      resetSpeedProfile();
      delay(100);

      sendTelData();

  }


}



void loopPrintGyro(){
      Serial.begin(115200);
      delay(1000);
      angle=0;
      while(1){
            long t=micros();
            readGyro();
            t = micros() - t;
            Serial.print(aSpeed);Serial.print(" ");
            Serial.print(angle);Serial.print(" ");
            Serial.println(t);
            delay(1);
      }
}

void loopIdaYVuelta(){
      for(int k=0;k<4;k++){
            moveOneCell(isExploring);
            delay(1000);
            giro180();
            delay(1000);
      }
}

void doMovement(int nextMovement){
      countData=0;
       if(nextMovement==TURN_RIGHT){
            delay(100);
            R90();
            delay(100);
      }
       if(nextMovement==TURN_LEFT){
            delay(100);
            L90();
            delay(100);
      }
      if(nextMovement==BACK){
            delay(100);
            R180();
            delay(100);
      }

      moveOneCell(isExploring);

}


int robotDir = DOWN_DIR;
int xRobot = 0;
int yRobot = 15;

SemaphoreHandle_t barrierSemaphore = xSemaphoreCreateCounting( 1, 0 );
int runningFF = 0;

void FFtask(void *a){
      runningFF=1;
      startBuzzer(440);
      int w = readParedes(robotDir);
      updateCurrentWalls(xRobot, yRobot, w);//walls[xRobot][yRobot] |=w;
      FFUpdateDistances(xRobot,yRobot);
      xSemaphoreGive(barrierSemaphore);
      runningFF = 0;
      delay(100);
      stopBuzzer();

      vTaskDelete(NULL);
}

void FFloop(){
      robotDir = DOWN_DIR;
      xRobot = 0;
      yRobot = 15;
      countData = 0;

      xTaskCreatePinnedToCore(FFtask, "FF",10000,NULL, 0,NULL,0);
      while(distances[xRobot][yRobot]>0){

            xSemaphoreTake(barrierSemaphore, portMAX_DELAY);
            short nextMovement = getNextMovement(xRobot,yRobot,robotDir);
            digitalWrite(LED_F,HIGH);
            robotDir = updateDir(robotDir,nextMovement);
            yRobot += yIncrementByDir(robotDir);
            xRobot += xIncrementByDir(robotDir);
            doMovement(nextMovement);
            digitalWrite(LED_F,LOW);

                  if(readButton()){
                        Serial.begin(115200);
                        while(1){
                              timerAlarmDisable(timer);
                              resetSpeedProfile();
                              printMaze(xRobot,yRobot,robotDir);
                              delay(100);
                              sendTelData();
                              delay(2000);
                        }
                  }
      }

      EEPROM_save_maze();

      for(int u=0;u<3;u++){

            digitalWrite(LED_F,HIGH);
            playNote(DO, 200);
            playNote(MI, 200);
            playNote(SOL, 200);
            playNote(DO*2, 400);
            digitalWrite(LED_F,LOW);
            delay(1000);
      }


      initializeDistancesIniGoal();
      xTaskCreatePinnedToCore(FFtask, "FF",10000,NULL, 0,NULL,0);
      while(distances[xRobot][yRobot]>0){
            xSemaphoreTake(barrierSemaphore, portMAX_DELAY);
            short nextMovement = getNextMovement(xRobot,yRobot,robotDir);
            digitalWrite(LED_F,HIGH);
            robotDir = updateDir(robotDir,nextMovement);
            yRobot += yIncrementByDir(robotDir);
            xRobot += xIncrementByDir(robotDir);
            doMovement(nextMovement);
            digitalWrite(LED_F,LOW);

                  if(readButton()){
                        Serial.begin(115200);
                        while(1){
                              timerAlarmDisable(timer);
                              resetSpeedProfile();
                              printMaze(xRobot,yRobot,robotDir);
                              delay(100);
                              sendTelData();
                              delay(2000);
                        }
                  }
      }



      //Serial.println("COME BACK:");
      //initializeDistancesIniGoal();
      //



      EEPROM_save_maze();
      R180();
      robotDir = updateDir(robotDir,BACK);
      for(int u=0;u<1;u++){

            digitalWrite(LED_F,HIGH);
            playNote(DO, 200);
            playNote(MI, 200);
            playNote(SOL, 200);
            playNote(DO*2, 400);
            digitalWrite(LED_F,LOW);
            delay(1000);
      }

      Serial.begin(115200);
                        while(1){
                              timerAlarmDisable(timer);
                              resetSpeedProfile();
                              printMaze(xRobot,yRobot,robotDir);
                              delay(2000);
                        }
}
