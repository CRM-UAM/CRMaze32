#ifndef _SPEED_CONTROLLER_H
#define _SPEED_CONTROLLER_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define LED_R 13
#define LED_L 15
#define LED_F 2
#define BUZZER_PIN 16

#define O_IR_F_L 4
#define O_IR_F_R 18
#define O_IR_DIA 17
#define O_IR_LAT 19

#define I_IR_F_L 36
#define I_IR_F_R 39
#define I_IR_DIA_L 34
#define I_IR_DIA_R 35
#define I_IR_LAT_L  32
#define I_IR_LAT_R 25

#define LMOTOR_EN 21
#define LMOTOR_CH 6
#define RMOTOR_EN 22
#define RMOTOR_CH 7
#define LMOTOR_DIR 5
#define RMOTOR_DIR 12

#define GYRO 33

#define ENC_1_B 23
#define ENC_1_A 26
#define ENC_2_B 27
#define ENC_2_A 14

#define BUTTON 0

#define DIAMETRO 36.6//17.0 //wheel DIAMETRO in mm
#define TICKS_PER_REV 3510//5000//3540 //ticks resolution of encoder per revolution
#define _PI 3.141592
#define speed_to_counts(v) (TICKS_PER_REV*v/(_PI*DIAMETRO)) // real speed in mm/ms to ticks/ms
#define counts_to_speed(c) ((_PI*DIAMETRO)*c/(TICKS_PER_REV)) // encoder speed in ticks/ms to mm/ms



#define WHEEL_DISTANCE 68
#define ONE_CELL_DISTANCE speed_to_counts(180*2) //18mm to encoder count

#define angSpeed_to_Wcounts(v) (WHEEL_DISTANCE*(v)*_PI*ONE_CELL_DISTANCE/2/180/180/1000)



//PUBLIC var:
extern int targetSpeedX;
extern int targetSpeedW;
extern short onlyUseGyroFeedback;
extern short onlyUseEncoderFeedback;
extern short useEncGyroFeedback;
extern int DLSensor; //Last IR - DL sensor mesure
extern int DRSensor; //Last IR - DR sensor mesure
extern int LFSensor; //Last IR - LF sensor mesure
extern int RFSensor; //Last IR - RF sensor mesure
extern int SRSensor; //Last IR - SR sensor mesure
extern int SLSensor; //Last IR - SL sensor mesure
extern double moveSpeed;
extern int LFvalue2; //to be configured
extern int RFvalue2;

extern int aSpeed;
extern int angle;

extern long oldEncoderCount; //used in move_one_cell
extern long encoderCount;

extern long leftEncoderCount;
extern long rightEncoderCount;

extern double maxSpeed;
extern double moveSpeed;

#define N_TEL 1050
extern int countData;
extern double dataTime[N_TEL];
extern double data1[N_TEL];
extern double data2[N_TEL];
extern double data3[N_TEL];
extern double data4[N_TEL];
extern double data5[N_TEL];
extern double data6[N_TEL];
extern double data7[N_TEL];
extern double data8[N_TEL];
//extern double data9[N_TEL];
//extern double data10[N_TEL];

int readButton();
void allLedsOn();
void allLedsOff();

void readGyro(void);
void readIRsensors();

//speed are in encoder counts/ms, dist is in encoder counts
double needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd);


void aligmentFrontWall();

void moveOneCell(short isExploring);
void giro180();
void giro90r();
void giro90l();

void L90();
void L90f();
void R90();
void R90f();
void R180();
void L180f();

void resetSpeedProfile();

void  getSensorEror();
extern int sensorError;
extern int a_scale;

void speedProfile();
void calculateMotorPwm();
void updateCurrentSpeed();
void getEncoderStatus();

void setLeftPwm(int p);
void setRightPwm(int p);

void configQEI_1();
void configQEI_2();
//inline int16_t read_clear_encoder1();
//inline int16_t read_clear_encoder2();

// void encoder_setup();
// void changeEnc2();
// void changeEnc1();
// void changeEnc22();
// void changeEnc11();
//
//
extern int robotDir;
extern int xRobot;
extern int yRobot;
extern int runningFF;
void FFtask(void *a);
void playNoteNoDelay(int freq, int dur);
void elapseMicros(uint32_t targetTime, uint32_t oldt);

#endif
