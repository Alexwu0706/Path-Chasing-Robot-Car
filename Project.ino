//ECE 3 Path Following Car Project 
//Bailun Wu

#include <ECE3.h>
#include <stdio.h>
#define P5_0 13 
#define P5_2 12
#define LED 76

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pin Assignments
const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;
const int right_nslp_pin = 11;  
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// Variable
int flag1 = 1;
int flag2 = 0;
int left_spd;
int right_spd;
uint16_t sensorValues[8];
float MinimumValue[8] = {643,689,643,597,597,620,667,784};    
float MaximumValue[8] = {1857,1811,1397,1820.2,1872,1880,1833,1716};    
float PastError;
float PresentError;
float endcondition = 0;
int donut = 1;                                //     2  for phanthon crosspiece
int nowstop = 2;                              //     3  for phanthon crosspiece
int donutT = 1050;
int base_spd = 55;
int steering_left_spd;
int steering_right_spd;
int LEDcount = 0;
int LEDmax = 480;
int LEDon = 160;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main
void setup() {
  
  pinMode(left_nslp_pin,OUTPUT);      //nslp is a "wake up" pin for your PWM pin (not sleep)
  pinMode(left_dir_pin,OUTPUT);       //dir is a direction operator to control the backward and forward direction of your car
  pinMode(left_pwm_pin,OUTPUT);       //pwm is the wheel speed
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  pinMode(LED,OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);    //Low = Sleep   High = Wake Up  
  digitalWrite(right_nslp_pin,HIGH);    
  
  ECE3_Init();
  delay(2000);
  ChangeBaseSpeed(0,base_spd);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop
void loop() {
  //General
  ECE3_read_IR(sensorValues);
  PresentError = error(MinimumValue,MaximumValue,sensorValues);
  endcondition = sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7];
  if(flag1 == 1){
    PastError = PresentError;
    flag1++;
  }
  if(endcondition >= 16000){
    flag2++;
    if(flag2 > nowstop){
      flag2 = nowstop;
    }
  }
  
  //Steering Command
  if(flag2 == nowstop){
    stopcar();
    digitalWrite(LED,LOW);
  }else{
    Extra_LED();
    steering_left_spd = base_spd - newspeed(PresentError,PastError);
    steering_right_spd = base_spd + newspeed(PresentError,PastError);
    analogWrite(left_pwm_pin,steering_left_spd);
    analogWrite(right_pwm_pin,steering_right_spd);
  }
  
  //180 Rotation
  if(endcondition >= 16000 && flag2 == donut){
    stopcar();
    digitalWrite(left_dir_pin,LOW);
    digitalWrite(right_dir_pin,HIGH);
    ChangeBaseSpeed(0,base_spd);
    delay(donutT);                                     //750 (70)    1000 (55)
    if(flag2 == donut){
      digitalWrite(left_dir_pin,LOW);
      digitalWrite(right_dir_pin,LOW);
      ChangeBaseSpeed(0,base_spd);
    }
  }
  
  PastError = PresentError;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function
int newspeed(float y, float z){
  float Kp = 0.1;                                       //0.1
  float Kd = 0.795;                                     //0.795     (55) 
  float P = Kp*y;
  float D = Kd*(y-z);
  int correction = P+D;
  return correction; 
}

void stopcar()
{
  analogWrite(right_pwm_pin,0);
  analogWrite(left_pwm_pin,0);
}

void Extra_LED()
{
  if(LEDcount == (LEDmax+1)){
    LEDcount = 0;
  }
  
  if(LEDcount <= LEDon){
    digitalWrite(LED,HIGH);
    LEDcount++;
  }else if((LEDcount > LEDon) && (LEDcount <= LEDmax)){
    digitalWrite(LED,LOW);
    LEDcount++;
  }  
}

float error(float Min[8],float Max[8],uint16_t SensorV[8])
{
  float NormalizedV[8];
  for(int i=0; i<8; i++){
    if((SensorV[i]-Min[i]) == 0){
       NormalizedV[i] = 0;
    }
    else{
       NormalizedV[i] = (SensorV[i]-Min[i])*(1000/Max[i]);
    }
  }

  float steeringV = (NormalizedV[0]*(-8)+NormalizedV[1]*(-4)+NormalizedV[2]*(-2)+NormalizedV[3]*(-1)+NormalizedV[4]*(1)+NormalizedV[5]*(2)+NormalizedV[6]*(4)+NormalizedV[7]*(8))/4;
  return steeringV;
}

void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd){
  int numSteps = 5;
  int pwmLeftVal = initialBaseSpd; // initialize left wheel speed 
  int pwmRightVal = initialBaseSpd;  // initialize right wheel speed 
  int deltaLeft = (finalBaseSpd-initialBaseSpd)/numSteps; // left in(de)crement
  int deltaRight = (finalBaseSpd-initialBaseSpd)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(60);   
  }
} 

int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}
