#include <Servo.h>
#include "Adafruit_MCP3008.h"
#include "Adafruit_TCS34725.h"
#include "SSD1306Wire.h"

Adafruit_MCP3008 Lightsensor_ADC1;
Adafruit_MCP3008 Lightsensor_ADC2;
Adafruit_MCP3008 Lightsensor_ADC3;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
SSD1306Wire OLED(0x3c, 4, 5);


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
#define _servo1 14
#define _servo2 15
#define _servo3 28
#define _servo4 29



#define motor1A  0
#define motor1B  1
#define motor2A  2
#define motor2B  3


int _sensorPins[20];
int _NumofSensor = 8;
int _min_sensor_values[20];
int _max_sensor_values[20];
int _lastPosition = 0;
int _Sensitive  = 20;
int stateOfRunPID = 0;
float  errors = 0, output = 0, integral = 0, derivative = 0, previous_error = 0;
uint8_t FrontLineColor = 0;
uint8_t BackLineColor = 0;

int _sensorPins_B[20];
int _NumofSensor_B = 8;
int _min_sensor_values_B[20];
int _max_sensor_values_B[20];
int _lastPosition_B = 0;
int _Sensitive_B  = 20;
int stateOfRunPID_B = 0;
float  errors_B = 0, output_B = 0, integral_B = 0, derivative_B = 0, previous_error_B = 0;

int _sensorPins_C[20] = {0,1,2,3,4,5,6,7};
int _NumofSensor_C = 8;
int _min_sensor_values_C[20];
int _max_sensor_values_C[20];


void TitanBot_setup() {
  analogWriteResolution(10);
  analogWriteRange(1023);
  Lightsensor_ADC1.begin(10, 11, 12, 6);
  Lightsensor_ADC2.begin(10, 11, 12, 13);
  Lightsensor_ADC3.begin(10, 11, 12, 8);
  pinMode(9, INPUT_PULLUP);
  OLED.init();
  OLED.flipScreenVertically();
  OLED.setFont(ArialMT_Plain_10);


}
int ADC_Read(int analog_CH) {

   return Lightsensor_ADC1.readADC(analog_CH);
}
int ADC_Read_B(int analog_CH) {

   return Lightsensor_ADC2.readADC(analog_CH);
}
int ADC_Read_C(int analog_CH) {



  
   if(analog_CH >= 4 ){analog_CH = analog_CH - 4;}
    else if (analog_CH < 4 and analog_CH >=0 )  {analog_CH = analog_CH + 4;}

  return Lightsensor_ADC3.readADC(analog_CH);
}
int IN(int _pins) {
  if (_pins == 1) {_pins = 25;}
  else if(_pins >=2 && _pins <=4){_pins = 25 +_pins;}
  else{return 0;}
  pinMode(_pins, INPUT); 
  return digitalRead(_pins);
}
void OUT(int _pins,uint8_t _Status){
  if (_pins == 1) {_pins = 25;}
  else if(_pins >=2 && _pins <=4){_pins = 25 +_pins;}
  pinMode(_pins, OUTPUT); 
  digitalWrite(_pins,_Status);
}
void buzzer(int freq, int timr_delay) {
  pinMode(7, OUTPUT);
  tone(7, freq);
  delay(timr_delay);
  tone(7, 0);
}
// void printText(uint8_t x,uint8_t y,String text,uint8_t size,uint16_t  color){
// 	tft_.setCursor(x, y);
// 	tft_.setTextSize(size);
//   tft_.setTextColor(color);
//   tft_.setTextWrap(true);
//   tft_.println(text);
// }
// void printText(uint8_t x,uint8_t y,String text,uint8_t size,uint16_t  color1,uint16_t  color2){
//   tft_.setCursor(x, y);
//   tft_.setTextSize(size);
//   tft_.setTextColor(color1,color2);
//   tft_.setTextWrap(true);
//   tft_.println(text);
// }
void Read_light_sensor(){
       OLED.drawString(0,0,String(String("F0::")));
       OLED.drawString(28,0,String(ADC_Read(0)));
       OLED.drawString(65,0,String(String("F1::")));
       OLED.drawString(93,0,String(ADC_Read(1)));
       OLED.drawString(0,16,String(String("F2::")));
       OLED.drawString(28,16,String(ADC_Read(2)));
       OLED.drawString(65,16,String(String("F3::")));
       OLED.drawString(93,16,String(ADC_Read(3)));
       OLED.drawString(0,32,String(String("F4::")));
       OLED.drawString(28,32,String(ADC_Read(4)));
       OLED.drawString(65,32,String(String("F5::")));
       OLED.drawString(93,32,String(ADC_Read(5)));
       OLED.drawString(0,45,String(String("F6::")));
       OLED.drawString(28,45,String(ADC_Read(6)));
       OLED.drawString(65,45,String(String("F7::")));
       OLED.drawString(93,45,String(ADC_Read(7)));
}
void Read_light_sensor_B(){
       OLED.drawString(0,0,String(String("B0::")));
       OLED.drawString(28,0,String(ADC_Read_B(0)));
       OLED.drawString(65,0,String(String("B1::")));
       OLED.drawString(93,0,String(ADC_Read_B(1)));
       OLED.drawString(0,16,String(String("B2::")));
       OLED.drawString(28,16,String(ADC_Read_B(2)));
       OLED.drawString(65,16,String(String("B3::")));
       OLED.drawString(93,16,String(ADC_Read_B(3)));
       OLED.drawString(0,32,String(String("B4::")));
       OLED.drawString(28,32,String(ADC_Read_B(4)));
       OLED.drawString(65,32,String(String("B5::")));
       OLED.drawString(93,32,String(ADC_Read_B(5)));
       OLED.drawString(0,45,String(String("B6::")));
       OLED.drawString(28,45,String(ADC_Read_B(6)));
       OLED.drawString(65,45,String(String("B7::")));
       OLED.drawString(93,45,String(ADC_Read_B(7)));
}
void Read_light_sensor_C(){
       OLED.drawString(0,0,String(String("C0::")));
       OLED.drawString(28,0,String(ADC_Read_C(0)));
       OLED.drawString(65,0,String(String("C1::")));
       OLED.drawString(93,0,String(ADC_Read_C(1)));
       OLED.drawString(0,16,String(String("C2::")));
       OLED.drawString(28,16,String(ADC_Read_C(2)));
       OLED.drawString(65,16,String(String("C3::")));
       OLED.drawString(93,16,String(ADC_Read_C(3)));
       OLED.drawString(0,32,String(String("C4::")));
       OLED.drawString(28,32,String(ADC_Read_C(4)));
       OLED.drawString(65,32,String(String("C5::")));
       OLED.drawString(93,32,String(ADC_Read_C(5)));
       OLED.drawString(0,45,String(String("C6::")));
       OLED.drawString(28,45,String(ADC_Read_C(6)));
       OLED.drawString(65,45,String(String("C7::")));
       OLED.drawString(93,45,String(ADC_Read_C(7)));
}
void wait_SW1() {
  int state_sensor = 0;
  pinMode(9,INPUT_PULLUP);
   OLED.clear();
   OLED.setFont(ArialMT_Plain_16);
   OLED.drawString(35,0,"TitanBot");
   OLED.drawString(34,20,"Welcome");
   OLED.display();
  delay(700);
   OLED.setFont(ArialMT_Plain_10);
  while(digitalRead(9) == 1){
    while(BOOTSEL){
      
        state_sensor += 1;
        if(state_sensor > 2)state_sensor = 0;
        buzzer(500,100);
      while(BOOTSEL){}
    }

      OLED.clear();
    if(state_sensor == 0){
     Read_light_sensor(); 
    }
    else if(state_sensor == 1){
     Read_light_sensor_B(); 
    }
    else if(state_sensor == 2){
     Read_light_sensor_C(); 
    }
     
      OLED.display();
     delay(50);

  }
  buzzer(500,100);
   OLED.clear();
   OLED.display();
  
}
void wait_boot_button() {
  int state_waitSW1 = 0;
  OLED.clear();
   OLED.setFont(ArialMT_Plain_16);
   OLED.drawString(35,0,"TitanBot");
   OLED.drawString(0,20,"wait BOOT button ");
   OLED.display();
  delay(700);
  do
  {
    
    
    
    delay(50);
  } while (BOOTSEL == 0);
  
  buzzer(500,100);
  OLED.clear();
   OLED.display();
}

void motor(int pin, int speed_Motor) {
  if (speed_Motor > 100)speed_Motor = 100;
  if (speed_Motor < -100)speed_Motor = -100;
  if (pin == 1) {
    if (speed_Motor < 0) {
      // Serial.println(speed_Motor);
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor1B, 1023);
      analogWrite(motor1A, 1023-abs(speed_Motor));

    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor1A, 1023);
      analogWrite(motor1B, 1023- abs(speed_Motor));
    }
  }
  else if (pin == 2) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor2B, 1023);
      analogWrite(motor2A, 1023-abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor2A, 1023);
      analogWrite(motor2B, 1023-abs(speed_Motor));
    }
  }
}

void ao(){
	analogWrite(motor1B, 1023);
	analogWrite(motor1A, 1023);
	analogWrite(motor2B, 1023);
	analogWrite(motor2A, 1023);
}
void aoS(int speed_break){
  speed_break = constrain(speed_break, 0, 100);
  speed_break = speed_break * 10.23;
  analogWrite(motor1B, speed_break);
  analogWrite(motor1A, speed_break);
  analogWrite(motor2B, speed_break);
  analogWrite(motor2A, speed_break);
}
void motorStop(int motor_ch){
  	if(motor_ch == 0){
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);

	}
	else if(motor_ch == 1 ){
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	}
	else if(motor_ch == 2 ){
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);
	}
	
	else{
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);

	}
}
void motorBreak(){
    analogWrite(motor1B, 1023);
    analogWrite(motor1A, 1023);
    analogWrite(motor2B, 1023);
    analogWrite(motor2A, 1023);

}
void motorBreak(int motor_ch){
  
	if(motor_ch == 0){
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);

	}
	else if(motor_ch == 1 ){
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	}
	else if(motor_ch == 2 ){
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	}
	
	else{
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	}
}
void fd(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,speed_Motor);

}
void fd2(int speed_MotorA,int speed_MotorB){
  motor(1,speed_MotorA);
  motor(2,speed_MotorB);
}
void bk(int speed_Motor){
	motor(1,-speed_Motor);
	motor(2,-speed_Motor);

}
void bk2(int speed_MotorA,int speed_MotorB){
  motor(1,-speed_MotorA);
  motor(2,-speed_MotorB);
}
void tl(int speed_Motor){
	motor(1,0);
	motor(2,speed_Motor);
}
void tr(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,0);
}
void sl(int speed_Motor){
	motor(1,-speed_Motor);
	motor(2,speed_Motor);
}
void sr(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,-speed_Motor);
}

void servoRun(uint8_t servo_ch, int16_t angle) {
  if(angle == 0 ){angle = 1;}

  if (servo_ch == 1)
  {
    if(angle == -1){servo1.detach();}
    servo1.attach(_servo1,300,2500,angle);
  }
  if (servo_ch == 2)
  {
    if(angle == -1){servo2.detach();}
    servo2.attach(_servo2,300,2500,angle);
  }
  if (servo_ch == 3)
  {
    if(angle == -1){servo3.detach();}
    servo3.attach(_servo3,300,2500,angle);
  }
  if (servo_ch == 4)
  {
    if(angle == -1){servo4.detach();}
    servo4.attach(_servo4,300,2500,angle);
  }
}
int ultrasonic(uint8_t Echo_pin , uint8_t Trig_pin) {
  int ECHO = Echo_pin;
  int TRIG = Trig_pin;
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  long duration = 0;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration += pulseIn(ECHO, HIGH);

  // Calculating the distance
  return (duration) * 0.034 / 2;
}



//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************

void setSensorPins(const int * _pins, int _NumofSensor_)
{
  _NumofSensor = _NumofSensor_;
  // _sensorPins = (int *)realloc(_sensorPins, sizeof(int) * _NumofSensor_);
  // _min_sensor_values = (int *)realloc(_min_sensor_values, sizeof(int) * _NumofSensor_);
  // _max_sensor_values = (int *)realloc(_max_sensor_values, sizeof(int) * _NumofSensor_);
  for (uint8_t i = 0; i < _NumofSensor_; i++)
  {
    _sensorPins[i] = _pins[i];
    _min_sensor_values[i] = 1023;
    _max_sensor_values[i] = 0;
  }

}
void setSensorMin(const int * _MinSensor)
{
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _min_sensor_values[i] = _MinSensor[i];
  }
}
void setSensorMax(const int * _MaxSensor)
{
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _max_sensor_values[i] = _MaxSensor[i];
  }
}

void PID_set_Min(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  //setSensorMin((const int[]) {S0, S1, S2, S3, S4, S5, S6, S7});
  _min_sensor_values[0] = S0;
  _min_sensor_values[1] = S1;
  _min_sensor_values[2] = S2;
  _min_sensor_values[3] = S3;
  _min_sensor_values[4] = S4;
  _min_sensor_values[5] = S5;
  _min_sensor_values[6] = S6;
  _min_sensor_values[7] = S7;
}
void PID_set_Max(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  _max_sensor_values[0] = S0;
  _max_sensor_values[1] = S1;
  _max_sensor_values[2] = S2;
  _max_sensor_values[3] = S3;
  _max_sensor_values[4] = S4;
  _max_sensor_values[5] = S5;
  _max_sensor_values[6] = S6;
  _max_sensor_values[7] = S7;
}

void setSensitive(const uint16_t  _SensorSensitive)
{
  _Sensitive = _SensorSensitive;
}
void setFrontLineColor(const uint16_t  _setFrontLineColor)     // if Value = 1 is BlackLine ,value = 0 is WhiteLine
{
  FrontLineColor = _setFrontLineColor;
}
int refSensor(int ch){
  return ( _max_sensor_values[ch] + _min_sensor_values[ch] ) / 2 ;
}

int ReadSensorMinValue(uint8_t _Pin) {
  return _min_sensor_values[_Pin];
}
int ReadSensorMaxValue(uint8_t _Pin) {
  return _max_sensor_values[_Pin];
}
int ReadLightSensor(int analog_CH) {
  int value = 0;

  if(FrontLineColor == 0)value= map(ADC_Read(_sensorPins[analog_CH]), _min_sensor_values[analog_CH], _max_sensor_values[analog_CH], 100, 0);
  else if (FrontLineColor == 1) value= map(ADC_Read(_sensorPins[analog_CH]), _min_sensor_values[analog_CH], _max_sensor_values[analog_CH], 0, 100);
  if(value < 0)value = 0;
  else if(value >100)value = 100;
  return value;
}
// void showGraph(){
//   int state_waitSW1 = 0;
//   pinMode(6, INPUT_PULLUP);
//   tft_.fillScreen(ST77XX_BLACK);

//   do {
//     //tft_.fillRect(20,0,140,128,TFT_BLACK);
//     for(int i = 0;i<_NumofSensor;i++){
//       tft_.setTextColor(TFT_WHITE, TFT_BLACK);
//       tft_.setTextSize(1);
//       tft_.drawString("A"+String(i)+"=",0,10*i);
//       tft_.fillRect(20,10*i,100,5,TFT_BLACK);
//       tft_.fillRect(20,10*i,ReadLightSensor(i),5,TFT_ORANGE);
//       tft_.drawString(String(ReadLightSensor(i))+"  ",130,10*i);
//     }
//     tft_.setTextSize(2);

//     static unsigned long lastTimeUpdateBackground = 0;
//     static bool flagBackground = false;

//     if(millis()-lastTimeUpdateBackground >= 100){
//       lastTimeUpdateBackground = millis();
//       flagBackground =! flagBackground;
//       tft_.setTextColor(flagBackground?TFT_RED:TFT_GREEN, flagBackground?TFT_BLUE:TFT_YELLOW);
//       tft_.drawString("  SW1 Press  ",0,115);
//     }
//     delay(50);
//   } while (digitalRead(6) == 1);
//   buzzer(500,100);
//  }
void setCalibrate(int cal_round) {
   
  for (int round_count = 0; round_count < cal_round; round_count ++ ) {

    for (uint8_t i = 0; i < _NumofSensor; i++)
    {
      if (ADC_Read(_sensorPins[i]) > _max_sensor_values[i] || _max_sensor_values[i] > 1023 ) {
        _max_sensor_values[i]  = ADC_Read(_sensorPins[i]);
        if (_max_sensor_values[i] > 1023 )_max_sensor_values[i] = 1023;
      }
    }
    for (uint8_t i = 0; i < _NumofSensor; i++)
    {
      if (ADC_Read(_sensorPins[i]) < _min_sensor_values[i] || _min_sensor_values[i] == 0) {
        _min_sensor_values[i] = ADC_Read(_sensorPins[i]);
        if (_min_sensor_values[i] < 0) _min_sensor_values[i] = 0;
      }
    }
  }
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _max_sensor_values[i] =  _max_sensor_values[i];
    _min_sensor_values[i] = _min_sensor_values[i];
  }


}



int readline()
{
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    long value = ReadLightSensor(i);
    // long value =  0 ;
    // if( FrontLineColor == 0)value = map(ADC(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 1000, 0);
    // else value = map(ADC(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 0, 1000);
    // if(value < 0)value = 0;
    if (value > _Sensitive) {
      onLine = true;
    }
    if (value > 5)
    {
      avg += (long)value * (i * 100)+50;
      sum += value;
    }
  }
  if (!onLine)
  {
    if (_lastPosition < (_NumofSensor - 1) * 100 / 2)
    {
      return 0;
    }
    else
    {
      return (_NumofSensor - 1) * 100;
    }
  }
  _lastPosition = avg / sum;
  return _lastPosition;
}

void lineFollow_PID(int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD) {

  int speed_PID = RUN_PID_speed;
  int present_position = readline();
  int setpoint = ((_NumofSensor - 1) * 100) / 2;
  errors = present_position - setpoint;
  if (errors == 0) integral = 0;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KI * integral + RUN_PID_KD * derivative;
  //int max_output = RUN_PID_speed;
  // if (output > max_output)output = max_output;
  // else if (output < -max_output)output = -max_output;

  int motorL = constrain(RUN_PID_speed + output, -RUN_PID_speed, RUN_PID_speed);
  int motorR = constrain(RUN_PID_speed - output, -RUN_PID_speed, RUN_PID_speed);
  // if(m1Speed < 0 )m1Speed = 0;
  // if(m2Speed < 0 )m2Speed = 0;

  motor(1,motorL);
  motor(2,motorR);
  previous_error = errors;

}
void run_PID(int RUN_PID_speed , int RUN_PID_Mspeed, float RUN_PID_KP, float RUN_PID_KD) {
  int speed_PID = RUN_PID_speed;
  int present_position = readline();
  int setpoint = ((_NumofSensor - 1) * 100) / 2;
  errors = present_position - setpoint;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    previous_error = errors;
  int max_output = RUN_PID_Mspeed;
  if (output > max_output)output = max_output;
  else if (output < -max_output)output = -max_output;
  int m1Speed = speed_PID + output ;
  int m2Speed = speed_PID - output;
  if(m1Speed < 0 )m1Speed = 0;
  if(m2Speed < 0 )m2Speed = 0;

  motor(1,m1Speed);
  motor(2,m2Speed);
  previous_error = errors;

}
void Run_PID(int speed_Motor,float KP,float KD){
  lineFollow_PID(speed_Motor,KP,0,KD);
}



void setSensorPins_B(const int * _pins, int _NumofSensor_)
{
  _NumofSensor_B = _NumofSensor_;
  // _sensorPins = (int *)realloc(_sensorPins, sizeof(int) * _NumofSensor_);
  // _min_sensor_values = (int *)realloc(_min_sensor_values, sizeof(int) * _NumofSensor_);
  // _max_sensor_values = (int *)realloc(_max_sensor_values, sizeof(int) * _NumofSensor_);
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _sensorPins_B[i] = _pins[i];
    _min_sensor_values_B[i] = 1023;
    _max_sensor_values_B[i] = 0;
  }

}
void setSensorMin_B(const int * _MinSensor)
{
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _min_sensor_values_B[i] = _MinSensor[i];
  }
}
void setSensorMax_B(const int * _MaxSensor)
{
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _max_sensor_values_B[i] = _MaxSensor[i];
  }
}
void setBackLineColor(const uint16_t  setBackLineColor)     // if Value = 1 is BlackLine ,value = 0 is WhiteLine
{
  BackLineColor = setBackLineColor;
}
int refSensor_B(int ch){
  return ( _max_sensor_values_B[ch] + _min_sensor_values_B[ch] ) / 2 ;
}
void PID_set_Min_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  //setSensorMin((const int[]) {S0, S1, S2, S3, S4, S5, S6, S7});
  _min_sensor_values_B[0] = S0;
  _min_sensor_values_B[1] = S1;
  _min_sensor_values_B[2] = S2;
  _min_sensor_values_B[3] = S3;
  _min_sensor_values_B[4] = S4;
  _min_sensor_values_B[5] = S5;
  _min_sensor_values_B[6] = S6;
  _min_sensor_values_B[7] = S7;
}
void PID_set_Max_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  _max_sensor_values_B[0] = S0;
  _max_sensor_values_B[1] = S1;
  _max_sensor_values_B[2] = S2;
  _max_sensor_values_B[3] = S3;
  _max_sensor_values_B[4] = S4;
  _max_sensor_values_B[5] = S5;
  _max_sensor_values_B[6] = S6;
  _max_sensor_values_B[7] = S7;
}

int ReadSensorMinValue_B(uint8_t _Pin) {
  return _min_sensor_values_B[_Pin];
}
int ReadSensorMaxValue_B(uint8_t _Pin) {
  return _max_sensor_values_B[_Pin];
}

int ReadLightSensor_B(int analog_CH) {
  int value = 0;

  if(BackLineColor == 0)value= map(ADC_Read_B(_sensorPins_B[analog_CH]), _min_sensor_values_B[analog_CH], _max_sensor_values_B[analog_CH], 100, 0);
  else if (BackLineColor == 1) value= map(ADC_Read_B(_sensorPins_B[analog_CH]), _min_sensor_values_B[analog_CH], _max_sensor_values_B[analog_CH], 0, 100);
  if(value < 0)value = 0;
  else if(value >100)value = 100;
  return value;
}
int readline_B()
{
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    long value =  ReadLightSensor_B(i);
    if (value > _Sensitive_B) {
      onLine = true;
    }
    if (value > 5)
    {
      avg += (long)value * (i * 100)+50;
      sum += value;
    }
  }
  if (!onLine)
  {
    if (_lastPosition_B < (_NumofSensor_B - 1) * 100 / 2)
    {
      return 0;
    }
    else
    {
      return (_NumofSensor_B - 1) * 100;
    }
  }
  _lastPosition_B = avg / sum;
  return _lastPosition_B;
}
void run_PID_B(int RUN_PID_speed , int RUN_PID_Mspeed, float RUN_PID_KP, float RUN_PID_KD) {
  int speed_PID = RUN_PID_speed;
  int present_position = readline_B();
  int setpoint = ((_NumofSensor_B - 1) * 100) / 2;
  errors = present_position - setpoint;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    previous_error = errors;
  int max_output = RUN_PID_Mspeed;
  if (output > max_output)output = max_output;
  else if (output < -max_output)output = -max_output;
  int m1Speed = speed_PID - output ;
  int m2Speed = speed_PID + output;
  if(m1Speed < 0 )m1Speed = 0;
  if(m2Speed < 0 )m2Speed = 0;

  motor(1,-m1Speed);
  motor(2,-m2Speed);
  previous_error_B = errors;

}
void lineFollow_PID_B(int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD) {

  int speed_PID = RUN_PID_speed;
  int present_position = readline_B();
  int setpoint = ((_NumofSensor - 1) * 100) / 2;
  errors = present_position - setpoint;
  if (errors == 0) integral = 0;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KI * integral + RUN_PID_KD * derivative;

  int motorL = constrain(RUN_PID_speed - output, -RUN_PID_speed, RUN_PID_speed);
  int motorR = constrain(RUN_PID_speed + output, -RUN_PID_speed, RUN_PID_speed);


  motor(1,-((motorL)));
  motor(2,-((motorR)));
  previous_error_B = errors;

}
void Run_PID_B(int speed_Motor,float KP,float KD){
  lineFollow_PID_B(speed_Motor,KP,0,KD);
}
void setCalibrate_B(int cal_round) {
      
  for (int round_count = 0; round_count < cal_round; round_count ++ ) {

    for (uint8_t i = 0; i < _NumofSensor_B; i++)
    {
      if (ADC_Read_B(_sensorPins_B[i]) > _max_sensor_values_B[i] || _max_sensor_values_B[i] > 1023 ) {
        _max_sensor_values_B[i]  = ADC_Read_B(_sensorPins_B[i]);
        if (_max_sensor_values_B[i] > 1023 )_max_sensor_values_B[i] = 1023;
      }
    }
    for (uint8_t i = 0; i < _NumofSensor_B; i++)
    {
      if (ADC_Read_B(_sensorPins_B[i]) < _min_sensor_values_B[i] || _min_sensor_values_B[i] == 0) {
        _min_sensor_values_B[i] = ADC_Read_B(_sensorPins_B[i]);
        if (_min_sensor_values_B[i] < 0) _min_sensor_values_B[i] = 0;
      }
    }
  }
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _max_sensor_values_B[i] =  _max_sensor_values_B[i];
    _min_sensor_values_B[i] = _min_sensor_values_B[i];
  }
}



void PID_set_Min_C(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  //setSensorMin((const int[]) {S0, S1, S2, S3, S4, S5, S6, S7});
  _min_sensor_values_C[0] = S0;
  _min_sensor_values_C[1] = S1;
  _min_sensor_values_C[2] = S2;
  _min_sensor_values_C[3] = S3;
  _min_sensor_values_C[4] = S4;
  _min_sensor_values_C[5] = S5;
  _min_sensor_values_C[6] = S6;
  _min_sensor_values_C[7] = S7;
}
void PID_set_Max_C(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  _max_sensor_values_C[0] = S0;
  _max_sensor_values_C[1] = S1;
  _max_sensor_values_C[2] = S2;
  _max_sensor_values_C[3] = S3;
  _max_sensor_values_C[4] = S4;
  _max_sensor_values_C[5] = S5;
  _max_sensor_values_C[6] = S6;
  _max_sensor_values_C[7] = S7;
}
int RefSensor_C(int ch){
  return ( _max_sensor_values_C[ch] + _min_sensor_values_C[ch] ) / 2 ;
}


int ReadSensorMinValue_C(uint8_t _Pin) {
  return _min_sensor_values_C[_Pin];
}
int ReadSensorMaxValue_C(uint8_t _Pin) {
  return _max_sensor_values_C[_Pin];
}

void setCalibrate_C(int cal_round) {
      
  for (int round_count = 0; round_count < cal_round; round_count ++ ) {

    for (uint8_t i = 0; i < _NumofSensor_C; i++)
    {
      if (ADC_Read_C(_sensorPins_C[i]) > _max_sensor_values_C[i] || _max_sensor_values_C[i] > 1023 ) {
        _max_sensor_values_C[i]  = ADC_Read_C(_sensorPins_C[i]);
        if (_max_sensor_values_C[i] > 1023 )_max_sensor_values_C[i] = 1023;
      }
    }
    for (uint8_t i = 0; i < _NumofSensor_C; i++)
    {
      if (ADC_Read_C(_sensorPins_C[i]) < _min_sensor_values_C[i] || _min_sensor_values_C[i] == 0) {
        _min_sensor_values_C[i] = ADC_Read_C(_sensorPins_C[i]);
        if (_min_sensor_values_C[i] < 0) _min_sensor_values_C[i] = 0;
      }
    }
  }
  for (uint8_t i = 0; i < _NumofSensor_C; i++)
  {
    _max_sensor_values_C[i] =  _max_sensor_values_C[i];
    _min_sensor_values_C[i] = _min_sensor_values_C[i];
  }
}
//
//
//


bool Read_status_sensor(int pin_sensor){
  return ADC_Read(_sensorPins[pin_sensor]) < ((_max_sensor_values[pin_sensor] + _min_sensor_values[pin_sensor]) / 2) ? true : false;
}



bool Read_status_sensor_B(int pin_sensor){
  return ADC_Read_B(_sensorPins_B[pin_sensor]) < ((_max_sensor_values_B[pin_sensor] + _min_sensor_values_B[pin_sensor]) / 2) ? true : false;
}

bool Read_status_sensor_C(int pin_sensor){
  return ADC_Read_C(_sensorPins_C[pin_sensor]) < ((_max_sensor_values_C[pin_sensor] + _min_sensor_values_C[pin_sensor]) / 2) ? true : false;
}

int refSensor_C(int ch){
  return ( _max_sensor_values_C[ch] + _min_sensor_values_C[ch] ) / 2 ;
}

int Read_sumValue_sensor(){
  int value = 0;
  for(int i = 0;i<_NumofSensor;i++){
    if(FrontLineColor == 0){
        value += map(ADC_Read(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 100, 0);
      }
      else {
        value += map(ADC_Read(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 0, 100);
      } 
  }
   
    return value;
}

int Read_sumValue_sensor_B(){
  int value = 0;
  for(int i = 0;i<_NumofSensor_B;i++){
    if(BackLineColor == 0){
        value += map(ADC_Read_B(_sensorPins_B[i]), _min_sensor_values_B[i], _max_sensor_values_B[i], 100, 0);
      }
      else {
        value += map(ADC_Read_B(_sensorPins_B[i]), _min_sensor_values_B[i], _max_sensor_values_B[i], 0, 100);
      } 
  }
   
    return value;
}


int PID_NumPin = 8;
int PID_NumPin_B = 8;
void PID_set_Pin(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){

}
void PID_set_Pin_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  
}

int Read_Color_TCS(int color_of_sensor)
{
  uint16_t clearcol_lib, red_lib, green_lib, blue_lib;
  float average_lib, r_lib, g_lib, b_lib;
  float data_color = 0.00;
 //delay(100); // Farbmessung dauert c. 50ms 
 tcs.getRawData(&red_lib, &green_lib, &blue_lib, &clearcol_lib);
 average_lib = (red_lib+green_lib+blue_lib)/3;
 r_lib = red_lib/average_lib;
 g_lib = green_lib/average_lib;
 b_lib = blue_lib/average_lib;
 if(color_of_sensor == 0){
  data_color =  r_lib*100;
 }
 else if(color_of_sensor == 1){
  data_color =  g_lib*100;
 }
  else if(color_of_sensor == 2){
  data_color =  b_lib*100;
 }

  return data_color;
}
void setCalibrate_with_moving(int speed_motor,int cal_round,int round) {

  for(int i_round = 0;i_round < round; i_round++)
  {
    fd(speed_motor); 
    for(int i = 0;i<cal_round;i++){
    	setCalibrate(1);
    	setCalibrate_B(1);
    	setCalibrate_C(1);
    }
    bk(speed_motor); 
    for(int i = 0;i<cal_round;i++){
    	setCalibrate(1);
    	setCalibrate_B(1);
    	setCalibrate_C(1);
    }
  } 

  ao();
}
void Run_PID_until_frontSensor(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD,int sumValue_traget){
  do{
    Run_PID(RUN_PID_speed,RUN_PID_KP,RUN_PID_KD);
  }while(Read_sumValue_sensor() < sumValue_traget);
}

void Run_PID_B_until_backSensor(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD,int sumValue_traget){
  do{
    Run_PID_B(RUN_PID_speed,RUN_PID_KP,RUN_PID_KD);
  }while(Read_sumValue_sensor_B() < sumValue_traget);
}
