#include <math.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
////////////PINS////////////////
// Distance Analog Sensors (Sharp)
#define SD1     (2)   //Sensor depan bagian kiri
#define SD2     (1)   //Sensor depan bagian kanan
#define SD3     (3)   //Sensor kiri
#define SD4     (0)   //Sensor kanan
// NodeMCU
#define NodeMCU_RX 50
#define NodeMCU_TX 52
SoftwareSerial NodeMCU (NodeMCU_RX,NodeMCU_TX);
// MP3
#define ARDUINO_RX 17 //should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 16  //connect to RX of the module
SoftwareSerial mp3(ARDUINO_RX, ARDUINO_TX);
static int8_t Send_buf[8] = {0} ;//The MP3 player undestands orders in a 8 int string
#define VOLUME_UP_ONE         0X1E
#define VOLUME_DOWN_ONE       0X05
#define CMD_PLAY_WITHVOLUME   0X22 //data is needed  0x7E 06 22 00 xx yy EF;(xx volume)(yy number of song)
#define CMD_SET_VOLUME        0X1E //DATA IS REQUIRED (number of volume from 0 up to 30(0x1E))
#define CMD_SEL_DEV           0X09 //SELECT STORAGE DEVICE, DATA IS REQUIRED
#define DEV_TF                0X02 //HELLO,IM THE DATA REQUIRED
#define CMD_PLAY_WITHFOLDER   0X0F //DATA IS NEEDED, 0x7E 06 0F 00 01 02 EF;(play the song with the directory \01\002xxxxxx.mp3
#define STOP_PLAY             0X16
// Battery Voltage input
#define battery     (A4)   //Analog
// IndicatorLED
#define led         (13)
// Motor1 Right
#define motor1Pin1  (6)
#define motor1Pin2  (5)
#define encodPinA1  (3)   // encoder A pin, interrupt pin of Arduino Uno
#define encodPinB1  (7)   // encoder B pin, read motor direction
// Motor2 Left
#define motor2Pin1  (9)
#define motor2Pin2  (10)
#define encodPinA2  (2)   // encoder A pin, interrupt pin of Arduino Uno
#define encodPinB2  (4)   // encoder B pin, read motor direction
// Bumper
#define bumper1     (8)
#define bumper2     (30)
// PWM for the micro metal motors //Values to delete soon since use of PID
#define pwmMax      (225) 
#define pwmMin      (100)  
// PID loop time
#define LOOPTIME    (100)
//////////////////Relay//////////////////
#define relay1      (23)     // Motor Vaccum
#define relay2      (25)     // Motor Pembersih Kanan
#define relay3      (27)     // Motor Pembersih Kiri
int debu = 0;
///////////////Mode//////////////////////
#define changemode  (33)
#define relaymotor  (35)     // Relay ON/OFF motor
///////////////Push Button///////////////
int pbuttonPin1 = 11;
int pbuttonPin2 = 12;
int val1 = 0;
int val0 = 0; 
int lightON1 = 0;
int lightON2 = 0;
int pushed1 = 0;
int pushed2 = 0;
/////////////////Serial/////////////////
char raspberry = 0;
/////////////////Dust///////////////////
  int measurePin    = A14; //Connect dust sensor to Arduino A0 pin
  int ledPower      = 51;   //Connect 3 led driver pins of dust sensor to Arduino D2
  int samplingTime  = 280;
  int deltaTime     = 40;
  int sleepTime     = 9680;
   
  float voMeasured  = 0;
  float calcVoltage = 0;
  float dustDensity = 0;
///////////////Constants////////////////
const float voltageBatCharged= 12.58; // Voltage measured when battery fully charged //Change this
const float batteryLimitDischarge = 11.00; // Safe value to not kill the Battery

// Variables will change:
int bumperState = 0;               // variabel untuk membaca status pushbutton
int counter = 0;                   // Prevents from being stuck
boolean control = true;
boolean printOnce = true;          //For debugging
unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPrint = 0;
unsigned long lastErrorMilli = 0;
//Motor 1
float speed_req1 = 100.0;            // speed (Set Point)
float speed_act1 = 0.0;             // speed (actual value)
int PWM_val1 = 100;                   // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count_1 = 0;          // rev counter
//Motor 2
float speed_req2 = 100.0;
float speed_act2 = 0.0;
int PWM_val2 = 100;
volatile long count_2 = 0;

float Kp =   0.6;// 0.65;            // PID Proportional control Gain   (Good values as well: 0.5)     
float Kd =   0.0;// 0.005;           // PID Derivitave control Gain
//////////////CODE/////////////
void setup() {
  Serial.begin(9600); // Initialize serial
  mp3.begin(9600);
  NodeMCU.begin(115200);
  delay(200);//wait for 200ms
  sendCommand(CMD_SEL_DEV, DEV_TF);//select the TF card 
  delay(200);//wait for 200ms
  //Initialize outputs and inputs
  //Dust Sensor
  pinMode(ledPower,OUTPUT);
  //Motor1
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  //Motor2
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  //LED
  pinMode(led, OUTPUT);
  //Relay
  pinMode (relay1, OUTPUT);
  pinMode (relay2, OUTPUT);
  pinMode (relay3, OUTPUT);
  //Mode
  pinMode (changemode, OUTPUT);
  pinMode (relaymotor, OUTPUT);
  //INPUTS
  //Motor encoders
  pinMode(encodPinA1, INPUT_PULLUP); 
  pinMode(encodPinB1, INPUT_PULLUP);
  pinMode(encodPinA2, INPUT_PULLUP); 
  pinMode(encodPinB2, INPUT_PULLUP); 
  //Bumper
  pinMode(bumper1, INPUT_PULLUP); 
  pinMode(bumper2, INPUT_PULLUP); 
  //Sensor
  pinMode(SD1, INPUT);
  pinMode(SD2, INPUT);
  pinMode(SD3, INPUT);
  pinMode(SD4, INPUT);
  //PushButton
  pinMode (pbuttonPin1, INPUT_PULLUP);
  pinMode (pbuttonPin2,INPUT_PULLUP);
  //Batt
  pinMode(battery, INPUT);
  //Encoder Interrupt executes rencoder_# function when falling edge of the signal
  //Refer to: https://www.arduino.cc/en/Reference/AttachInterrupt
  //Arduino UNO only has 2 external interrupts
  attachInterrupt(0, rencoder_1, FALLING);  //Pin 2 of Arduino Uno
  attachInterrupt(1, rencoder_2, FALLING);  //Pin 3 of Arduino Uno
  digitalWrite (relay1, HIGH);
  digitalWrite (relay2, HIGH);
  digitalWrite (relay3, HIGH);
  digitalWrite (relaymotor, HIGH);
    
  ///////////////////////////////Wait////////////////////////////////////////
  //Wait about 5 s and initialize fan if voltage ok
  waitBlinking(3,1); //5 seconds at 1 Hz
  //Crank (initialize the fan because the voltage drops when cranking)
  if(readBattery(battery)>12.1){
    /*digitalWrite(fanmotor, HIGH);            //Turn the Fan ON
    digitalWrite(washmotor1, HIGH);          //Turn the Washmotor1 ON
    digitalWrite(washmotor2, HIGH);          //Turn the Washmotor2 ON
    delay(500);   */                         //For 1000ms
  }
  else {
    //do nothing Convention
    }
  
} 
/////////////////////////////////////////////////MAIN CODE//////////////////////////////
void loop(){
  //Keep the control of the battery automatically turn the fan off
  //If control = true the battery level is ok, otherwise the battery is low.
  dust();
  Start();
  Mode();
  batteryControl(battery); //modifies the variable control of the battery is low
  setMotors(); //Set pwm of each motor according to the actual speed
  serialRobot();
  controlRobot(); // Execute all conditions to move
  printMotorsInfo();
  
}
//////////Functions To Use //////////
//PORTD, containts Digital pins 0-7, 4 and 7 used to consider the direction of each motor. (PIND is for read only) 
void rencoder_1()  {                          // pulse and direction, direct port reading to save cycles
  if (PIND & 0b00010000)     count_1++;       // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count_1--;        // if (digitalRead(encodPinB1)==LOW)   count --;
}

void rencoder_2()  {                          // pulse and direction, direct port reading to save cycles
  if (PIND & 0b10000000)     count_2++;       // if(digitalRead(encodPinB2)==HIGH)   count ++;
  else                      count_2--;        // if (digitalRead(encodPinB2)==LOW)   count --;
}
void waitBlinking(int n, int frequency){
  //blink for n seconds at frequency hz
  for (int i=1; i <= n; i++){
    for(int j=1; j<=frequency; j++){
      digitalWrite(led, HIGH);   
      delay((1000/frequency)/2);   //Half time on            
      digitalWrite(led, LOW);                                                                                                                                                                                                                                     
      delay((1000/frequency)/2);   //Half time off
    }
   } 
}
double sdSHARP(int Sensor){
  //Returns the distance in cm
  double dist = pow(analogRead(Sensor), -0.857); // x to power of y
  return (dist * 1167.9);
}
  float  readBattery(int input){
  int readInput;
  float voltage;
  readInput = analogRead(input);
  voltage = (((readInput*4.9)/1000)*voltageBatCharged ) / 5; 
  Serial.print("Battery= ");     Serial.println(voltage);
  return voltage;
  } 
void Start(){
  val0 = digitalRead(pbuttonPin1);
  if(val0 == HIGH && lightON1 == LOW){
    pushed1 = 1-pushed1;
    delay(100);
  }    
  lightON1 = val0;
      if(pushed1 == HIGH){
        Serial.println("Go");
        digitalWrite(relaymotor, HIGH);
      }else{
        Serial.println("Stop");
        digitalWrite(relaymotor, LOW);
      }     
  delay(100);
}
void batteryControl(int input){
  //Turn everything off in case the battery is low
  float v_battery;
  v_battery = readBattery(input);
  if(v_battery<=11.6){ //battery limit of discharge, Don't put this limit lower than  11.1V or you can kill the battery
    control = false;
    }
  else {
    //Do nothing Convention
    }
}
void dust(){
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);
  
  voMeasured = analogRead(measurePin); // read the dust value
  
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured * (5.0 / 1024.0);
  
  dustDensity = ((0.17 * calcVoltage - 0.1)*1000.0)-120;
  Serial.println();
  Serial.print("Raw Signal Value (0-1023): ");
  Serial.println(voMeasured);
  
  Serial.print(" - Voltage: ");
  Serial.println(calcVoltage);
  
  Serial.print(" - Dust Density: ");
  Serial.println(dustDensity); // unit: ug/m3
  float d = dustDensity;
  if (isnan(d)){
    return;
  }
  root["dust"] = d;

  if (NodeMCU.available()>0)
  {
    root.printTo(NodeMCU);
  }
}
void moveMotors(int moveTime, int pwmMotor1, int pwmMotor2, char direc){
  //Manipulate direction according the desired movement of the motors
  switch(direc){
    case 'f':
      analogWrite(motor1Pin1, pwmMotor1); 
      analogWrite(motor1Pin2, 0);               //PWM value when 0 = 0% and 255 = 100%
      analogWrite(motor2Pin1, pwmMotor2); 
      analogWrite(motor2Pin2, 0); 
      delay(moveTime);
      break;
    case 'b':
      analogWrite(motor1Pin1, 0); 
      analogWrite(motor1Pin2, pwmMotor1);
      analogWrite(motor2Pin1, 0); 
      analogWrite(motor2Pin2, pwmMotor2); 
      delay(moveTime);
      break;
    case 'l':
      analogWrite(motor1Pin1, 0); 
      analogWrite(motor1Pin2, pwmMotor1); 
      analogWrite(motor2Pin1, pwmMotor2);
      analogWrite(motor2Pin2, 0); 
      delay(moveTime);
      break;
    case 'r':
      analogWrite(motor1Pin1, pwmMotor1); 
      analogWrite(motor1Pin2, 0); 
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, pwmMotor2); 
      delay(moveTime);
      break;
    case 's':
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, 0); 
      analogWrite(motor2Pin1, 0); 
      analogWrite(motor2Pin2, 0); 
      delay(moveTime);
      break;
    default:
      //Do nothing convention
      Serial.println("Default");
      break;
  }
}
void getMotorsSpeed()  {  
  static long countAnt_1 = 0, countAnt_2 = 0, countAnt_3 = 0, countAnt_4 = 0; // last count, static variables preserve the last value
  speed_act1 = ((count_1 - countAnt_1)*(60*(1000/LOOPTIME)))/(3*298);         // 3 pulses X 298 gear ratio = 894 counts per output shaft rev
  countAnt_1 = count_1;
  speed_act2 = ((count_2 - countAnt_2)*(60*(1000/LOOPTIME)))/(3*298);         
  countAnt_2 = count_2;
}
int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
  float pidTerm = 0;                                                          // PID correction
  float error = 0;                                  
  static float last_error = 0;
  unsigned long reachTime = 0;
  error = abs(targetValue) - abs(currentValue); 
  //Serial.print(" Error: ");      Serial.print(error);                
  /*if (error <= 1.0 && printOnce){
    //Measure time to see when the motor reached the Set point
    reachTime = millis()-lastErrorMilli;
    Serial.print("SP reachead: ");  Serial.print(reachTime); Serial.println();
    printOnce = false;
    }
  */
  //PID controller, not using Ki at the moment
  pidTerm = (Kp * error) + (Kd * (error - last_error));                            
  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}
void setMotors(){
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorsSpeed();                                                         // calculate speed, volts and Amps
    //Global values
    PWM_val1 = updatePid(PWM_val1, speed_req1, speed_act1);                   // compute PWM value
    PWM_val2 = updatePid(PWM_val2, speed_req2, speed_act2);                   // compute PWM value
  }
}
void printMotorsInfo()  {                                                     // display data
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    Serial.print("SP_1:");              Serial.print(speed_req1); 
    Serial.print("  RPM_1: ");          Serial.print(speed_act1);
    Serial.print("  PWM_1: ");          Serial.print(PWM_val1);
    Serial.print(", SP_2: ");           Serial.print(speed_req2);
    Serial.print("  RPM_2: ");          Serial.print(speed_act2);
    Serial.print("  PWM_2: ");          Serial.print(PWM_val2);
  }
}
void serialRobot(){
   if (Serial.available() > 0){
    raspberry = Serial.read();
    if (raspberry == '0'){ 
       digitalWrite (relay1, LOW);
       digitalWrite (relay2, LOW);
       digitalWrite (relay3, LOW);
    }
    else if (raspberry == '1'){ 
       digitalWrite (relay1, HIGH);
       digitalWrite (relay2, HIGH);
       digitalWrite (relay3, HIGH);
    } 
   }
   else { 
       digitalWrite (relay1, HIGH);
       digitalWrite (relay2, HIGH);
       digitalWrite (relay3, HIGH);
    }
}
void controlRobot(){
  Serial.print("SD1= ");
  Serial.println(sdSHARP(SD1));
  Serial.print("SD2= ");
  Serial.println(sdSHARP(SD2));
  Serial.print("SD3= ");
  Serial.println(sdSHARP(SD3));              
  Serial.print("SD4= ");
  Serial.println(sdSHARP(SD4));
  //delay(200);*/
  float minDistanceSharp = 7;                 // Distance in cm
  bumperState = digitalRead(bumper1);
  if (control){
    digitalWrite(led, HIGH);
    if (sdSHARP(SD2) < minDistanceSharp){      // If the distance between an object and the left front sensor is less than 4.3 cm or the bumper hits, it will move to the left
      if (counter == 2){                       // prevent of being stuck on corners
        counter = 0;
        }
      else {
        //Do nothing Convention
      }
      moveMotors(100, PWM_val1, PWM_val2, 'f'); // approach a bit
      moveMotors(1500, PWM_val1, PWM_val2, 'b'); // backward delay of 500ms
      moveMotors(2500, PWM_val1, PWM_val2, 'l');
      counter = counter + 2;
      Serial.println("Turn Left");
      }
      else if (sdSHARP(SD1) < minDistanceSharp){ 
      //If the distance between an object and the right front sensor is less than 4.3 cm, it will move to the right
      if (counter == 1){
        counter = 0;
        }
      else{
        //Do nothing Convention
      }
      moveMotors(100, PWM_val1, PWM_val2, 'f'); 
      moveMotors(1500, PWM_val1, PWM_val2, 'b');
      moveMotors(2500, PWM_val1, PWM_val2, 'r');
      counter++;
      Serial.println("Turn Right");
      }
      else if (sdSHARP(SD3) < minDistanceSharp){                                                         
      //If the distance between an object and the right front sensor is less than 4.3 cm, it will move to the right
      if (counter == 3){
        counter = 0;
        }
      else{
        //Do nothing Convention
      }
      moveMotors(100, PWM_val1, PWM_val2, 'f'); 
      moveMotors(1500, PWM_val1, PWM_val2, 'b');
      moveMotors(2500, PWM_val1, PWM_val2, 'r');
      counter = counter + 3;
      Serial.println("Turn Right");
      }
      else if (sdSHARP(SD4) < minDistanceSharp){                                                     
      //If the distance between an object and the right front sensor is less than 4.3 cm, it will move to the right
      if (counter == 4){
        counter = 0;
        }
      else{
        //Do nothing Convention
      }
      moveMotors(100, PWM_val1, PWM_val2, 'f'); // approach a bit
      moveMotors(1500, PWM_val1, PWM_val2, 'b'); // backward delay of 500ms
      moveMotors(2500, PWM_val1, PWM_val2, 'l');
      counter = counter + 4;
      Serial.println("Turn Left");
      }
      else if (bumperState == 0){
      counter = 0;
      moveMotors(500, PWM_val1, PWM_val2, 'b'); 
      moveMotors(300, PWM_val1, PWM_val2, 'l');
      Serial.print("  Turn Left ");
      }
      else {
      if(counter == 5){ //Corner
        moveMotors(1000, PWM_val1, PWM_val2, 'b');
        moveMotors(1000, PWM_val1, PWM_val2, 'l');
        counter = 0;
        }
       else {
        moveMotors(300, PWM_val1, PWM_val2, 'f');   
        Serial.println("Move Forward");
      }
     }
  }
  else if (!control){               //If the battery is low, turn everything off
   /* digitalWrite(fanmotor, LOW);    //Turn the Fan OFF
    digitalWrite(washmotor1, LOW);  //Turn the Washmotor OFF
    digitalWrite(washmotor2, LOW);  //Turn the Washmotor OFF */
    moveMotors(0, 0, 0, 's');
    digitalWrite(relaymotor, HIGH);
    Serial.print("Low Battery! ");
    Serial.println();
    waitBlinking(1,3);               //blink as warning 3hz in a loop
    }
}
void sendCommand(int8_t command, int16_t dat)
{
 delay(20);
 Send_buf[0] = 0x7e;                  //starting byte
 Send_buf[1] = 0xff;                  //version
 Send_buf[2] = 0x06;                  //the number of bytes of the command without starting byte and ending byte
 Send_buf[3] = command;               //
 Send_buf[4] = 0x00;                  //0x00 = no feedback, 0x01 = feedback
 Send_buf[5] = (int8_t)(dat >> 8);    //datah
 Send_buf[6] = (int8_t)(dat);         //datal
 Send_buf[7] = 0xef;                  //ending byte
 for(uint8_t i=0; i<8; i++)           //
 {
   mp3.write(Send_buf[i]) ;           //send bit to serial mp3
   Serial.print(Send_buf[i],HEX);     //send bit to serial monitor in pc
 }
 Serial.println();
}
void Mode() {
  val1 = digitalRead(pbuttonPin2);
  if(val1 == HIGH && lightON2 == LOW){
    pushed2 = 1-pushed2;
    delay(100);
  }    
  lightON2 = val1;
      if(pushed2 == HIGH){
        Serial.println("Mode Mega");
        digitalWrite (changemode,HIGH);
      }else{
        Serial.println("Mode Pi");
        digitalWrite (changemode, LOW);
      }     
  delay(100);
}
