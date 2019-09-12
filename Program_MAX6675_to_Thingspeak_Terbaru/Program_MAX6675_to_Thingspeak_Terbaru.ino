#include <ESP8266WiFi.h>;
#include <WiFiClient.h>;
#include <ThingSpeak.h>;
#include <SPI.h>;
const char* ssid = "MAIN HALL"; //Your Network SSID 
const char* password = "officejoe"; //Your Network Password
float suhu;
#define MAX6675_CS   D7
#define MAX6675_SO   D6
#define MAX6675_SCK  D5 
WiFiClient client; 
unsigned long myChannelNumber = 859052; //Your Channel Number (Without Brackets)
const char * myWriteAPIKey = "J8SV8Q93SAC5FZLU"; //Your Write API Key
void setup(){
Serial.begin(9600);
delay(10);
// Connect to WiFi network
WiFi.begin(ssid, password);
ThingSpeak.begin(client);
} 
void loop(){
 suhu = readThermocouple();
 Serial.print(readThermocouple());
 Serial.println('c');
 delay(15000); 
 ThingSpeak.writeField(myChannelNumber, 1,suhu, myWriteAPIKey); //Update in ThingSpeak
}
double readThermocouple() {
  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  digitalWrite(MAX6675_CS, LOW);
  delay(1);
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) 
  {    
    return NAN;     
  }
  v >>= 3;
  return v*0.25;
}
