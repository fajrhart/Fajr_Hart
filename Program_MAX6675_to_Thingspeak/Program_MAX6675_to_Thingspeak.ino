#include <ESP8266WiFi.h>
String apiKey = "J8SV8Q93SAC5FZLU";     //  Enter your Write API key from ThingSpeak
const char *ssid =  "Sony";     // replace with your wifi ssid and wpa2 key
const char *pass =  "saufika09";
const char* server = "api.thingspeak.com";
#include <SPI.h>
#define MAX6675_CS   D7
#define MAX6675_SO   D6
#define MAX6675_SCK  D5 
int suhu;
WiFiClient client;
void setup() {
  Serial.begin(9600);
  delay(10);
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED){
  delay(500);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
}
void loop() {
  // put your main code here, to run repeatedly:
  suhu = readThermocouple();
  Serial.print(readThermocouple());
  Serial.println('c');
  delay(500);
  if (client.connect(server,80))   //   "184.106.153.149" or api.thingspeak.com
                      {  
                            
                             String postStr = apiKey;
                             postStr +="&field1=";
                             postStr += String(suhu);
                             postStr += "\r\n\r\n";
 
                             client.print("POST /update HTTP/1.1\n");
                             client.print("Host: api.thingspeak.com\n");
                             client.print("Connection: close\n");
                             client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
                             client.print("Content-Type: application/x-www-form-urlencoded\n");
                             client.print("Content-Length: ");
                             client.print(postStr.length());
                             client.print("\n\n");
                             client.print(postStr);
                             Serial.print("Suhu: ");
                             Serial.print(suhu);
                             Serial.println("C. Kirim ke Thingspeak.");
                        }
          client.stop();
          Serial.println("Menunggu.....");
  
  // thingspeak needs minimum 15 sec delay between updates, i've set it to 30 seconds
  delay(15000);
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
