#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266_SSL.h>
#include <ArduinoJson.h>
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "55ee7ede510846349aceee18f5606e9f";
int pinA = D1;
int pinB = D2;
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Sony";
char pass[] = "saufika00";
SoftwareSerial NodeMCU(D6,D5);
WidgetLCD lcd(V1);
WidgetTerminal terminal(V2);
void setup()
{
  // Debug console
  Serial.begin(9600);
  NodeMCU.begin(115200);
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  digitalWrite (pinA, LOW);
  digitalWrite (pinB, LOW);
  while (!Serial)continue;
  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  lcd.clear(); //Use it to clear the LCD Widget
}
void loop()
{
  Blynk.run();
  Arduino();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}
void Arduino()
{
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(NodeMCU);
  if (root == JsonObject::invalid())
  {
    return;
  }
  //Print the data in the serial monitor
  Serial.println("JSON received and parsed");
  root.prettyPrintTo(Serial);
  Serial.println("");
  Serial.print("Kadar Debu  ");
  int data1=root["dust"];
  lcd.print(2, 0, "Kadar Debu:"); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
  lcd.print(3, 1, data1);
  lcd.print(7, 1, "ug/m3");
  lcd.clear();
  terminal.print("Kadar Debu :");
  terminal.print(data1);
  terminal.println("ug/m3");
  terminal.flush();
  Serial.println(data1);
  Serial.println("");
  Serial.println("---------------------xxxxx--------------------");
  Serial.println("");
}
