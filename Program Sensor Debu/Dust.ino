/////////////////////////////////////////////////////////////////////////////
// Sharp GP2Y1014AU0F Dust Sensor 
// Board Connection:
//   GP2Y1014    Arduino
//   V-LED       Between R1 and C1
//   LED-GND     C1 and GND
//   LED         Pin 7
//   S-GND       GND
//   Vo          A5
//   Vcc         5V
//
// Serial monitor setting:
//   9600 baud
/////////////////////////////////////////////////////////////////////////////
#define USE_AVG

// Arduino pin
const int sharpLEDPin = 51;   // Arduino digital pin 7 connect to sensor LED.
const int sharpVoPin = A14;   // Arduino analog pin 5 connect to sensor Vo.
#ifdef USE_AVG
#define N 100
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0;
#endif // USE_AVG
static float Voc = 0.6;
const float K = 0.5;
/////////////////////////////////////////////////////////////////////////////
void printValue(String text, unsigned int value, bool isLast = false) {
  Serial.print(text);
  Serial.print("=");
  Serial.print(value);
  if (!isLast) {
    Serial.print(", ");
  }
}
void printFValue(String text, float value, String units, bool isLast = false) {
  Serial.print(text);
  Serial.print("=");
  Serial.print(value);
  Serial.print(units);
  if (!isLast) {
    Serial.print(", ");
  }
}

/////////////////////////////////////////////////////////////////////////////
void setup() {
  // Set LED pin for output.
  pinMode(sharpLEDPin, OUTPUT);
  Serial.begin(9600);
  delay(2000);
  Serial.println("");
  Serial.println("GP2Y1014AU0F Demo");
  Serial.println("=================");
}
void loop() {  
  // Turn on the dust sensor LED by setting digital pin LOW.
  digitalWrite(sharpLEDPin, LOW);
  delayMicroseconds(280);
  int VoRaw = analogRead(sharpVoPin);
  digitalWrite(sharpLEDPin, HIGH);
  delayMicroseconds(9620);
  #ifdef PRINT_RAW_DATA
  printValue("VoRaw", VoRaw, true);
  Serial.println("");
  #endif // PRINT_RAW_DATA
  float Vo = VoRaw;
  #ifdef USE_AVG
  VoRawTotal += VoRaw;
  VoRawCount++;
  if ( VoRawCount >= N ) {
    Vo = 1.0 * VoRawTotal / N;
    VoRawCount = 0;
    VoRawTotal = 0;
  } else {
    return;
  }
  #endif // USE_AVG
  Vo = Vo / 1024.0 * 5.0;
  printFValue("Vo", Vo*1000.0, "mV");
  float dV = Vo - Voc;
  if ( dV < 0 ) {
    dV = 0;
    Voc = Vo;
  }
  float dustDensity = dV / K * 100.0;
  printFValue("DustDensity", dustDensity, "ug/m3", true);
  Serial.println("");
}
