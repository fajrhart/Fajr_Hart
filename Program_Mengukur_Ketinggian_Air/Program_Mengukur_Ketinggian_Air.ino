// ---------------------------------------------
//               Sensor jarak HC-SR04
//      untuk mengukur tinggi air dalam gelas
// --------------------------------------------- 
// Copyright (c) Fajr_Hart
#define PIN_TRIG 13
#define PIN_ECHO 12

const double JARAK_DASAR = 23.12; // Diperoleh saat
                                  // gelas disingkirkan

void setup() 
{
  Serial.begin (9600);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

void loop() 
{
  // Berikan isyarat HIGH pada pin trig 10 mikrodetik
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);  // Tunda 10 mikrodetik 
  digitalWrite(PIN_TRIG, LOW);
  
  // Baca hasilnya di pin echo
  double selang = pulseIn(PIN_ECHO, HIGH);
  
  // Hitung jarak yang diperoleh
  double tinggi = JARAK_DASAR - 0.0343 * ( selang / 2);
  
  Serial.print(tinggi);
  Serial.println(" cm");
  
  delay(1000); // Tunda satu detik
}
