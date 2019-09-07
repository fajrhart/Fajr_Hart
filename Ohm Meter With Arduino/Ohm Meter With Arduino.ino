#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

int analogPin= 0;    // pin analog A0
int V_diukur= 0;
int Vin= 5;          // tegangan input reff 5v 
float Vout= 0;
int apply_voltage = 2;

float R2= 0;
float buffer= 0;

int ch2K = 5;         // pin resistor 2k
int ch20K = 4;        // pin resistor 20k
int ch200K = 3;       // pin resistor 200k
int ch1M = 6;         // pin resistro 1m

int range2k =9;        // pin range 2k
int range20k =10;      // pin range 20k
int range200k =11;     // pin range 200k
int range1M =12;       // pin range 1m

void setup()
{
  lcd.begin();
  Serial.begin(9600);
  pinMode(V_diukur,INPUT);
  pinMode(analogPin,INPUT);
  pinMode(apply_voltage,OUTPUT);
  
  //range mode menjadi input (0-1kK; 10k-100k; 100k-1M)
  pinMode(range2k,INPUT);
  pinMode(range20k,INPUT);
  pinMode(range200k,INPUT);
  pinMode(range1M,INPUT);


  // pin resistor menjadi input 
  pinMode(ch2K,INPUT);
  pinMode(ch20K,INPUT);
  pinMode(ch200K,INPUT);
  pinMode(ch1M,INPUT);

}

void loop()
{
///////////////////-2k-/////////////////////
if (digitalRead(range2k))
{  
  digitalWrite(apply_voltage,HIGH);
  pinMode(ch2K,OUTPUT);
  pinMode(ch20K,INPUT);
  pinMode(ch200K,INPUT);
  pinMode(ch1M,INPUT);
  digitalWrite(ch2K,LOW);
  
  float R1= 2; // Tetapkan nilai ini ke nilai resistor yang digunakan dalam K ohm
  V_diukur= analogRead(analogPin); //in 8bits
  
      buffer= V_diukur * Vin;
      Vout= (buffer)/1023.0;  //in volts
      buffer= (Vin/Vout) -1;
      R2= R1 * buffer*1000; // dikali 1000 karena 1K ohm
  
      if (R2 > 2000)
      {
        Serial.print("   Plih Range");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("   Plih Range");
        lcd.setCursor(0,1); 
        lcd.print("  Dan Pasang R");      
        delay(1000);
      }
  
      if (R2 < 2000)
      {
        Serial.print("----[2K]----");
        Serial.print("\n");
        Serial.print("\n");
        Serial.print("Resistance: ");
        Serial.print(R2);
        Serial.print(" ");
        Serial.print("  ohms");
        Serial.print("\n");
        
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("RANGE:    0 - 2K"); // range 0 - 2k
        lcd.setCursor(0,1); 
        lcd.print(R2);

        lcd.setCursor(10,1); 
        lcd.print("  ohms");
        delay(1000);
      }
    
}
////////////////////////////////////////////

///////////////////-20k-/////////////////////
if (digitalRead(range20k))
{  
  digitalWrite(apply_voltage,HIGH);
  pinMode(ch2K,INPUT);
  pinMode(ch20K,OUTPUT);
  pinMode(ch200K,INPUT);
  pinMode(ch1M,INPUT);
  digitalWrite(ch20K,LOW);
  
  float R1= 20; // Tetapkan nilai ini ke nilai resistor yang digunakan dalam K ohm
  V_diukur= analogRead(analogPin); //in 8bits
  
      buffer= V_diukur * Vin;
      Vout= (buffer)/1023.0;  //in volts
      buffer= (Vin/Vout) -1;
      R2= R1 * buffer;
  
      if (R2 > 20)
      {
        Serial.print("   Plih Range");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("   Plih Range");
        lcd.setCursor(0,1); 
        lcd.print("  Dan Pasang R");        
        delay(1000);
      }
  
      if (R2 < 20)
      {
        Serial.print("----[20K]----");
        Serial.print("\n");
        Serial.print("\n");
        Serial.print("Resistance: ");
        Serial.print(R2);
        Serial.print(" ");
        Serial.print("K ohms");
        Serial.print("\n");
        
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("RANGE:   2 - 20K"); // range 2k - 20k
        lcd.setCursor(0,1); 
        lcd.print(R2);

        lcd.setCursor(10,1); 
        lcd.print("K ohms");
        delay(1000);
      }
    
}
////////////////////////////////////////////

///////////////////-200k-/////////////////////
if (digitalRead(range200k))
{  
  digitalWrite(apply_voltage,HIGH);
  pinMode(ch2K,INPUT);
  pinMode(ch20K,INPUT);
  pinMode(ch200K,OUTPUT);
  pinMode(ch1M,INPUT);
  digitalWrite(ch200K,LOW);
  
  float R1= 200; // Tetapkan nilai ini ke nilai resistor yang digunakan dalam K ohm
  V_diukur= analogRead(analogPin); //in 8bits
  
      buffer= V_diukur* Vin;
      Vout= (buffer)/1023.0;  //in volts
      buffer= (Vin/Vout) -1;
      R2= R1 * buffer;
  
      if (R2 > 200)
      {
        Serial.print("   Plih Range");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("   Plih Range");
        lcd.setCursor(0,1); 
        lcd.print("  Dan Pasang R");        
        delay(1000);
      }
  
      if (R2 < 200)
      {
        Serial.print("----[200K]----");
        Serial.print("\n");
        Serial.print("\n");
        Serial.print("Resistance: ");
        Serial.print(R2);
        Serial.print(" ");
        Serial.print("K ohms");
        Serial.print("\n");
        
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("RANGE: 20 - 200K"); // range 20k -200k
        lcd.setCursor(0,1); 
        lcd.print(R2);

        lcd.setCursor(10,1); 
        lcd.print("K ohms");
        delay(1000);
      }    
}
////////////////////////////////////////////

///////////////////-1M-/////////////////////
if (digitalRead(range1M))
{  
  digitalWrite(apply_voltage,HIGH);
  pinMode(ch2K,INPUT);
  pinMode(ch20K,INPUT);
  pinMode(ch200K,INPUT);
  pinMode(ch1M,OUTPUT);
  digitalWrite(ch1M,LOW);
  
  float R1= 1; // Tetapkan nilai ini ke nilai resistor yang digunakan dalam M ohm
  V_diukur= analogRead(analogPin); //in 8bits
 
      buffer= V_diukur * Vin;
      Vout= (buffer)/1023.0;  //in volts
      buffer= (Vin/Vout) -1;
      R2= R1 * buffer;
  
      if (R2 > 2)
      {
        Serial.print("   Plih Range");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("   Plih Range");
        lcd.setCursor(0,1); 
        lcd.print("  Dan Pasang R");      

        lcd.setCursor(10,1); 
        lcd.print("M ohms");       
        delay(1000);
      }
  
      if (R2 < 2)
      {
        Serial.print("----[1M]----");
        Serial.print("\n");
        Serial.print("\n");
        Serial.print("Resistance: ");
        Serial.print(R2);
        Serial.print(" ");
        Serial.print("M ohms");
        Serial.print("\n");
        
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("RANGE:  200 - 1M");  // range 200k - 1m
        lcd.setCursor(0,1); 
        lcd.print(R2);

        lcd.setCursor(10,1); 
        lcd.print("M ohms");
        delay(1000);
      }
    
}
////////////////////////////////////////////

if ((digitalRead(range2k)==LOW)  &&  (digitalRead(range20k)==LOW)  &&  (digitalRead(range200k)==LOW)   &&  (digitalRead(range1M)==LOW)) // ketika semua kondisi low
  { 
    Serial.print("   Plih Range");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   Plih Range");
    lcd.setCursor(0,1); 
    lcd.print("  Dan Pasang R");       
    Serial.print("\n");        
    delay(1000);
  }
}
