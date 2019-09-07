#include <HCMAX7219.h>
#include "SPI.h"
#define LOAD 10
HCMAX7219 HCMAX7219(LOAD);
void setup(){       
}
void loop(){
  byte Loopcounter;
  int Position;
  HCMAX7219.Clear();
  for (Loopcounter = 0; Loopcounter <= 2; Loopcounter++)
  {
    for(Position=0; Position <= 64; Position++)
    {
      HCMAX7219.printMatrix("TEXT", Position);
      HCMAX7219.Refresh();
      delay(80);
    }
  }
}
