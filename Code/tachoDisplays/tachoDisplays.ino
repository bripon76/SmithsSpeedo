#include <EEPROM.h>
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//Tacho-Stepper Motor
#define STEPS_KMH_140 690.0 /*Steps per 140 km/h*/
#define STEPS_MAXRANGE 850 /* Rechte Seite FUEL Ausschnitt*/
#include <SwitecX25.h>

// standard X25.168 range 315 degrees at 1/3 degree steps
#define STEPS (315*3)
// For motors connected to digital pins 4,5,6,7
SwitecX25 motor1(STEPS, 4, 5, 6, 7);

U8G2_SH1106_128X64_NONAME_1_HW_I2C fuel(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); //Grosses Display (FUEL) X:10..117 Y:17..44
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C kilometer(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  //kleines Display (Kilometerstand)


unsigned long totalkilometers = 0; //1503880; //x10
char kilometerBuffer[10]; //Textpuffer für Kilometerdisplay

/*********Für motorische analoge Anzeige******/
double maxSpeed=160;
double kmh=0;
struct stuetzstellen {
  int skala;
  int steps;
};

struct stuetzstelleEintrag {
  int links;
  int rechts;
};

int listLength = 9; //10 Einträge, ab 0. Eintrag gezählt
stuetzstellen skalaWerte[] = {
  {.skala = 0, .steps = 0},
  {.skala = 20, .steps = 50},
  {.skala = 40, .steps = 160},
  {.skala = 60, .steps = 268},
  {.skala = 80, .steps = 375},
  {.skala = 100, .steps = 478},
  {.skala = 120, .steps = 582},
  {.skala = 140, .steps = 683},
  {.skala = 150, .steps = 743},
  {.skala = 160, .steps = 815}
};

//Argumente: xn=km/h-Wert
stuetzstelleEintrag findListEntry(double xn) {
  stuetzstelleEintrag returnStruct;
  int returnValue = 0;
  //Rechte Stützstelle finden
  for (int i = 0; i < listLength; i++) {
    returnValue = i;
    if (skalaWerte[i].skala >= xn) break;
  }
  returnStruct.rechts = returnValue;

  returnValue = listLength;
  //Linke Stützstelle finden
  for (int i = listLength; i >= 0; i--) {
    returnValue = i;
    if (skalaWerte[i].skala <= xn) break;
  }
  returnStruct.links = returnValue;
  return returnStruct;
}

int linearInterpolation(double xn, double x1, int y1, double x2, int y2) {
  int yn = y1 + ((y2 - y1) / (x2 - x1)) * (xn - x1);
  return yn;
}

/*********Ende motorische analoge Anzeige******/



/****** Tanknadel *******/
double minAngle = (70.0 / 180.0) * 3.14159;
double maxAngle = (110.0 / 180.0) * 3.14159;
double alpha = 0;
uint32_t nextFuelMillis = 0; //Millisekunden, bis Display refresh bekommen soll
uint32_t nextNeedleMillis = 0; //Millisekunden, bis Nadel refresh bekommen soll


int xShift = 63, yShift = -140;
int r1 = 151;
int r2 = 171;
int x1, y1, x2, y2;


void setup() {
  // run the motor against the stops
  motor1.zero();
  // start moving towards the center of the range
  motor1.setPosition(0);

  Serial.begin(9600);
  Serial.println("Enter kmh from 0 through 160");


  // put your setup code here, to run once:
  fuel.setI2CAddress(0x7A);
  fuel.begin();
  kilometer.begin();
  fuel.firstPage();
  do {

    fuel.setFont(u8g2_font_logisoso16_tr);
    fuel.setFontPosTop();
    fuel.drawStr(37, 22, "SMITHS");
    fuel.drawStr(37, 40, "Digital");
  } while ( fuel.nextPage() );


  kilometerText(totalkilometers);
  kilometer.setContrast(0);
  kilometer.firstPage();
  do {
    kilometer.setFont(u8g2_font_logisoso22_tn);
    kilometer.drawStr(0, 28, kilometerBuffer);
  } while ( kilometer.nextPage());
  motor1.setPosition(STEPS_MAXRANGE);
  motor1.updateBlocking();
  motor1.setPosition(0);
  motor1.updateBlocking();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis()>nextNeedleMillis){
  nextNeedleMillis=millis()+100;
  alpha += 0.01;
  if (alpha < minAngle || alpha >= maxAngle)alpha = minAngle;
  // put your main code here, to run repeatedly:
  x1 = (cos(alpha) * r1) + xShift;
  y1 = (sin(alpha) * r1) + yShift;
  x2 = (cos(alpha) * r2) + xShift;
  y2 = (sin(alpha) * r2) + yShift;
  }
  if (millis() > nextFuelMillis) {
    nextFuelMillis = millis() + 1000;
    fuel.firstPage();
    do {
      fuel.setFontPosTop();
      if(kmh>maxSpeed){
        fuel.setFont(u8g2_font_logisoso32_tn);
        fuel.setCursor(18, 12);
        fuel.print(kmh,1);
      }
      else{
      fuel.setFont(u8g2_font_logisoso16_tr);
      fuel.drawStr(10, 24, "E");
      fuel.drawStr(50, 28, "1/2");
      fuel.drawStr(108, 24, "F");
      fuel.drawLine(x1, y1, x2, y2); //Nadel
      fuel.drawLine(x1+1, y1, x2+1, y2); //Nadel
      fuel.drawLine(x1-1, y1, x2-1, y2); //Nadel
      }
    } while ( fuel.nextPage() );
  }


  static int nextPos = 0;
  // the motor only moves when you call update
  motor1.update();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 10 || c == 13) {
      kmh=nextPos;
      if (nextPos > 160)nextPos = maxSpeed;
      if (nextPos < 0)nextPos = 0;
      
      stuetzstelleEintrag berechnung = findListEntry(nextPos);
      int yn;
      if (berechnung.links == berechnung.rechts) yn = skalaWerte[berechnung.links].steps;
      else {
        yn = linearInterpolation(nextPos, skalaWerte[berechnung.links].skala, skalaWerte[berechnung.links].steps,
                                 skalaWerte[berechnung.rechts].skala, skalaWerte[berechnung.rechts].steps);
      }
      motor1.setPosition(yn);
      nextPos = 0;
    } else if (c >= '0' && c <= '9') {
      nextPos = 10 * nextPos + (c - '0');
    }
  }
}

//Kilometerstand speichern
void writeEepromWL24(uint32_t value) {
  //Cap value. at 24 Bits (3 Bytes)
  value &= 0x00FFFFFF;
  //Find address for Low Byte
  uint16_t hiAddr = 255; //Base address
  uint16_t midAddr = hiAddr - 1;
  uint16_t loAddr = ((value & 0x00FF0000) >> 16) % 253;

  //EEPROM.write(addr, val);
  EEPROM.write(hiAddr, ((value & 0x00FF0000) >> 16));
  EEPROM.write(midAddr, ((value & 0x0000FF00) >> 8));
  EEPROM.write(loAddr, (value & 0x000000FF));
}

uint32_t readEepromWL24() {
  //Find address for Low Byte
  uint16_t hiAddr = 255; //Base address
  uint16_t midAddr = hiAddr - 1;
  //value = EEPROM.read(address);
  uint16_t loAddr = EEPROM.read(hiAddr);
  uint32_t value = EEPROM.read(loAddr);
  value |= (EEPROM.read(midAddr) << 8);
  value |= (EEPROM.read(hiAddr) << 16);
}

//Kilometer auf Kilometerzähler anzeigen
void kilometerText(unsigned long totalkilometers) {
  unsigned int numbers[8] = {};
  numbers[0] = (totalkilometers / 10000000) % 10;
  numbers[1] = (totalkilometers / 1000000) % 10;
  numbers[2] = (totalkilometers / 100000) % 10;
  numbers[3] = (totalkilometers / 10000) % 10;
  numbers[4] = (totalkilometers / 1000) % 10;
  numbers[5] = (totalkilometers / 100) % 10;
  numbers[6] = (totalkilometers / 10) % 10;
  numbers[7] = (totalkilometers) % 10;
  sprintf(kilometerBuffer, "%0u%0u%0u%0u%0u%0u%0u,%0u", numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6], numbers[7]);
}
