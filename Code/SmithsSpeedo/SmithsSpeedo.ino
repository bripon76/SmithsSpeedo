//To Do:
//Kilometer alle 10 min. speichern (optional)
//Tageskilometerzähler (Differenz zwischen realem Kilometerzähler und letztem Reset-Kilometerstand)

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

//SoftwareSerial
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX

// standard X25.168 range 315 degrees at 1/3 degree steps
#define STEPS (315*3)
// For motors connected to digital pins 4,5,6,7
SwitecX25 motor1(STEPS, 4, 5, 6, 7);

U8G2_SH1106_128X64_NONAME_1_HW_I2C fuel(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); //Grosses Display (FUEL) X:10..117 Y:17..44
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C kilometer(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  //kleines Display (Kilometerstand)

/********* Kilometerzähler ***********/
#define ORIGINAL_KILOMETERS 1503880
unsigned long totalKilometers = 0; //; //x10
unsigned long totalPulses = 0;
char kilometerBuffer[10]; //Textpuffer für Kilometerdisplay
unsigned long nextKilometerMillis = 0;
int kilometerTimeout = 0; //Zeitmessung für Kilometer speichern im Stand
#define KILOMETER_TIMEOUT_SECONDS 10 /*Timeout in Sekunden*/
int tagesKilometerReset=0; //Tageskilometerzähler letzter Reset-Stand
char tagesKilometerBuffer[8]={};//Textpuffer für Tageskilometerdisplay


unsigned long displayKilometers=0; //Zwischenspeicher für Displays (Berechnung Kilometerstand aus Pulsen)

/********* Geschwindigkeitsmessung ***********/
volatile uint16_t timerSteps = 0;

#define STEPS_REV 5 /*Steps/rev*/
#define CIRCUMFERENCE 1.531 /*m*/
uint32_t nextSampleMillis = 0;

/*********Für motorische analoge Anzeige******/
double maxSpeed = 160;
double kmh = 0;
double lastKmh = 0;
struct stuetzstellen {
  int skala;
  int steps;
};

struct stuetzstelleEintrag {
  int links;
  int rechts;
};

int skalaListLength = 10; //10 Einträge
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
stuetzstelleEintrag findListEntry(double xn, stuetzstellen* stuetzstellenliste, uint8_t listLength) {
  stuetzstelleEintrag returnStruct;
  int returnValue = 0;
  //Rechte Stützstelle finden
  for (int i = 0; i < listLength; i++) {
    returnValue = i;
    if (stuetzstellenliste[i].skala >= xn) break;
  }
  returnStruct.rechts = returnValue;

  returnValue = listLength - 1;
  //Linke Stützstelle finden
  for (int i = (listLength - 1); i >= 0; i--) {
    returnValue = i;
    if (stuetzstellenliste[i].skala <= xn) break;
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
int fuelListLength = 7; //10 Einträge
//steps=liter, skala=ADC Value
stuetzstellen fuelWerte[] = {
  {.skala = 293, .steps = 20},
  {.skala = 306, .steps = 17},
  {.skala = 333, .steps = 13},
  {.skala = 448, .steps = 8},
  {.skala = 591, .steps = 5},
  {.skala = 721, .steps = 3},
  {.skala = 968, .steps = 0}
};

double maxLiters = 20;
double minAngle = (70.0 / 180.0) * 3.14159; //Rechter Anschlag
double maxAngle = (108.0 / 180.0) * 3.14159; //Linker Anschlag
double needleAngle = 0;
uint32_t nextFuelMillis = 0; //Millisekunden, bis Display refresh bekommen soll
uint32_t nextNeedleMillis = 0; //Millisekunden, bis Nadel refresh bekommen soll
double lastneedleAngle = maxAngle; //Für Filterfunktion (Start bei maxAngle=Start bei Minimum)

int xShift = 63, yShift = -140;
int r1 = 151;
int r2 = 171;
int x1, y1, x2, y2;
/****** Ende Tanknadel *******/

/******** Temperatur/Feuchtesensor******/

// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#define DHTPIN 9     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHT_VCC_PIN 8
#define DHT_GND_PIN 10
uint32_t nextTemperatureMillis = 0;
float temperature, humidity;
DHT dht(DHTPIN, DHTTYPE);


void setup() {
  Serial.begin(9600);

  //Softwareserial
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");

  //Kilometerstand aus EEPROM holen
  totalKilometers = readEepromWL24();
  tagesKilometerReset = readEeprom24(256); //256...258

  
  Serial.println(totalKilometers);

  //Timer1 Setup für Tachogeber
  TIMSK1 = (1 << TOIE1); //Enable Timer Overflow Interrupt
  TCCR1A = 0; //Normal Mode, no pins connected
  TCCR1B = 0 ; //Stop Timer1, is activated by first pulse on Int0
  TCNT1 = 0;
  //TCCR1B = (1 << CS12); //Pre-Scaler 256 --> 16 MHz/256=62.5 kHz (0.95 Hz min)
  //Timer is configured with 16 µs--> resolution.

  //Interrupt Pin setup
  attachInterrupt(digitalPinToInterrupt(2), freqMeasure, RISING);


  //Initialize DHT Temperature Sensor
  //GND and VCC Pins
  
  pinMode(DHT_VCC_PIN, OUTPUT);
  pinMode(DHT_GND_PIN, OUTPUT);
  digitalWrite(DHT_GND_PIN, LOW);
  digitalWrite(DHT_VCC_PIN, HIGH);
  dht.begin();
  //Dummy reading
  double dummy = dht.readTemperature();
  
  // run the motor against the stops
  motor1.zero();
  // start moving towards the center of the range
  motor1.setPosition(0);


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



  kilometerText(totalKilometers);
  drawKilometer();

  //ist EEPROM fabrikneu? Kilometerstand reinschreiben
  if (totalKilometers == 0x00FFFFFF) {
    writeEepromWL24(ORIGINAL_KILOMETERS);
    totalKilometers = ORIGINAL_KILOMETERS;
  }

  motor1.setPosition(STEPS_MAXRANGE);
  motor1.updateBlocking();
  motor1.setPosition(0);
  motor1.updateBlocking();

  //Serial.println("unfiltered\tfiltered");
}

uint32_t globalMillis = 0;

void loop() {



  globalMillis = millis();
  // Geschwindigkeitsmessung
  if (globalMillis > nextSampleMillis) {
    double tempKmh = 0;
    nextSampleMillis = globalMillis + 100;
    if (timerSteps != 0) tempKmh = (3.6 * (CIRCUMFERENCE / STEPS_REV) / (0.000016 * (double)timerSteps));
    else tempKmh = 0;
    kmh = (0.9 * lastKmh) + (0.1 * tempKmh); //Filterfunktion
    //Serial.print(tempKmh); Serial.print('\t'); Serial.println(kmh);


    lastKmh = kmh;

    //Serial.print(lastKmh);

    int nextPos = kmh;
    if (nextPos > maxSpeed)nextPos = maxSpeed;
    if (nextPos < 0)nextPos = 0;
    //Tachonadel
    stuetzstelleEintrag berechnung = findListEntry(nextPos, skalaWerte, skalaListLength);
    int yn;
    if (berechnung.links == berechnung.rechts) yn = skalaWerte[berechnung.links].steps;
    else {
      yn = linearInterpolation(nextPos, skalaWerte[berechnung.links].skala, skalaWerte[berechnung.links].steps,
                               skalaWerte[berechnung.rechts].skala, skalaWerte[berechnung.rechts].steps);
    }
    motor1.setPosition(yn);
  }



  // Kilometerstand
  if (globalMillis > nextKilometerMillis) {

    nextKilometerMillis = globalMillis + 1400;

    //aus totalPulses Kilometerstand x10 berechnen
    displayKilometers = totalKilometers + ((totalPulses / STEPS_REV) * CIRCUMFERENCE) / 100;
    kilometerText(displayKilometers);
    drawKilometer();
    //Serial.print(displayKilometers);
    }



  
  // Wait a few seconds between measurements.
  if (globalMillis > nextTemperatureMillis) {
    nextTemperatureMillis = globalMillis + 10000;
    //humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temperature = dht.readTemperature();
  }

 
  // put your main code here, to run repeatedly:
  if (globalMillis > nextNeedleMillis) {
    nextNeedleMillis = globalMillis + 1000;
    int fuelAnalog = analogRead(0);
    double liters;
    stuetzstelleEintrag berechnung = findListEntry(fuelAnalog, fuelWerte, fuelListLength);
    if (berechnung.links == berechnung.rechts) liters = fuelWerte[berechnung.links].steps;
    else {
      liters = linearInterpolation(fuelAnalog, fuelWerte[berechnung.links].skala, fuelWerte[berechnung.links].steps,
                                   fuelWerte[berechnung.rechts].skala, fuelWerte[berechnung.rechts].steps);
    }

    double newAngle = maxAngle - (((maxAngle - minAngle) / maxLiters) * liters);
    needleAngle = (newAngle * 0.3) + (lastneedleAngle * 0.7);
    lastneedleAngle = needleAngle;
    if (needleAngle < minAngle) needleAngle = minAngle;
    if (needleAngle >= maxAngle) needleAngle = maxAngle;
    // put your main code here, to run repeatedly:
    x1 = (cos(needleAngle) * r1) + xShift;
    y1 = (sin(needleAngle) * r1) + yShift;
    x2 = (cos(needleAngle) * r2) + xShift;
    y2 = (sin(needleAngle) * r2) + yShift;
  }


  //Tanknadel Display zeichnen und Kilometer speichern
  if (globalMillis > nextFuelMillis) {
    nextFuelMillis = globalMillis + 1000;
    //Hier wird ein Sekundentakt abgegriffen, um die Zeit im Stand zu messen (für Kilometer im EEPROM speichern)
    if (kmh == 0 && kilometerTimeout) {
      if (kilometerTimeout <= KILOMETER_TIMEOUT_SECONDS)
        kilometerTimeout++;
    } else { //if kmh !=0
      kilometerTimeout = 0;
    }

    //Kilometer speichern wenn kilometerTimeout Sollwert erreicht hat
    if(kilometerTimeout==KILOMETER_TIMEOUT_SECONDS) {
      writeEepromWL24(displayKilometers);
    }

    fuel.firstPage();
    do {
      fuel.setFontPosTop();

      //Wenn Tacho>MaxSpeed, dann in großer Schrift km/h darstellen

      if (kmh > maxSpeed) {
        fuel.setFont(u8g2_font_logisoso32_tn);
        fuel.setCursor(18, 18);
        fuel.print(kmh, 1);
      }

      //Anzeige der Tanknadel und der Temperatur etc.
      else {
        fuel.setFont(u8g2_font_logisoso16_tr);
        fuel.drawStr(10, 22, "E");
        fuel.drawStr(50, 26, "1/2");
        fuel.drawStr(106, 22, "F");
        fuel.drawLine(x1, y1, x2, y2); //Nadel
        fuel.drawLine(x1 + 1, y1, x2 + 1, y2); //Nadel
        fuel.drawLine(x1 - 1, y1, x2 - 1, y2); //Nadel

        
        fuel.setCursor(10, 45);
        fuel.print((int)temperature);
        fuel.print("*C");

        //Tageskilometerzähler
        fuel.setCursor(70, 45);
        tagesKilometerText(displayKilometers-tagesKilometerReset);
        fuel.print(tagesKilometerBuffer);
        //Serial.print(tagesKilometerBuffer);
      }
    } while ( fuel.nextPage());
  }

  // the motor only moves when you call update
  motor1.update();

espsenddata();


}

//Kilometerstand speichern
void writeEepromWL24(uint32_t myvalue) {
  //Cap value. at 24 Bits (3 Bytes)
  myvalue &= 0x00FFFFFF;
  //Find address for Low Byte
  uint16_t hiAddr = 255; //Base address
  uint16_t midAddr = hiAddr - 1;
  uint16_t loAddr = ((myvalue & 0x00FF0000) >> 16) % 253;

  //EEPROM.write(addr, val);
  EEPROM.write(hiAddr, ((myvalue & 0x00FF0000) >> 16));
  EEPROM.write(midAddr, ((myvalue & 0x0000FF00) >> 8));
  EEPROM.write(loAddr, (myvalue & 0x000000FF));
}

uint32_t readEepromWL24() {
  //Find address for Low Byte
  uint16_t hiAddr = 255; //Base address
  uint16_t midAddr = hiAddr - 1;
  //value = EEPROM.read(address);
  uint16_t loAddr = EEPROM.read(hiAddr);

  uint32_t myvalue = (EEPROM.read(loAddr) << 0) & 0x000000FF;
  myvalue |= (EEPROM.read(midAddr) << 8) & 0x0000FF00;
  myvalue |= ((uint32_t)loAddr << 16) & 0x00FF0000;
  return myvalue;
}

//Kilometerstand speichern
void writeEeprom24(uint32_t addr, uint32_t myvalue) {
  //Cap value. at 24 Bits (3 Bytes)
  myvalue &= 0x00FFFFFF;
  //Find address for Low Byte
  uint16_t hiAddr = addr+2; //Base address
  uint16_t midAddr = hiAddr - 1;
  uint16_t loAddr = hiAddr - 2;

  //EEPROM.write(addr, val);
  EEPROM.write(hiAddr, ((myvalue & 0x00FF0000) >> 16));
  EEPROM.write(midAddr, ((myvalue & 0x0000FF00) >> 8));
  EEPROM.write(loAddr, (myvalue & 0x000000FF));
}

uint32_t readEeprom24(uint32_t addr) {
  uint16_t hiAddr = addr+2; //Base address
  uint16_t midAddr = hiAddr - 1;
  uint16_t loAddr = hiAddr - 2;

  uint32_t myvalue = (EEPROM.read(loAddr) << 0) & 0x000000FF;
  myvalue |= (EEPROM.read(midAddr) << 8) & 0x0000FF00;
  myvalue |= ((uint32_t)loAddr << 16) & 0x00FF0000;
  return myvalue;
}

//Kilometer auf Kilometerzähler anzeigen
void kilometerText(unsigned long totalKilometers) {
  unsigned int numbers[8] = {};
  numbers[0] = (totalKilometers / 10000000) % 10;
  numbers[1] = (totalKilometers / 1000000) % 10;
  numbers[2] = (totalKilometers / 100000) % 10;
  numbers[3] = (totalKilometers / 10000) % 10;
  numbers[4] = (totalKilometers / 1000) % 10;
  numbers[5] = (totalKilometers / 100) % 10;
  numbers[6] = (totalKilometers / 10) % 10;
  numbers[7] = (totalKilometers) % 10;
  sprintf(kilometerBuffer, "%0u%0u%0u%0u%0u%0u%0u,%0u", numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6], numbers[7]);
}

//Tageskilometer auf Fuel-Anzeige anzeigen
void tagesKilometerText(unsigned long totalKilometers) {
  unsigned int numbers[4] = {};
  numbers[0] = (totalKilometers / 1000) % 10;
  numbers[1] = (totalKilometers / 100) % 10;
  numbers[2] = (totalKilometers / 10) % 10;
  numbers[3] = (totalKilometers) % 10;
  sprintf(tagesKilometerBuffer, "%0u%0u%0u,%0u", numbers[0], numbers[1], numbers[2], numbers[3]);
}

void drawKilometer() {
  kilometer.setContrast(0);
  kilometer.firstPage();
  do {
    kilometer.setFont(u8g2_font_logisoso22_tn);
    kilometer.drawStr(0, 28, kilometerBuffer);
  } while ( kilometer.nextPage());
}

//Timer Overflow Interrupt: Disable Timer0 clock, save timerSteps as 0.
ISR (TIMER1_OVF_vect) {
  TCCR1B = 0; //stop Timer1
  timerSteps = 0; //Set 0 timer steps
}

//Pin 2 Interrupt
void freqMeasure() {
  //TCCR1B = 0; //stop Timer1
  timerSteps = TCNT1; //Save Timer1 value
  TCCR1B = (1 << CS12); //Start Timer1
  TCNT1 = 0; //Reset Timer1
  totalPulses++;
}

void espsenddata(){
  #define SEPARATION_CHAR ','
  //ESP Data
  //{"TotalKm":150510,"DayKm":19,"AverageKmh":34,"AverageFuel":5.4,"Fuel":17,"TopSpeed":172}
  mySerial.print("{");
  mySerial.print(F("\"TotalKm\":\"")); mySerial.print(totalKilometers); mySerial.print('\"'); mySerial.print(SEPARATION_CHAR);
  mySerial.print(F("\"DayKm\":\"")); mySerial.print(tagesKilometerBuffer); mySerial.print('\"'); mySerial.print(SEPARATION_CHAR);
  mySerial.print(F("\"AverageKmh\":\"")); mySerial.print("Demo"); mySerial.print('\"'); mySerial.print(SEPARATION_CHAR);
  mySerial.print(F("\"AverageFuel\":\"")); mySerial.print("Demo"); mySerial.print('\"'); mySerial.print(SEPARATION_CHAR);
  mySerial.print(F("\"Fuel\":\"")); mySerial.print("Demo"); mySerial.print('\"'); mySerial.print(SEPARATION_CHAR);
  mySerial.print(F("\"TopSpeed\":\"")); mySerial.print("Demo"); mySerial.print('\"');
  mySerial.print("}");
  //Finally, print a newline.
  mySerial.println();
  
  delay(2000);

}
