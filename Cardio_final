/*
 * Huy Nguyen - Khe Bach
 * EE 474
 * Winter 2017
 * Lab 8
 */
#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  while (1);
}

/* The service information */

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;
//---------------------------------------------------------------------------------
#include <SPI.h>
#include "ILI9341_t3.h"
#include <SdFat.h>
#include <SPI.h>


// For the Adafruit shield, these are the default.

#define TFT_DC 9
#define TFT_CS 10

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

// set up config for PDB
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))

/*
  ADC_CFG1_ADIV(1)         Divide ratio = 2 (F_BUS = 48 MHz => ADCK = 12 MHz)
  ADC_CFG1_MODE(1)         Single ended 12 bit mode
  ADC_CFG1_ADLSMP          Long sample time
*/

#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

/*
  ADC_CFG2_MUXSEL          Select channels ADxxb
  ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// SD card variable
SdFat sd;
SdFile cFile;

//HR detection
int limit = 12000;
uint16_t HR  = 0;
uint16_t HRold  = 0;
float RR = 0;
float RRold = 0;
float QRS = 0;
float QRSold = 0;
float PRI = 0;
float PRIold = 0;

String heartType = "";
String heartTypeOld = "";

// Calibrating
int samples = 0;
const int CS = 4;
uint16_t sampleRates = 250;
int fileNo = 1;
uint16_t isCompleted = 0;

// Array of data
uint16_t dataSize = 7400;
int data[7400];
uint16_t rawData[7400];
uint16_t dataNo = 0;

// Moving Avg
float currentAvg;
float lastAvg;
float lastCurrentAvg;
float dataHB;

// HR Avg
uint16_t currentHR;
uint16_t lastHR;
uint16_t lastCurrentHR;
uint16_t n;

//RR Avg
uint16_t currentRR;
uint16_t lastRR;
uint16_t lastCurrentRR;
uint16_t m;

//RR Avg
uint16_t currentQRS;
uint16_t lastQRS;
uint16_t lastCurrentQRS;
uint16_t l;

int rawVal;
int val;

// Stablizing
int startTime = 0;
int currentTime = 0;
int isRunning = 0;
int isStable = 0;
int envelopeTimeStart = 0;
int increment = 0;


// Button
uint16_t flag = 1; // flag for control the state
uint16_t buttonFlag1 = 1;
uint16_t state = 1; // state on or off
uint16_t menuState = 1;
uint16_t isStopping = 0;
int k = 0;

// SD flag
uint16_t readFile = 0;

//Display flag
uint16_t onMenu = 0;

//Calirbrating
float startTimeHR = 0;
float endTimeHR = 0;
uint8_t isCounting = 0;

// QRS detection
uint16_t QRSflag = 0;
uint16_t QRScount = 0;
uint16_t Qindex = 0;
uint16_t Rindex = 0;
boolean firstZero = false;
uint16_t indexQRS = -1;

//---------------------------------------------------------------------------- SD card
// SD Card 
void writeHeader() {
  //cFile.close();
  cFile.write("HN_KB");
  cFile.print(fileNo);
  cFile.write(",");
  cFile.print(sampleRates);  //-------------------------- should comment out
  cFile.println();
}

//---------------------------------------------------------------------------- logData
void logData() {
  int z = 0;
  if (isCompleted == 0)
  {
    for (uint8_t j = 0; j < 100; j++)
    { 
      for (uint8_t i = 0; i < 8; i++)
        {
          cFile.print(data[z]);
          z++;
          cFile.write(", ");
        }
      cFile.println();
    }
    cFile.println();
    cFile.write("EOF");
    cFile.close();
    isCompleted = 1;
    fileNo++;
    Serial.println("Writing completed");
  }
}

//---------------------------------------------------------------------------- PDB set up
void pdb_init()
{
  // Set up PDB
  SIM_SCGC6 |= SIM_SCGC6_PDB; // Enable PDB clock
  PDB0_MOD = 150;             // 250Hz
  PDB0_IDLY = 0;              // Interrupt delay
  
  // Enable pre-trigger
  PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

//---------------------------------------------------------------------------- ADC set up

void adc_init()
{
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

  adcCalibrate();
  //Serial.println("calibrated");

  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | 5; //(for more information see page 655 in manual
                                // why use AD9 (value 9) as input?? neccessary??
  NVIC_ENABLE_IRQ(IRQ_ADC0);
}

//----------------------------------------------------------------------------Adc trigger
void adcCalibrate() {
  uint16_t sum;

  // Begin calibration
  ADC0_SC3 = ADC_SC3_CAL;
  // Wait for calibration
  while (ADC0_SC3 & ADC_SC3_CAL);

  // Plus side gain
  // ADC Plus-Side General Calibration Value Register
  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;

  // Minus side gain (not used in single-ended mode)
  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;
}

//----------------------------------------------------------------------------Set up
void setup() {
  //analogReadResolution(10); // set resolution
  
  //set serial & LCD
  Serial.begin(11500);  
  tft.begin();
  tft.setRotation(3);
  tft.setCursor(93,100);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(5);
  tft.print("HELLO");
  delay(1000); 
  tft.fillScreen(ILI9341_BLACK);
  
  // Set the array data to 0
  for (int i = 0; i < dataSize; i++) {
    data[i] = 0;
  }
  
  pinMode(1, INPUT_PULLUP); // Stop signal
  pinMode(2, INPUT_PULLUP); // set switch input to INPUT_PULLUP
  
  //Serial.println("PDB initial");
  pdb_init();
  //Serial.println("ADC initial");
  adc_init();
  
//------------------------------------------------------
  while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;
  randomSeed(micros());

  /* Initialise the module */
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  /* Perform a factory reset to make sure everything is in a known state */
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit HRM")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
    if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  ble.reset();
}

//----------------------------------------------------------------------------Loop
void loop() {
   // reading the val of the button
  int reading = digitalRead(1); 
  int reading2 = digitalRead(2); 

  // Display message
  if ((reading2 == LOW) && (buttonFlag1 == 1)) {
    menuState = !menuState;
    buttonFlag1 = 0;
  }
  else if (reading2 == HIGH) {
    buttonFlag1 = 1;
  }
  //Serial.println(menuState);
  if (menuState && (reading == LOW) && (flag == 1)) {
    readFile = 1;
  } 
  else {
    flag = 1;
  }
  if (readFile == 1) {
    fileNo = 1;
    increment = 1;
    isStable = 1;
    char string1[] = "log";
    char string2[] = ".txt";
    char fileName[30];
    Serial.println(fileNo);
    if (!sd.begin(CS, SPI_HALF_SPEED)) { 
      Serial.println("Err"); 
    }
    sprintf(fileName, "%s%i%s",string1, fileNo, string2);
    Serial.println(cFile.open("log1.txt", O_READ));
    Serial.println(fileName);
    delay(1000);
    //int ch;
    int i = 0;
    while ((data[i] = cFile.read()) >=0) {
      i++;
      Serial.println(data[i]);
    }
  }
  if (menuState != 0) {
    if (onMenu == 0) {
      tft.fillScreen(ILI9341_BLACK);
      menu();
      onMenu = 1;
    }
    return;
  } 
  if (onMenu == 1) {
    onMenu = 0;
    menuE();
  }
    
  interface(lastCurrentHR, lastCurrentQRS, PRI, lastCurrentRR);
  currentTime = millis();

  // Stablizing 
  if (val == 0 && isRunning != 1) {
         isRunning = 1;
         envelopeTimeStart = currentTime;
  }
  
  if (isRunning & !isStable) { 
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(25, 105);
    tft.setTextSize(3);
    tft.print("STABILIZING. . .");
    if (millis() - envelopeTimeStart > 3000) {
      isStable = 1;
    }
  }
  
  if (currentTime - startTime > 20) {
    startTime = currentTime;
    if (isStable) {  
      tft.setTextColor(ILI9341_BLACK);
      tft.setCursor(25, 105);
      tft.setTextSize(3);
      tft.print("STABILIZING. . .");
      // Button States  
      if ((reading == LOW) && (flag == 1)) { //click to stop
        state = !state;
        flag = 0;
        isStopping = 1;
      }
      else if (reading == HIGH) flag = 1;
      if (state == 1 && isStopping == 0){
        // if there is new avgVal, add it into the array data, and erasing the old trace as it draws new signal
        if (increment) {
          for (uint16_t i = dataSize - 1; i > 0; i--) {
            data[i] = data[i - 1];
          }
          data[0] = lastCurrentAvg;
          increment = 0;
          k++;
          // grid distance = 320, distance between 2 val = 2.5, 250/2.5 = 100 values
          // Shift the signal to 62.5 from y axis
          for (int i = 8; i < 108; i++) {
            tft.drawLine(tft.width() - 2.5 * i, -data[i+k] + 62.5, tft.width() - 2.5 * (i + 1), -data[i + 1 + k] + 62.5, ILI9341_BLACK); 
          }
          k = 0;
        }
        for (int i = 8; i < 108 ; i++) {   
          tft.drawLine(tft.width() - 2.5 * i, -data[i] + 62.5, tft.width() - 2.5 * (i + 1), -data[i + 1] + 62.5, ILI9341_GREEN);
        }
        // calculate heart rate
        thresholdVal(HR, limit);
        
        if (dataHB > limit && isCounting == 0) {
          //QRS Flag
          QRSflag = 1;
          startTimeHR = millis();
          isCounting = 1;
          Serial.println("startTimeHR");
        }
        else if ((millis() - startTimeHR >= 400) && isCounting == 1) {
          isCounting = 2;
        }
        else if (dataHB > limit && isCounting == 2) {
          endTimeHR = millis();
          // Calculating RR and Averaging
          RR = (endTimeHR - startTimeHR);
          //Avg RR
          if (m == 0) {
            currentRR = RR;
            m++; 
            Serial.println("HAHAHA");
          }
          else if (m < 5) {
            currentRR = avg(RR, m, lastRR);
            lastRR = currentRR;
            m++;
            if (m == 5) {
              m = 0;
              lastCurrentRR  = currentRR;
            }
          }
          //Averaging HR
          HR = (1000 / RR) * 60;
          if (n == 0) {
            currentHR = HR;
            n++; 
          }
          else if (n < 5) {
            currentHR = avg(HR, n, lastHR);
            lastHR = currentHR;
            n++;
            if (n == 5) {
              n = 0;
              lastCurrentHR  = currentHR;
            }
          }  
          isCounting = 3;
        }
        else if ((millis() - startTimeHR >= 400) && isCounting == 3) {
          isCounting = 0;
        }
        
        //QRS count
        if (QRSflag == 1) {
          QRScount++;
        }
//        Serial.println("flag");
//        Serial.println(QRSflag);
//        Serial.println(QRScount);
        if ((QRSflag == 1) && (QRScount == 15)) {
          // check value 0 - 15
          for (int i = 15; i > 0; i--) {
            if ((data[i] == 0) && (firstZero == true)) {
              Rindex = i;
              //Serial.println(i);
              break;
            }
            if (data[i] == 0) {
              firstZero = true;
            }
          }
          
          // check value 0 - 15
          firstZero = false;
          for (int i = 15; i < 30; i++) {
            if ((data[i] == 0) && (firstZero == true)) {
              Qindex = i;
              //Serial.println(i);
              break;
            }
            if (data[i] == 0) {
              firstZero = true;
            }
          } 
          if ((Qindex != -1) && (Rindex != -1)) {
             indexQRS = Qindex - Rindex;
          }
          // Set the count and flag to 0 when found indexQRS
          QRSflag = 0;
          QRScount = 0;
          
          // The QRS distance is the No of samples * 0.004ms
          QRS = indexQRS * 4;
          
          //Averaging QRS
          if (l == 0) {
            currentQRS = QRS;
            l++; 
          }
          else if (l < 5) {
            currentQRS = avg(QRS, l, lastQRS);
            lastQRS = currentQRS;
            l++;
            if (l == 5) {
              l = 0;
              lastCurrentQRS  = currentQRS;
            }
          }                 
        }
      }
      // erase the old trace when hitting the button to run
      else if (state == 1 && isStopping == 1) {
        for (int i = 0; i < dataSize - 1; i++) {
          tft.drawLine(tft.width() - 2.5 * i, data[i] + 62.5, tft.width() - 2.5 * (i + 1), data[i + 1] + 62.5, ILI9341_BLACK);
        }
        isStopping = 0;   
      }
    }
  }
  
  // Stop automatically after 30s, or when the button is hit - 30e3/4ms = 7500 val
  if ((dataNo >= 7400 && isCompleted == 0) ||  (flag == 0)) {
    digitalWrite(CS, HIGH);
    if (!sd.begin(CS, SPI_HALF_SPEED))
    {
      Serial.println("Begin Err");
    }
    char string1[] = "log";
    char string2[] = ".txt";
    char fileName[30];
    sprintf(fileName, "%s%i%s",string1, fileNo, string2);
    Serial.println(fileName); 
    if (sd.exists(fileName))
    {
       Serial.println("File exists");
       sd.remove(fileName);
    }
    
    if (!cFile.open(fileName, FILE_WRITE))
    {
      Serial.println(" Open Err");
    }

    writeHeader();
    if (!cFile.sync())
    {
      Serial.print("Sync error");
      delay(1000);
    }
    else if (cFile.getWriteError())
    {
      Serial.println("Write error");
      delay(1000);
    }
    logData();
    dataNo = 0;
    isCompleted = 0;
  }
//------------------------------------------------------------------
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",00le.println(lastCurrentHR, HEX);

  /* Check if command executed OK */
  ble.waitForOK();
//  /* Delay before next measurement update */
//  delay(1000);-") );
//  b
}

//----------------------------------------------------------------------------//Heart Rate Threshold
//Heart Rate Threshold
int thresholdVal(int HR, int limit) {
  if (millis() >= 1000){
    if (HR < 50 && limit > 8000)
      limit = limit - 100;
    else if (HR > 120 && limit < 15000)
      limit = limit + 100;
  }
  return limit;
}

//----------------------------------------------------------------------------// Moving Avg Function
// Moving Avg Function
float avg(float currentVal, int n, float lastAvg){
    float currentAvg = (lastAvg * (n-1) + currentVal) / n; 
    return currentAvg;
}

//----------------------------------------------------------------------------//Low pass filter
//Low pass filter at 8Hz 2nd order
int v0l = 0;
int v1l = 0;
int v2l;
float lowpass(int x) { //class II
  v0l = v1l;
  v1l = v2l;
  v2l = (8.826086668431319324e-3 * x)
     + (-0.75251618158180888507 * v0l)
     + (1.71721183490808360084 * v1l);
  //Serial.println((v0 + v2) + 2 * v2);
  return ((v0l + v2l) + 2 * v1l);
}

//----------------------------------------------------------------------------//High pass filter
//High pass filter
int v0h = 0;
int v1h = 0;
int v2h;
float highpass(int x) { //class II 
  v0h = v1h;
  v1h = v2h;
  v2h = (9.911535951016632318e-1 * x)
     + (-0.98238545061412485548 * v0h)
     + (1.98222892979252818257 * v1h);
  return (v0h + v2h) - 2 * v1h;
}

//----------------------------------------------------------------------------//Differentiator filter
//Differentiator filter
float diff(int currentVal)
{
  float temp;
  if (dataNo >= 4)
    temp = 0.2*currentVal + 0.1*data[0] -
              0.1*data[2] - 0.2*data[3];  
  else
    return currentVal;
  return temp;
}

//----------------------------------------------------------------------------// Display
void menu() {
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(75, 25);
  tft.setTextSize(3);
  tft.print("ECG MACHINE");
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(25, 75);
  tft.setTextSize(2);
  tft.print("Button 1: Measuring new heart rate.");

  tft.setCursor(25, 150);
  tft.setTextSize(2); 
  tft.print("Button 2: Stop measurement and write in SD card.");
}

void menuE() {
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(75, 25);
  tft.setTextSize(3);
  tft.print("ECG MACHINE");
  
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(25, 75);
  tft.setTextSize(2);
  tft.print("Button 1: Measuring new heart rate.");

  tft.setCursor(25, 150);
  tft.setTextSize(2); 
  tft.print("Button 2: Stop measurement and write in SD card.");
}
//----------------------------------------------------------------------------// Display1
// Display Arhhthmia Detection Type
void interface(uint16_t HR, float QRS, float QT, float RR) {

    // Draw the grid the first time
  // Vertical
  for (int i = 50; i < 325; i+=25) {
    tft.drawLine(i, 25, i, 100, ILI9341_RED);            
  }
  // Horizontal
  for (int i = 25; i < 125; i+=25) {
    tft.drawLine(50, i, 300, i, ILI9341_RED);           
  }

  // Time axis
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(1);
  tft.setCursor(58, 5);
  tft.print(360);
  tft.setCursor(83, 5);
  tft.print(320);
  tft.setCursor(108, 5);
  tft.print(280);
  tft.setCursor(133, 5);
  tft.print(240);
  tft.setCursor(158, 5);
  tft.print(200);
  tft.setCursor(183, 5);
  tft.print(160);
  tft.setCursor(208, 5);
  tft.print(120);
  tft.setCursor(238, 5);
  tft.print(80);
  tft.setCursor(265, 5);
  tft.print(40);
  tft.setCursor(297, 5);
  tft.print(0);
 
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(2, 5);
  tft.setTextSize(1);
  tft.print("Time(ms)");

  // Voltage Axis
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(34, 20);
  tft.setTextSize(1);
  tft.print("mV");  
  tft.setCursor(40, 59); 
  tft.print("0"); 
  tft.setCursor(40, 41); 
  tft.print("1"); 
  tft.setCursor(34, 77); 
  tft.print("-1"); 
  
  // Heart Rates Column
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(25, 145);
  tft.setTextSize(2);
  tft.print("Heart");
  tft.setCursor(25, 162);
  tft.print("Rates");
  tft.setCursor(25, 190);
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_BLACK);
  tft.print(HRold);
  tft.setCursor(25, 190);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(lastCurrentHR);
  tft.drawLine(100, 135, 100, 228, ILI9341_RED);
  HRold = lastCurrentHR;

  // Analysis Column
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(125, 145);
  tft.setTextSize(2);
  tft.print("ECG Analysis");

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(125, 175);
  tft.setTextSize(1);
  tft.print("ECG Status:");
  tft.setCursor(125, 190);
  tft.print("RR Int (ms):");
  tft.setCursor(125, 205);
  tft.print("QRS Int (ms):");
  tft.setCursor(125, 220);
  tft.print("PR Int (ms):");

  // ECG Status
  tft.setTextSize(1);
  if (lastCurrentHR == 0) {
    heartType = "Detecting...";
  }
  else if (lastCurrentHR < 60) {
    heartType = "Bradycardia";
  } 
  else if (lastCurrentHR >= 60 && lastCurrentHR <= 100) {
    heartType = "Normal";
  } 
  else if (lastCurrentHR > 100) {
    heartType = "Tachycardia";
  }
  else if (lastCurrentHR >= 60 && lastCurrentHR <= 100 && QRS >= 120) {
    heartType = "PVC";
  }
  tft.setCursor(225, 175);
  tft.setTextColor(ILI9341_BLACK);
  tft.print(heartTypeOld);
  tft.setCursor(225, 175);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(heartType);
  heartTypeOld = heartType;
  //Serial.println(heartType);
  //Serial.println(heartTypeOld);

  // RR Int
  tft.setCursor(225, 190);
  tft.setTextColor(ILI9341_BLACK);
  tft.print(RRold);
  tft.setCursor(225, 190);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(lastCurrentRR);
  RRold = lastCurrentRR;

  // QRS Int
  tft.setCursor(225, 205);
  tft.setTextColor(ILI9341_BLACK);
  tft.print(QRSold);
  tft.setCursor(225, 205);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(QRS);
  QRSold = lastCurrentQRS; 
  
  // QT Int
  tft.setCursor(225, 220);
  tft.print(PRI);
}

//----------------------------------------------------------------------------//ADC INTERRUPT
//ADC INTERRUPT
void adc0_isr()
{
  //dataNo = actualDataNo;
  // ADC0_RA - ADC Data Result Register - read value from pin A0
  rawData[dataNo] = ADC0_RA;
  
  float temp1 = lowpass(rawData[dataNo]);
  float temp2 = highpass(temp1);
  temp1 = diff(temp2);
  
  val = temp1; 

  // Value for detecting heart beat
  dataHB = val*abs(val)/10;

  // Print out value on LCD
  lastCurrentAvg = temp1/30;
  
  // grid heigh is 75
  if (lastCurrentAvg > 37.5) {
    lastCurrentAvg = 37.5;
  } 
  else if (lastCurrentAvg < -37.5) {
    lastCurrentAvg = -37.5;
  }
  increment = 1;
  //dataNo++;
}

//----------------------------------------------------------------------------// PDB INTERRUPT
// PDB INTERRUPT
void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF; // (also clears interrupt flag
}
