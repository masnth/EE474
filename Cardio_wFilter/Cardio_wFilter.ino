
  // Consider multiply the averaging value to have nice peak



/*
 * Huy Nguyen - Khe Bach
 * EE 474
 * Winter 2017
 * Lab 7
 */

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

int samples = 0;
const int CS = 4;
int sampleRates = 250;
int fileNo = 1;
int isCompleted = 0;

// Array of data
int dataSize = 7400;
uint16_t data[7400];
uint16_t rawData[7400];
int dataNo = 0;

// Moving Avg
float currentAvg;
float lastAvg;
float lastCurrentAvg;
int n;

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
int flag = 1; // flag for control the state
int state = 1; // state on or off
int isStopping = 0;
int k = 0;

//----------------------------------------------------------------------------
// SD Card 
void writeHeader() {
  //cFile.close();
  cFile.write("HN_KB");
  cFile.print(fileNo);
  cFile.write(",");
  cFile.print(sampleRates);
  cFile.println();
}

//----------------------------------------------------------------------------
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
    Serial.println("Writing completed");
  }
}

//----------------------------------------------------------------------------
// Using filter
float testFrequencyLow = 25; // High of 180 beats
float testFrequencyHigh = 0.5; // Low of 30 beats

//float filter(float temp) {
//  filter1Type *filter1;
//  filter1_init(filter1);
//  temp = filter1->output;
//  return temp;
//}

//----------------------------------------------------------------------------
void setup() {
  //analogReadResolution(10); // set resolution
  
  //set serial & LCD
  Serial.begin(11500);  
  tft.begin();
  tft.setRotation(1);
  tft.setCursor(93,100);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(5);
  tft.print("HELLO");
  delay(1000); 
  tft.fillScreen(ILI9341_WHITE);
  
  // Set the array data to 0
  for (int i = 0; i < dataSize; i++) {
    data[i] = 0;
  }

  // Draw the grid the first time
  for (int i = 0; i < tft.width(); i+=25) {
    tft.drawLine(i, tft.height(),  i, 0, ILI9341_RED);            //y
    tft.drawLine(tft.width(), i, 0, i, ILI9341_RED);              //x
  }
  for (int i = 0; i < tft.width(); i+=75) {
    tft.drawLine(i+1, tft.height(),  i+1, 0, ILI9341_RED);            //y
    tft.drawLine(tft.width(), i+1, 0, i+1, ILI9341_RED);              //x
  }
  
  pinMode(1, INPUT_PULLUP); // set switch input to INPUT_PULLUP
  
  //Serial.println("PDB initial");
  pdb_init();
  //Serial.println("ADC initial");
  adc_init();
}

//----------------------------------------------------------------------------
//pdb set up
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

//----------------------------------------------------------------------------
// set up ADC
void adc_init()
{
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

  adcCalibrate();
  Serial.println("calibrated");

  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | 5; //(for more information see page 655 in manual
                                // why use AD9 (value 9) as input?? neccessary??
  NVIC_ENABLE_IRQ(IRQ_ADC0);
}

//----------------------------------------------------------------------------
//Adc trigger
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


//----------------------------------------------------------------------------
void loop() {
  
  //Serial.println(dataNo);
  int reading = digitalRead(1); // reading the val of the button
  
  currentTime = millis();

  // Stablizing 
  if (val > 900 && isRunning != 1) {
         isRunning = 1;
         envelopeTimeStart = currentTime;
  }
  
  if (isRunning & !isStable) {
      if (millis() - envelopeTimeStart > 2000) isStable = 1;
  }
  
  if (currentTime - startTime > 20) {
    startTime = currentTime;
    
    if (isStable) {  
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
          for (uint8_t i = dataSize - 1; i > 0; i--) {
            data[i] = data[i - 1];
          }
          data[0] = lastCurrentAvg;
          
          increment = 0; 
          k++;
          // tft.width = 320, distance between 2 val = 2.5
          // 320/2.5 = 128 values
          for (int i = 0; i < 128 ; i++) {
            tft.drawLine(tft.width() - 2.5 * i, data[i+k]/5 + tft.height()/10, tft.width() - 2.5 * (i + 1), data[i + 1 + k]/5 + tft.height()/10, ILI9341_WHITE); 
          }
          k = 0;
        }
        for (int i = 0; i < tft.width(); i+=25) {
          tft.drawLine(i, tft.height(),  i, 0, ILI9341_RED);           //y
          tft.drawLine(tft.width(), i, 0, i, ILI9341_RED);            //x
        } 
        for (int i = 0; i < tft.width(); i+=75) {
          tft.drawLine(i+1, tft.height(),  i+1, 0, ILI9341_RED);            //y
          tft.drawLine(tft.width(), i+1, 0, i+1, ILI9341_RED);              //x
         
        }
        for (int i = 0; i < 128; i++) {   
          tft.drawLine(tft.width() - 2.5 * i, data[i]/5 + tft.height()/10, tft.width() - 2.5 * (i + 1), data[i + 1]/5 + tft.height()/10, ILI9341_BLACK);
        }
      }
      // erase the old trace when hitting the button to run
      else if (state == 1 && isStopping == 1) {
        for (int i = 0; i < dataSize - 1; i++) {
          tft.drawLine(tft.width() - 5 * i, data[i]/5 + tft.height()/10, tft.width() - 5 * (i + 1), data[i + 1]/5 + tft.height()/10, ILI9341_WHITE);
        }
        isStopping = 0;   
        //Serial.println("HHAHHAHAHAH");     
      }
    }
  }
  // Stop automatically after 30s, or when the button is hit 30e3/4ms = 7500 val
  if ((dataNo >= 15000 && isCompleted == 0) ||  (flag == 0)) // Originally dataNo >= 7400
  {
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
}

//----------------------------------------------------------------------------
// Moving Avg Function
float avg(float currentVal, int n, float lastAvg){
    float currentAvg = (lastAvg*(n-1) + currentVal - lastAvg*(n-1)/n)/n; 
    return currentAvg;
}

//----------------------------------------------------------------------------
//Low pass filter
uint16_t lowpass(uint16_t rawData[], uint16_t dataNo)
{
  uint16_t temp;
  if (dataNo >= 12)
    temp = 2*data[dataNo-1] - data[dataNo - 2] + rawData[dataNo] -
                  2*rawData[dataNo-6] + rawData[dataNo-12];  
  else
    temp = rawData[dataNo];
  return temp;
}

//----------------------------------------------------------------------------
//High pass filter
uint16_t highpass(uint16_t rawData[], uint16_t dataNo)
{
  uint16_t temp;
  if (dataNo >= 32)
    temp = -data[dataNo-1] + rawData[dataNo] -
                  32*rawData[dataNo-16] + rawData[dataNo-32];  
  else
    temp = rawData[dataNo];
  return temp;
}

//----------------------------------------------------------------------------
//Differentiator filter
uint16_t diff(uint16_t rawData[], uint16_t dataNo)
{
  uint16_t temp;
  if (dataNo >= 32)
    temp = 0.2*rawData[dataNo] + 0.1*rawData[dataNo-1] -
              0.1*rawData[dataNo-3] - 0.2*rawData[dataNo-4];  
  else
    temp = rawData[dataNo];
  return temp;
}


//----------------------------------------------------------------------------
//ADC INTERRUPT
void adc0_isr()
{
  // ADC0_RA - ADC Data Result Register
  //val = ADC0_RA/4; // read value from pin A0
  
  float temp = ADC0_RA/4;
  //val = temp; 
  //val = filter(temp);
  // Calculate moving avg if there is new data
  if (n == 0) {
    currentAvg = val;
    n++; 
  }
  else if (n < 4) {
    currentAvg = avg(val, n, lastAvg);
    lastAvg = currentAvg;
    n++;
  }
  else {
    n = 0;
    lastCurrentAvg = currentAvg;
  }
  increment = 1;
  dataNo++;
}

// PDB INTERRUPT
void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF; // (also clears interrupt flag
}

