
#include <mcp_can.h>
#include <SPI.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <LEDMatrixDriver.hpp>

#define rxPin 14
#define txPin 15

//#define DEBUG 1
#define ENCODER_USE_INTERRUPTS 1
#define ENCODER_OPTIMIZE_INTERRUPTS 1

// Flatpack2 Configuration
#define MAXTEMP 80
#define MAXVOLT 57.5
#define MINVOLT 43.5
#define MAXCUR  37.4
#define MINCUR  1.0
#define DELTA 0.23 // Voltage error correction for Vlad's fp2

// RGB LED pins
#define REDpin 4
#define GREENpin 7
#define BLUEpin 9


/*
 *  Second RS232
 */
// Set up a new SoftwareSerial object
SoftwareSerial rs232 =  SoftwareSerial(rxPin, txPin);

/*
 *  CAN BUS Interface
 */
const int CAN_CS_PIN = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(CAN_CS_PIN);


/*
 *  ENCODER
 */
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   NOTE: Avoid using pins with LEDs attached
Encoder Enc(3, 6);

const uint8_t buttonPIN = 5; // Button to select voltage and current
long oldPosition  = -999;


/*
 *  LED DRIVER
 */
// Define the ChipSelect pin for the led matrix (Dont use the SS or MISO pin of your Arduino!)
const uint8_t LEDMATRIX_CS_PIN = 8;
const int NO_OF_DRIVERS = 2; // Each MAX7219 driver can drive eight 7-segment displays.
// The LEDMatrixDriver class instance
LEDMatrixDriver lmd(NO_OF_DRIVERS, LEDMATRIX_CS_PIN);


/*
 *  Eltec Flatpack2 config
 */
const char *alarms0Strings[] = {"OVS_LOCK_OUT", "MOD_FAIL_PRIMARY", "MOD_FAIL_SECONDARY", "HIGH_MAINS", "LOW_MAINS", "HIGH_TEMP", "LOW_TEMP", "CURRENT_LIMIT"};
const char *alarms1Strings[] = {"INTERNAL_VOLTAGE", "MODULE_FAIL", "MOD_FAIL_SECONDARY", "FAN1_SPEED_LOW", "FAN2_SPEED_LOW", "SUB_MOD1_FAIL", "FAN3_SPEED_LOW", "INNER_VOLT"};

bool serialNumberReceived = false;
uint8_t serialNumber[6];
uint8_t setParams[8];
uint8_t dfltParams[5] = {0x29, 0x15, 0x00, 0xD2, 0x14}; // Set rectifier permanently to 53.30 (14 D2) (actually its ts 53.5 (53.30+DELTA for correction)


// Timing constants
const uint32_t timePeriod100ms = 100; // Delay 100ms
const uint32_t timePeriod1s = 1000; // Delay, 1s
const uint32_t timePeriod3s = 3000; // Delay 3s
const uint32_t timePeriod5s = 5000; // Delay, 5s
const uint32_t timePeriod30s = 30000; // Delay 30s

uint8_t flag = 0;
bool dflt = true;
unsigned long lastLogInTime = 0;


/*
 *  RGB Led
 */
void setColor(int redValue, int greenValue,  int blueValue) {
  analogWrite(REDpin, redValue);
  analogWrite(GREENpin,  greenValue);
  analogWrite(BLUEpin, blueValue);
}


/*
 *  -------------------- SETUP --------------------
 */
void setup() {
  bool flip = true;

  //Defining the pins as OUTPUT for the RGB Led
  pinMode(REDpin,  OUTPUT);              
  pinMode(GREENpin, OUTPUT);
  pinMode(BLUEpin, OUTPUT);

//#ifdef DEBUG  
  Serial.begin(115200);
  while (!Serial);
//#endif

  // Define pin modes for Serial TX and RX
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
    
  // Set the baud rate for the SoftwareSerial object and wait for it to be ready
  rs232.begin(115200);
  /*
  while (rs232.isListening()) {
    if(flip) {
      setColor(0, 0, 255);  // BLUE;
      delay(300);
    }  
    flip = !flip;
  } 
  */ 
  
 /* 
  * Init CAN BUS
  */
  pinMode(CAN_CS_PIN, OUTPUT);
  pinMode(CAN_INT_PIN, INPUT);

START_INIT:
  if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ) == CAN_OK) {
    setColor(0, 255, 0);  // GREEN
    rs232.println("MCP2515 initialized successfully.");
#ifdef DEBUG    
    Serial.println("MCP2515 initialized successfully.");
#endif    
  } else {
    setColor(255, 0, 0);  // RED
    rs232.println("Failed to initialize MCP2515. Halting.");
#ifdef DEBUG    
    Serial.println("Failed to initialize MCP2515. Halting.");
#endif    
    delay(100);
    goto START_INIT;
  }

  CAN.setMode(MCP_NORMAL);

  /* 
   * Init the Display
   */  
  lmd.setEnabled(true);
  lmd.setIntensity(2);  // 0 = min, 15 = max
  lmd.setScanLimit(7);  // 0-7: Show 1-8 digits. Beware of currenct restrictions for 1-3 digits! See datasheet.
  lmd.setDecode(0xFF);  // Enable "BCD Type B" decoding for all digits.
  // Show "empty" screen
  for(int j=0; j<5; j++) {
    if (j%2) {
      for(int i=15; i>=0; i--)  
        lmd.setDigit(i, LEDMatrixDriver::BCD_DASH, false);
    } else {    
      for(int i=15; i>=0; i--)  
        lmd.setDigit(i, LEDMatrixDriver::BCD_BLANK, true);
    }    
    lmd.display();
    delay(500);
  }
}  

/*
 *  Print CAN BUS messages
 */
void printMessage(uint32_t rxID, uint8_t len, uint8_t rxBuf[]) {
  char output[256];

  snprintf(output, 256, "ID: 0x%.8lX Length: %1d Data:", rxID, len);
  Serial.print(output);

  for (int i = 0; i < len; ++i) {
    snprintf(output, 256, " 0x%.2X", rxBuf[i]);
    Serial.print(output);
  }

  Serial.println();
}


/*
 * CAN BUS Log in to FlatPack2
 */
void logIn() {
#ifdef DEBUG
  Serial.println("Logging in.");
#endif
  uint8_t txBuf[8] = { 0 };
  for (int i = 0; i < 6; ++i) {
    txBuf[i] = serialNumber[i];
  }
  CAN.sendMsgBuf(0x05004804, 1, 8, txBuf);
}


/*
 * CAN BUS - get serial number for the Flatpack2
 */
void processLogInRequest(uint32_t rxID, uint8_t len, uint8_t rxBuf[]) {
#ifdef DEBUG  
  Serial.println("--------");
  Serial.print("Found power supply ");
#endif  

  char output[3];
  int j = 7;

  rs232.print("FP2 Serial #: ");
  for (int i = 0; i < 6; ++i) {
    serialNumber[i] = rxBuf[i + 1];
    snprintf(output, 3, "%.2X", serialNumber[i]);
    rs232.print(output);
#ifdef DEBUG       
    if(i<4) { // Show Serial
      int n = atoi(&output[0]);
      lmd.setDigit(j--, n/10); 
      lmd.setDigit(j--, n%10); 
      lmd.display();
    }  
#endif       
  }
  rs232.println();
  serialNumberReceived = true;
}


/**************************************************************
* Set the Output Current                                      *
*/  
float selectCurrent(float current)
{
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 100;   // the debounce time; increase if the output flickers
  int buttonState;            // the current reading from the input pin
  int lastButtonState = LOW;  // the previous reading from the input pin

  char output[256];
  float c;
  long newPosition = 0;
  uint8_t esc = 0;

  if((c = current) < 1.0)
    c = MAXCUR;
  
  while (digitalRead(buttonPIN) == HIGH); // Waiting for the knob to be released
  
  do {
    
    for(int j=7; j>3; j--) {
      lmd.setDigit(j, LEDMatrixDriver::BCD_BLANK, false); // Dimm the other LED segments
    }
    dtostrf(c, 3, 2, output);
    int c1 = atoi(&output[0]);
    int c2 = atoi(&output[2]);
    lmd.setDigit(3, LEDMatrixDriver::BCD_BLANK, false);
    if(c >= 10.0)
      lmd.setDigit(2, (c1/10), false);
    else  
      lmd.setDigit(2, LEDMatrixDriver::BCD_BLANK, false);
    lmd.setDigit(1, (c1%10), true);
    lmd.setDigit(0, (c2/10), false);
    lmd.display();
      
    newPosition = Enc.read();
    if (newPosition != oldPosition) {
      if(newPosition > oldPosition) {
        c += 0.1;
        if(c > MAXCUR) c = MAXCUR;
      } else {
        c -= 0.1;
        if(c < MINCUR) c = MINCUR;
      } 
      oldPosition = newPosition;
      delay(50);
#ifdef DEBUG    
      Serial.println(c);
#endif
    }
    
    int reading = digitalRead(buttonPIN);
    if(reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;
        if (buttonState == HIGH) {
          esc++;
        }
      }
    }
    lastButtonState = reading;
    if(esc > 1)
      break;
  } while (1);

  flag = 0; // Reset flag

  if(c>0)
    return(c);
  else
    return(MAXCUR);
}


/**************************************************************
* Set the Output Voltage                                      *
*  
**************************************************************/
float selectVoltage(float outputVoltage)
{
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 100;   // the debounce time; increase if the output flickers
  int buttonState;            // the current reading from the input pin
  int lastButtonState = LOW;  // the previous reading from the input pin

  char output[256];
  float v;
  long newPosition = 0;
  uint8_t esc = 0;

  v = outputVoltage;
  
  while (digitalRead(buttonPIN) == HIGH); // Waiting for the knob to be released
  
  do {
    
    for(int j=0; j<4; j++) {
     lmd.setDigit(j, LEDMatrixDriver::BCD_BLANK, false); // Dimm the other LED segments
    }
    dtostrf(v, 4, 2, output);
    int v1 = atoi(&output[0]);
    int v2 = atoi(&output[3]);  v2 = (v2-(v2%10)); // Remove precision
    lmd.setDigit(7, (v1/10), false);
    lmd.setDigit(6, (v1%10), true);
    lmd.setDigit(5, (v2/10), false);
    lmd.setDigit(4, (v2%10), false);
    lmd.display();
      
    newPosition = Enc.read();
    if (newPosition != oldPosition) {
      if(newPosition > oldPosition) {
        v += 0.1;
        if(v > MAXVOLT) v = MAXVOLT;
      } else {
        v -= 0.1;
        if(v < MINVOLT) v = MINVOLT; 
      } 
      oldPosition = newPosition;
      delay(50);
#ifdef DEBUG    
      Serial.println(v);
#endif
    }
    
    int reading = digitalRead(buttonPIN);
    if(reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;
        if (buttonState == HIGH) {
          esc++;
        }
      }
    }
    lastButtonState = reading;
    if(esc > 1)
      break;
  } while (1);
  
  return(v-DELTA);
}


/*
 *  Set Flatpack2 params
 * 
unsigned char stmp2[8] = {0xA0, 0x00, 0x80, 0x16, 0x80, 0x16, 0x3E, 0x17};    //set rectifier to 16.0 amps (00 A0) 57.60 (16 80) and OVP to 59.5V (17 3E)
CAN.sendMsgBuf(0x05FF4004, 1, 8, stmp2); 
 
So one login followed by repeated requests with less then 5s interval is enough. For robustness I send both login and request every time.
If communication is lost the FP reverts back to the default voltage.

This default voltage can be set by sending first login and then a message with ID: 0x05009C00.
Byte 0 = 0x29, Byte 1 = 0x15, Byte 2= 0x00.

For 43.7V translate 4370 to HEX => 0x1112, separate into 2 bytes and place as byte 3 & 4.

byte stmp2[5] = {0x29, 0x15, 0x00, 0x12, 0x11};                   //set rectifier permanently to 43.70 (11 12)
CAN.sendMsgBuf(0x05009C00, 1, 5, stmp2); 
*/
bool setPower(float v, float c)
{
  dflt = false;
  if (serialNumberReceived) {
    unsigned int MAXVOLTAGE = MAXVOLT;
    unsigned int VOLTAGE = v*100;
    unsigned int CURRENT = c*100;
    
    setParams[0] = CURRENT & 0xFF; setParams[1] = (CURRENT >> 8) & 0xFF; // Set Current
    setParams[2] = VOLTAGE & 0xFF; setParams[3] = (VOLTAGE >> 8) & 0xFF; // Set initial Voltage
    setParams[4] = VOLTAGE & 0xFF; setParams[5] = (VOLTAGE >> 8) & 0xFF; // Set Final Voltage
    setParams[6] = MAXVOLTAGE & 0xFF; setParams[7] = (MAXVOLTAGE >> 8) & 0xFF; // Set OVP (MAX) Voltage

    logIn();
    delay(80);
    CAN.sendMsgBuf(0x05FF4004, 1, 8, setParams);
  }  
  return(dflt);
}


/*
 *  CAN BUS: Receive and Publish Flatpack2 Status Messages
 */

void processStatusMessage(uint32_t rxID, uint8_t len, uint8_t rxBuf[]) {
  int intakeTemperature = rxBuf[0];
  float current = 0.1f * (rxBuf[1] | (rxBuf[2] << 8));
  float outputVoltage = DELTA + (0.01f * (rxBuf[3] | (rxBuf[4] << 8)));
  int inputVoltage = rxBuf[5] | (rxBuf[6] << 8);
  int outputTemperature = rxBuf[7];

  float v, c;
  if (flag) {
    v = selectVoltage(outputVoltage);
    c = selectCurrent(current);
    setPower(v, c);
    return;
  }  

  char output[256];
  
  // Voltage
  if(outputVoltage > MINVOLT && outputVoltage < MAXVOLT) {
    dtostrf(outputVoltage, 4, 2, output);
    int v1 = atoi(&output[0]); //Serial.print(v1/10); Serial.print(v1%10);
    int v2 = atoi(&output[3]); //Serial.print(v2/10); Serial.print(v2%10);
    lmd.setDigit(7, (v1/10), false);
    lmd.setDigit(6, (v1%10), true);
    lmd.setDigit(5, (v2/10), false);
    lmd.setDigit(4, (v2%10), false);
    lmd.display();
  }  

  // Current
  if(current <= MAXCUR) {
    dtostrf(current, 4, 1, output);
    int c1 = atoi(&output[0]); //Serial.print(c1/10); Serial.print(c1%10);
    int c2 = atoi(&output[2]); //Serial.print(c2/10); Serial.print(c2%10);
    lmd.setDigit(0, (c1/10), false);
    lmd.setDigit(1, (c1%10), true);
    lmd.setDigit(2, (c2/10), false);
    lmd.setDigit(3, LEDMatrixDriver::BCD_BLANK, false);
    lmd.display();
  }  

  // Temperature
  if(intakeTemperature > 1 && intakeTemperature < MAXTEMP) {
    int t1 = intakeTemperature/10;
    int t2 = intakeTemperature%10;
    lmd.setDigit(15, t1, false);
    lmd.setDigit(14, t2, true);
    lmd.setDigit(13, LEDMatrixDriver::BCD_BLANK, false);
    lmd.setDigit(12, LEDMatrixDriver::BCD_BLANK, false);
    lmd.display();
  }  
  if(outputTemperature > 1 && outputTemperature < MAXTEMP) {
    int t1 = outputTemperature/10;
    int t2 = outputTemperature%10;
    lmd.setDigit(11, LEDMatrixDriver::BCD_BLANK, false);
    lmd.setDigit(10, LEDMatrixDriver::BCD_BLANK, false);
    lmd.setDigit(9, t1, false);
    lmd.setDigit(8, t2, true);
    lmd.display();
  }  
  
#ifdef DEBUG  
  Serial.println("--------");
  Serial.println("Status message:");

  Serial.print("Intake temperature: ");
  Serial.print(intakeTemperature);
  Serial.println(" deg C");

  Serial.print("Current: ");
  Serial.print(current);
  Serial.println("A");

  Serial.print("Output voltage: ");
  Serial.print(outputVoltage);
  Serial.println("V");

  Serial.print("Input voltage: ");
  Serial.print(inputVoltage);
  Serial.println("V");

  Serial.print("Output temperature: ");
  Serial.print(outputTemperature);
  Serial.println(" deg C");
#endif

  if (rxID == 0x05014010) {
    rs232.println("Currently in walk in (voltage ramping up)");
  }

  bool hasWarning = rxID == 0x05014008;
  bool hasAlarm = rxID == 0x0501400C;


  if (hasWarning) {
    setColor(255, 255, 0); // YELLOW
    rs232.println("WARNING");
  } else if (hasAlarm) {
    setColor(255, 0, 0);  // RED
    rs232.println("ALARM");
  } else
    setColor(0, 255, 0);  // GREEN

  if (hasWarning || hasAlarm) {
    uint8_t txBuf[3] = {0x08, hasWarning ? 0x04 : 0x08, 0x00};
    CAN.sendMsgBuf(0x0501BFFC, 1, 3, txBuf);
  }
#ifdef DEBUG
  Serial.println("--------");
#endif  
}


/*
 *  ALARMS for Flatpack2
 */
void processWarningOrAlarmMessage(uint32_t rxID, uint8_t len, uint8_t rxBuf[]) {
  bool isWarning = rxBuf[1] == 0x04;

  if (isWarning) {
    setColor(255, 255, 0); // YELLOW
    rs232.print("Warnings:");
  } else {
    setColor(255, 0, 0);  // RED
    rs232.print("Alarms:");
  }

  uint8_t alarms0 = rxBuf[3];
  uint8_t alarms1 = rxBuf[4];

  for (int i = 0; i < 8; ++i) {
    if (alarms0 & (1 << i)) {
      rs232.print(" ");
      rs232.print(alarms0Strings[i]);
    }

    if (alarms1 & (1 << i)) {
      rs232.print(" ");
      rs232.print(alarms1Strings[i]);
    }
  }
  rs232.println();
  rs232.println("--------");;
}


/*
 *  -------------------- LOOP ----------------------
 */

void loop() {
  /*
   * EMCODEDR Button
   */
  static uint32_t lastMillisButton = millis(); 
  static uint8_t buttonPressed = 0;
  if(digitalRead(buttonPIN) == HIGH) {
    lastMillisButton = millis(); // reset timer
    buttonPressed &= B110;
  }
  else if((millis() - lastMillisButton) > timePeriod100ms){ // Button pressed for 100ms
    buttonPressed |= B011;
  }

  if((buttonPressed == B011) && (millis() - lastMillisButton > timePeriod3s)) { // Button pressed for 3s -> select voltage/current
    flag = 1;
  }

  /*
   * CAN BUS Flatpack2
   */
  // Log in two seconds
  if (serialNumberReceived && millis() - lastLogInTime > 2000) {
    logIn();
    if(dflt) {
      delay(80);
      CAN.sendMsgBuf(0x05009C00, 1, 5, dfltParams); // Once turned on - set default (permanent voltage)
      dflt = false;
    }  
    lastLogInTime = millis();
  }

  // Read the CAN Bus (Active low)
  if (!digitalRead(CAN_INT_PIN)) {
    uint32_t rxID;
    uint8_t len = 0;
    uint8_t rxBuf[8];

    CAN.readMsgBuf((unsigned long *)&rxID, &len, rxBuf);

    // Limit ID to lowest 29 bits (extended CAN)
    rxID &= 0x1FFFFFFF;
#ifdef DEBUG
    printMessage(rxID, len, rxBuf);
#endif    

    if (!serialNumberReceived && (rxID & 0xFFFF0000) == 0x05000000) {
      processLogInRequest(rxID, len, rxBuf);
    } else if ((rxID & 0xFFFFFF00) == 0x05014000) {
      processStatusMessage(rxID, len, rxBuf);
    } else if (rxID == 0x0501BFFC) {
      processWarningOrAlarmMessage(rxID, len, rxBuf);
    }
  }
}
