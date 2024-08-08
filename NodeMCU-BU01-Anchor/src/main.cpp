/*
  Anchor module. Modified version of Thomas Trojer's DW1000 library is required!
  Modes
   C - calibrate
     calibrates an ESP32_UWB module intended for use as a fixed anchor point.
     Uses binary search to find anchor antenna delay to calibrate against a known distance

     Remote tag (at origin) must be set up with default antenna delay (library default = 16384)

     User input required, possibly unique to each tag:
     1) accurately measured distance from anchor to tag
     2) address of anchor
     
     output: antenna delay parameter for use in final anchor setup.
     S. James Remington 2/20/2022
   R - run
     User input required
     1) previously calibrated anchor delay
     2) anchor_address - first byte XX in MAC address XX:00:5B:D5:A9:9A:E2:9C
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#define CONFIG_VALIDATION_KEY 214748361

struct AnchorConfig {
  int      validKey;      // Validation key. Must By equal to CONFIG_VALIDATION_KEY is config was saved
  char     mode;          // C or R (calibrate or run)
  float    calibrDist;    // Measured distance from anchor to tag in meters
  char     address[24];   // MAC address of anchor
  int      adelay;        // Antenna delay
  bool     rangeFiltered; // Use range filter to smooth data
  byte     dw1000Mode;     // DW1000 modes
                          // 1 - MODE_LONGDATA_RANGE_LOWPOWER
                          // 2 - MODE_SHORTDATA_FAST_LOWPOWER
                          // 3 - MODE_LONGDATA_FAST_LOWPOWER
                          // 4 - MODE_SHORTDATA_FAST_ACCURACY
                          // 5 - MODE_LONGDATA_FAST_ACCURACY
                          // 6 - MODE_LONGDATA_RANGE_ACCURACY
                          // 7 - MODE_LONGDATA_RANGE_ACCURACY2
  bool     dw100MaxPower; // use max available DW1000 power (can violate regulation)
};

AnchorConfig fAnchorConfig;

void loadAnchorConfig() {
  EEPROM.get(0, fAnchorConfig);  
  if (fAnchorConfig.validKey != CONFIG_VALIDATION_KEY) {
    // Default config
    fAnchorConfig.validKey = CONFIG_VALIDATION_KEY;
    fAnchorConfig.mode = 'R';
    fAnchorConfig.calibrDist = (285 - 1.75) * 0.0254;
    strcpy(fAnchorConfig.address, "84:00:5B:D5:A9:9A:E2:9C");
    fAnchorConfig.adelay = 16580;
    fAnchorConfig.rangeFiltered = false;
    fAnchorConfig.dw1000Mode = 1;
    fAnchorConfig.dw100MaxPower = false;
  }
}

void saveAnchorConfig() {
  EEPROM.put(0, fAnchorConfig);
}

void printAnchorConfig() {
  Serial1.print(" Anchor mode: ");  
  Serial1.println(fAnchorConfig.mode);
  Serial1.print(" Anchor address: ");  
  Serial1.println(fAnchorConfig.address);
  Serial1.print(" Antenna delay: ");
  Serial1.println(fAnchorConfig.adelay);
  Serial1.print(" Measured distance: ");
  Serial1.println(fAnchorConfig.calibrDist);
  Serial1.print(" Range filtered: ");
  Serial1.println(fAnchorConfig.rangeFiltered);
  Serial1.print(" DW1000 mode: ");
  Serial1.println(fAnchorConfig.dw1000Mode);
  Serial1.print(" DW1000 max power: ");
  Serial1.println(fAnchorConfig.dw100MaxPower);
}

uint16_t fAdelayDelta = 100; //initial binary search step size

bool fDoPrintDistance = false;

void newRange()
{
  if (fAnchorConfig.mode == 'R' && fDoPrintDistance) {
    // Run mode
    Serial1.println(DW1000Ranging.getDistantDevice()->getRange());
  }
  
  if (fAnchorConfig.mode == 'C' && fAdelayDelta >= 3) {
    // Calibrate mode
    static float last_delta = 0.0;
    float dist = DW1000Ranging.getDistantDevice()->getRange();
    Serial1.print("Distance: ");
    Serial1.println(dist);

    float this_delta = dist - fAnchorConfig.calibrDist;  //error in measured distance

    if (this_delta * last_delta < 0.0) {
      fAdelayDelta = fAdelayDelta / 2; //sign changed, reduce step size
    }  
    
    last_delta = this_delta;
    
    if (this_delta > 0.0) {
      fAnchorConfig.adelay += fAdelayDelta; //new trial Adelay
    } 
    else {
      fAnchorConfig.adelay -= fAdelayDelta;
    } 
    
    if (fAdelayDelta < 3) {
      saveAnchorConfig();
      Serial1.print("Final Adelay ");
    }

    Serial1.println (fAnchorConfig.adelay);

    DW1000.setAntennaDelay(fAnchorConfig.adelay);
  }
}

void newDevice(DW1000Device *device)
{
  Serial1.print("Device added: ");
  Serial1.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial1.print("Deleted inactive device: ");
  Serial1.println(device->getShortAddress(), HEX);
}

// connection pins
const uint8_t PIN_RST = PB12; // reset pin
const uint8_t PIN_IRQ = PB0;  // irq pin
const uint8_t PIN_SS = PA4;   // spi select pin

const byte *getDw1000Mode() {
  switch(fAnchorConfig.dw1000Mode) {
    case 1:
      return DW1000.MODE_LONGDATA_RANGE_LOWPOWER;
    case 2:
      return DW1000.MODE_SHORTDATA_FAST_LOWPOWER;
    case 3:  
      return DW1000.MODE_LONGDATA_FAST_LOWPOWER;
    case 4:
      return DW1000.MODE_SHORTDATA_FAST_ACCURACY;
    case 5:
      return DW1000.MODE_LONGDATA_FAST_ACCURACY;
    case 6:
      return DW1000.MODE_LONGDATA_RANGE_ACCURACY;
    case 7:
      return DW1000.MODE_LONGDATA_RANGE_ACCURACY2;
    default:
      return DW1000.MODE_LONGDATA_RANGE_LOWPOWER;
  }
}

void setup() {
  Serial1.begin(115200);
  delay(1000); // wait for serial monitor to connect
  
  // init the configuration
  SPI.begin();
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // Load anchor configuration
  loadAnchorConfig();
  printAnchorConfig();

  DW1000.setAntennaDelay(fAnchorConfig.adelay);
  DW1000._maxPower = fAnchorConfig.dw100MaxPower;
  
  DW1000Ranging.useRangeFilter(fAnchorConfig.rangeFiltered);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as anchor, don't assign random short address
  DW1000Ranging.startAsAnchor(fAnchorConfig.address, getDw1000Mode(), false);
}

void copyCharArrValue(String str, char* carray, int maxLength) {
    for (int i = 0; i < str.length() && i < maxLength; i++) { 
      carray[i] = str[i]; 
    }
}

void loop() {
  DW1000Ranging.loop();

  if (Serial1.available()) {
    String lStr = Serial1.readString();
    lStr.trim();
    
    if (lStr == "START_DISTANCE_MONITOR") {
      fDoPrintDistance = true;
      Serial1.println("Distance monitoring started...");
    }
    else if (lStr == "STOP_DISTANCE_MONITOR") {
      fDoPrintDistance = false;
      Serial1.println("Distance monitoring stopped.");
    }
    else if (lStr.startsWith("SET_RANGE_FILTERED=")) {
      const char *lStrValue = lStr.c_str() + 19;
      fAnchorConfig.rangeFiltered = (*lStrValue == 'Y');
      DW1000Ranging.useRangeFilter(fAnchorConfig.rangeFiltered);
      saveAnchorConfig();
      printAnchorConfig();
    }
    else if (lStr == "REBOOT") {
      NVIC_SystemReset();
    }
    else if (lStr.startsWith("SET_ADDRESS=")) {
      String lAddress = lStr.substring(12);
      copyCharArrValue(lStr.substring(12), fAnchorConfig.address, 24);
      saveAnchorConfig();
      printAnchorConfig();
    }
    else if (lStr.startsWith("SET_MODE=")) {
      const char *lStrValue = lStr.c_str() + 9;
      fAnchorConfig.mode = *lStrValue;
      saveAnchorConfig();
      NVIC_SystemReset();
    }
    else if (lStr.startsWith("SET_DW1000_MODE=")) {
      const char *lStrValue = lStr.c_str() + 16;
      fAnchorConfig.dw1000Mode = atoi(lStrValue);
      saveAnchorConfig();
      NVIC_SystemReset();
    }
    else if (lStr.startsWith("SET_DW1000_MAX_POWER=")) {
      const char *lStrValue = lStr.c_str() + 21;
      fAnchorConfig.dw100MaxPower = (*lStrValue == 'Y');
      saveAnchorConfig();
      NVIC_SystemReset();
    }
    else if (lStr.startsWith("SET_CALIBR_DIST=")) {
      const char *lStrValue = lStr.c_str() + 16;
      float lValue = atof(lStrValue);
      fAnchorConfig.calibrDist = lValue;
      saveAnchorConfig();
      printAnchorConfig();
    }
    else if (lStr.startsWith("SET_ANTENNA_DELAY=")) {
      const char *lStrValue = lStr.c_str() + 18;
      int lValue = atoi(lStrValue);
      fAnchorConfig.adelay = lValue;
      saveAnchorConfig();
      printAnchorConfig();
    }
  }
}
