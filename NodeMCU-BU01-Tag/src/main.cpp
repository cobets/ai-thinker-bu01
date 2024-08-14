/* 
  Tag module
  Modes
    C - sets the tag address and antenna delay to default, this mode used for calibrating the anchors.
    R - run mode
        this version is 2D (X,Y) only, 4 or more anchors (overdetermined linear least square solution)
        The Z coordinates of anchors and tag are all assumed to be zero, so for highest accuracy
        the anchors should be approximately in the same horizontal plane as the tag.
        This code does not average position measurements!
  
  S. James Remington 1/2022
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include "ardupilot.h"

// #define TAG_DEBUG

#define CONFIG_VALIDATION_KEY 214748360
#define DEFAULT_ADDRESS "7D:00:22:EA:82:60:3B:9C"
// #define DEFAULT_ANTENNA_DELAY 16384
#define N_ANCHORS 4

struct TagConfig {
  int      validKey;              // Validation key. Must By equal to CONFIG_VALIDATION_KEY is config was saved
  char     mode;                  // C or R (calibrate or run)
  char     address[24];           // MAC address of anchor
  float    aMatrix[N_ANCHORS][3]; // Anchors coordinates matrix [0,1,2,3][x,y,z]
                                  // Z values are ignored in this code, except to compute RMS distance error
  byte      dw1000Mode;           // DW1000 modes
                                  // 1 - MODE_LONGDATA_RANGE_LOWPOWER
                                  // 2 - MODE_SHORTDATA_FAST_LOWPOWER
                                  // 3 - MODE_LONGDATA_FAST_LOWPOWER
                                  // 4 - MODE_SHORTDATA_FAST_ACCURACY
                                  // 5 - MODE_LONGDATA_FAST_ACCURACY
                                  // 6 - MODE_LONGDATA_RANGE_ACCURACY
                                  // 7 - MODE_LONGDATA_RANGE_ACCURACY2
  bool     dw100MaxPower; // use max available DW1000 power (can violate regulation)
};

TagConfig fConfig;

void loadTagConfig() {
  EEPROM.get(0, fConfig);
  if (fConfig.validKey != CONFIG_VALIDATION_KEY) {
    // Default config
    fConfig.validKey = CONFIG_VALIDATION_KEY;
    fConfig.mode = 'R';
    strcpy(fConfig.address, DEFAULT_ADDRESS);
    fConfig.dw1000Mode = 1;
    fConfig.aMatrix[0][0] = 0.0;  fConfig.aMatrix[0][1] = 0.0;  fConfig.aMatrix[0][2] = 0.97;
    fConfig.aMatrix[1][0] = 3.99; fConfig.aMatrix[1][1] = 5.44; fConfig.aMatrix[1][2] = 1.14;
    fConfig.aMatrix[2][0] = 3.71; fConfig.aMatrix[2][1] = -0.3; fConfig.aMatrix[2][2] = 0.6;
    fConfig.aMatrix[3][0] = 0.56; fConfig.aMatrix[3][1] = 4.88; fConfig.aMatrix[3][2] = 0.14;
    fConfig.dw100MaxPower = false;
  }
}

void saveTagConfig() {
  EEPROM.put(0, fConfig);
}

void printTagConfig() {
#ifndef TAG_DEBUG
  Serial1.print(" Tag mode: ");  
  Serial1.println(fConfig.mode);
  Serial1.print(" Tag address: ");  
  Serial1.println(fConfig.address);
  Serial1.print(" DW1000 mode: ");
  Serial1.println(fConfig.dw1000Mode);
  Serial1.print(" DW1000 max power: ");
  Serial1.println(fConfig.dw100MaxPower);

  Serial1.println(" Anchor coordinate matrix: ");
  
  for (int i = 0; i < N_ANCHORS; i++) {
    for (int j = 0; j <= 2; j++) {
      Serial1.print(fConfig.aMatrix[i][j]);
      Serial1.print(", ");
    }
    Serial1.println();
  }
#endif  
}

// connection pins
const uint8_t PIN_RST = PB12; // reset pin
const uint8_t PIN_IRQ = PB0;  // irq pin
const uint8_t PIN_SS = PA4;   // spi select pin

// variables for position determination
#define ANCHOR_DISTANCE_EXPIRED 5000   //measurements older than this are ignore (milliseconds) 

uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports

float current_tag_position[2] = {0.0, 0.0}; //global current position (meters with respect to anchor origin)
float current_distance_rmse = 0.0;  //rms error in distance calc => crude measure of position error (meters).  Needs to be better characterized

int trilat2D_4A(void) {

  // for method see technical paper at
  // https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf
  // S. James Remington 1/2022
  //
  // A nice feature of this method is that the normal matrix depends only on the anchor arrangement
  // and needs to be inverted only once. Hence, the position calculation should be robust.
  //
  static bool first = true;  //first time through, some preliminary work
  float d[N_ANCHORS]; //temp vector, distances from anchors

  static float A[N_ANCHORS - 1][2], Ainv[2][2], b[N_ANCHORS - 1], kv[N_ANCHORS]; //calculated in first call, used in later calls

  int i, j, k;
  // copy distances to local storage
  for (i = 0; i < N_ANCHORS; i++) d[i] = last_anchor_distance[i];

  if (first) {  //intermediate fixed vectors
    first = false;

    float x[N_ANCHORS], y[N_ANCHORS]; //intermediate vectors

    for (i = 0; i < N_ANCHORS; i++) {
      x[i] = fConfig.aMatrix[i][0];
      y[i] = fConfig.aMatrix[i][1];
      kv[i] = x[i] * x[i] + y[i] * y[i];
    }

    // set up least squares equation

    for (i = 1; i < N_ANCHORS; i++) {
      A[i - 1][0] = x[i] - x[0];
      A[i - 1][1] = y[i] - y[0];
    }

    float ATA[2][2];  //calculate A transpose A
    // Cij = sum(k) (Aki*Akj)
    for (i = 0; i < 2; i++) {
      for (j = 0; j < 2; j++) {
        ATA[i][j] = 0.0;
        for (k = 0; k < N_ANCHORS - 1; k++) ATA[i][j] += A[k][i] * A[k][j];
      }
    }

    //invert ATA
    float det = ATA[0][0] * ATA[1][1] - ATA[1][0] * ATA[0][1];
    if (fabs(det) < 1.0E-4) {
      Serial1.println("***Singular matrix, check anchor coordinates***");
      while (1) delay(1); //hang
    }

    det = 1.0 / det;
    //scale adjoint
    Ainv[0][0] =  det * ATA[1][1];
    Ainv[0][1] = -det * ATA[0][1];
    Ainv[1][0] = -det * ATA[1][0];
    Ainv[1][1] =  det * ATA[0][0];
  } //end if (first);

  //least squares solution for position
  //solve:  (x,y) = 0.5*(Ainv AT b)

  for (i = 1; i < N_ANCHORS; i++) {
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + kv[i] - kv[0];
  }

  float ATb[2] = {0.0}; //A transpose b
  for (i = 0; i < N_ANCHORS - 1; i++) {
    ATb[0] += A[i][0] * b[i];
    ATb[1] += A[i][1] * b[i];
  }

  current_tag_position[0] = 0.5 * (Ainv[0][0] * ATb[0] + Ainv[0][1] * ATb[1]);
  current_tag_position[1] = 0.5 * (Ainv[1][0] * ATb[0] + Ainv[1][1] * ATb[1]);

  // calculate rms error for distances
  float rmse = 0.0, dc0 = 0.0, dc1 = 0.0, dc2 = 0.0;
  for (i = 0; i < N_ANCHORS; i++) {
    dc0 = current_tag_position[0] - fConfig.aMatrix[i][0];
    dc1 = current_tag_position[1] - fConfig.aMatrix[i][1];
    dc2 = fConfig.aMatrix[i][2]; //include known Z coordinate of anchor
    dc0 = d[i] - sqrt(dc0 * dc0 + dc1 * dc1 + dc2 * dc2);
    rmse += dc0 * dc0;
  }
  current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS));

  ap_send_vehicle_position(current_tag_position, current_distance_rmse);

  return 1;
} //end trilat2D_3A

void newRangeRun()
{
  // collect distance data from anchors, presently configured for 4 anchors
  // solve for position if all four current

  int i;  //index of this anchor, expecting values 1 to 7
  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x07;

  if (index > 0) {
    last_anchor_update[index - 1] = millis();  //decrement index for array index
    float range = DW1000Ranging.getDistantDevice()->getRange();
    last_anchor_distance[index - 1] = range;
    
    if (range < 0.0 || range > 30.0)
      last_anchor_update[index - 1] = 0;  //error or out of bounds, ignore this measurement
  }

  int detected = 0;

  //reject old measurements
  for (i = 0; i < N_ANCHORS; i++) {
    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) last_anchor_update[i] = 0; //not from this one
    if (last_anchor_update[i] > 0) detected++;
  }

  if (detected == 4) //four measurements minimum
  {
    ap_send_beacon_config(N_ANCHORS, fConfig.aMatrix);
    ap_send_beacon_distance(N_ANCHORS, last_anchor_distance);

    trilat2D_4A();

#ifndef TAG_DEBUG
    //output the values (X, Y and error estimate)
    Serial1.print("P= ");
    Serial1.print(current_tag_position[0]);
    Serial1.write(',');
    Serial1.print(current_tag_position[1]);
    Serial1.write(',');
    Serial1.println(current_distance_rmse);
#endif    
  }
}  //end newRangeRun

void newRange()
{
  if (fConfig.mode == 'C') {
    // Calibrate mode
    Serial1.println(DW1000Ranging.getDistantDevice()->getRange());
  }
  else if (fConfig.mode == 'R') {
    // Run mode
    newRangeRun();
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

const byte *getDw1000Mode() {
  switch(fConfig.dw1000Mode) {
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
  fcboardSerial.begin(9600);

  delay(1000);

  //initialize configuration
  SPI.begin();
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // Load tag configuration
  loadTagConfig();
  printTagConfig();

  DW1000._maxPower = fConfig.dw100MaxPower;
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // start as tag, do not assign random short address
  DW1000Ranging.startAsTag(fConfig.address, getDw1000Mode(), false);
}

void copyCharArrValue(String str, char* carray, int maxLength) {
    for (int i = 0; i < str.length() && i < maxLength; i++) { 
      carray[i] = str[i]; 
    }
}

void setAnchorMatrix(int anchorInx, String value, int startPos) {
  const char *lStrValue = value.c_str() + startPos;
  char *lTokenValue = strtok(strdup(lStrValue), ",");
  
  int i = 0;
  while (lTokenValue != NULL && i <= 2) {
    fConfig.aMatrix[anchorInx][i++] = atof(lTokenValue);
    lTokenValue = strtok(NULL, ",");
  }
}

void loop()
{
  DW1000Ranging.loop();

#ifndef TAG_DEBUG
  if (Serial1.available()) {
    String lStr = Serial1.readString();
    lStr.trim();
    
    if (lStr.startsWith("SET_MODE=")) {
      const char *lStrValue = lStr.c_str() + 9;
      fConfig.mode = *lStrValue;
      
      if (fConfig.mode == 'C') {
        strcpy(fConfig.address, DEFAULT_ADDRESS);
      }
     
      saveTagConfig();
      NVIC_SystemReset();
    }
    else if (lStr.startsWith("SET_DW1000_MODE=")) {
      const char *lStrValue = lStr.c_str() + 16;
      fConfig.dw1000Mode = atoi(lStrValue);
      saveTagConfig();
      NVIC_SystemReset();
    }
    else if (lStr.startsWith("SET_ANCHOR1_XYZ=")) {
      setAnchorMatrix(0, lStr, 16);
      saveTagConfig();
      printTagConfig();
    }
    else if (lStr.startsWith("SET_ANCHOR2_XYZ=")) {
      setAnchorMatrix(1, lStr, 16);
      saveTagConfig();
      printTagConfig();
    }
    else if (lStr.startsWith("SET_ANCHOR3_XYZ=")) {
      setAnchorMatrix(2, lStr, 16);
      saveTagConfig();
      printTagConfig();
    }
    else if (lStr.startsWith("SET_ANCHOR4_XYZ=")) {
      setAnchorMatrix(3, lStr, 16);
      saveTagConfig();
      printTagConfig();
    }
    else if (lStr.startsWith("SET_DW1000_MAX_POWER=")) {
      const char *lStrValue = lStr.c_str() + 21;
      fConfig.dw100MaxPower = (*lStrValue == 'Y');
      saveTagConfig();
      NVIC_SystemReset();
    }
    else if (lStr == "REBOOT") {
      NVIC_SystemReset();
    }
    else if (lStr.startsWith("SET_ADDRESS=")) {
      String lAddress = lStr.substring(12);
      copyCharArrValue(lStr.substring(12), fConfig.address, 24);
      saveTagConfig();
      printTagConfig();
    }
    else if (lStr == "START_RANGE_FILTER") {
      DW1000Ranging.useRangeFilter(true);
      Serial1.println("Range filter started...");
    }
    else if (lStr == "STOP_RANGE_FILTER") {
      DW1000Ranging.useRangeFilter(false);
      Serial1.println("Range filter stopped.");
    }
  }
#endif  
}
