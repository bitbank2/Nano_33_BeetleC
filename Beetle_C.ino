//
// Control a M5Stack Beetle-C robot car from an Arduino Nano 33 BLE + BLE HID gamepad
// Copyright (c) 2019 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// This code is free to use for personal projects
//

#include <Wire.h>
#include <ArduinoBLE.h>

enum {
  CONTROLLER_ACGAM_R1 = 0,
  CONTROLLER_MINIPLUS,
  CONTROLLER_UNKNOWN
};

char *szACGAM = (char *)"ACGAM R1          ";
char *szMINIPLUS = (char *)"MINI PLUS";
static int iControllerType;
volatile static int bConnected, bChanged;

// Current button state
// The first 8 bits are for the (up to 8) push buttons
volatile static uint16_t u16Buttons;
// bits to define the direction pad since it's not analog
#define BUTTON_LEFT 0x100
#define BUTTON_RIGHT 0x200
#define BUTTON_UP 0x400
#define BUTTON_DOWN 0x800
#define BUTTON_A 0x0010
#define BUTTON_B 0x0001
#define BUTTON_C 0x0008
#define BUTTON_D 0x0002
#define BUTTON_RETURN 0x0080
#define BUTTON_POWER 0x0040

void ProcessHIDReport(uint8_t *ucData, int iLen);

//
// Transmit the right wheel speed and direction
// 1 to 127 = forward
// -1 to -128 = backwards
// 
void leftwheel(uint8_t val) {
  Wire.beginTransmission(0x38);
  Wire.write(0x00);
  Wire.write(val);
  Wire.endTransmission();
}
//
// Transmit the right wheel speed and direction
// 1 to 127 = forward
// -1 to -128 = backwards
// 
void rightwheel(uint8_t val) {
  Wire.beginTransmission(0x38);
  Wire.write(0x01);
  Wire.write(val);
  Wire.endTransmission();
}

//
// Change an LED color
// LED number 0-6
// Color value = 0xRRGGBB
//
void led(uint8_t num, uint32_t val) {
  Wire.beginTransmission(0x38);
  Wire.write(0x02);
  Wire.write(num);
  Wire.write(uint8_t(val >> 16));  // red
  Wire.write(uint8_t(val >> 8));   // green
  Wire.write(uint8_t(val & 0xff)); // blue
  Wire.endTransmission();
}

//
// Callback which handles HID report updates
//
void HIDReportWritten(BLEDevice central, BLECharacteristic characteristic)
{
int iLen, i;
uint8_t ucTemp[128];

  // central wrote new HID report info
  iLen = characteristic.readValue(ucTemp, sizeof(ucTemp));
  ProcessHIDReport(ucTemp, iLen);
} /* HIDReportWritten() */
//
// Parse the HID report to get the control/button info
//
void ProcessHIDReport(uint8_t *ucData, int iLen)
{
int i;

  bChanged = 1;
  switch (iControllerType)
  {
    case CONTROLLER_ACGAM_R1:
      if (iLen == 2) // it always writes 2 byte reports
      {
        // assumes it's in "Game Mode"
        u16Buttons = ucData[0]; // 6 buttons
        i = ucData[1] & 0xc0; // Y axis (0x40 == centered)
        if (i == 0) // up
           u16Buttons |= BUTTON_UP;
        else if (i == 0x80) // down
           u16Buttons |= BUTTON_DOWN;
        i = ucData[1] & 0x30; // X axis (0x10 == centered)
        if (i == 0) // left
           u16Buttons |= BUTTON_LEFT;
        else if (i == 0x20)
           u16Buttons |= BUTTON_RIGHT;
      }
      break;
    case CONTROLLER_MINIPLUS:
      if (iLen == 9) // it always writes 9 byte reports
      {
        // directions
        u16Buttons &= ~(BUTTON_UP | BUTTON_DOWN | BUTTON_LEFT | BUTTON_RIGHT);
        if (ucData[2] == 0x81) // left
           u16Buttons |= BUTTON_LEFT;
        else if (ucData[2] == 0x7f) // right
           u16Buttons |= BUTTON_RIGHT;
        if (ucData[3] == 0x81) // up
           u16Buttons |= BUTTON_UP;
        else if (ucData[3] == 0x7f) // down
           u16Buttons |= BUTTON_DOWN;
        u16Buttons &= 0xff00; // gather other buttons
        u16Buttons |= ucData[0]; // A/B/C/D buttons
        u16Buttons |= (ucData[1] << 3); // start + select
      }
      break;
  } // switch on controller type
} /* ProcessHIDReport() */

//
// Loop while BLE is connected and use the control info to drive the Beetle-C
//
void monitorActions(BLEDevice peripheral) {

char ucTemp[128];
int i, iLen, iCount;
int l, r, delta;
uint16_t u16OldButts; // old state to know when a button is newly pressed or released

BLEService hidService;
//int ihidCount = 0;
  led(0,0x20); // blue to start
  // connect to the peripheral
  if (peripheral.connect()) {
    led(0,0x2000); // green = good
  } else {
    led(0,0x200000); // red = bad
    return;
  }

  // discover peripheral attributes
//  Serial.println("Discovering service 0x1812 ...");
  if (peripheral.discoverService("1812")) {
//    ShowMsg("0x1812 discovered", 6);
  } else {
//    ShowMsg("0x1812 disc failed", 6);
    led(0,0x200000); // red = bad
    peripheral.disconnect();

    while (1);
    return;
  }

  hidService = peripheral.service("1812"); // get the HID service

//  Serial.print("characteristic count = "); Serial.println(hidService.characteristicCount(), DEC);

  iCount = hidService.characteristicCount();
  for (i=0; i<iCount; i++)
  {
    BLECharacteristic bc = hidService.characteristic(i);
//    Serial.print("characteristic "); Serial.print(i, DEC);
//    Serial.print(" = "); Serial.println(bc.uuid());
//    Serial.print("Descriptor count = "); Serial.println(bc.descriptorCount(), DEC);
    if (strcasecmp(bc.uuid(),"2a4D") == 0) // enable notify
    {
      bc.subscribe();
      bc.setEventHandler(BLEWritten, HIDReportWritten);
    }
  }
    
  BLECharacteristic protmodChar = hidService.characteristic("2A4E"); // get protocol mode characteristic
  if (protmodChar != NULL)
  {
    protmodChar.writeValue((uint8_t)0x01); // set protocol report mode (we want reports) 
  }

  l = r = delta = 0;
  while (peripheral.connected()) {
    // while the peripheral is connected
    if (bChanged)
    {
      bChanged = 0;
      if (u16Buttons & BUTTON_UP)
        delta = 1;
      else if (u16Buttons & BUTTON_DOWN)
        delta -= 1;
      else
        delta = 0; // neutral joystick position
      if (u16Buttons & BUTTON_RETURN) // full stop
      {
        l = r = delta = 0;
      }
      // Control the front LED with the A-D buttons
      if (u16Buttons & BUTTON_A && !(u16OldButts & BUTTON_A))
         led(0, 0x200000); // red 
      if (u16Buttons & BUTTON_B && !(u16OldButts & BUTTON_B))
         led(0, 0x2000); // green 
      if (u16Buttons & BUTTON_C && !(u16OldButts & BUTTON_C))
         led(0, 0x20); // blue 
      if (u16Buttons & BUTTON_D && !(u16OldButts & BUTTON_D))
         led(0, 0x202020); // white 
      u16OldButts = u16Buttons;
    } // bChanged
    // Update the L/R motor speed variables with the current delta
    l += delta; r += delta;
    
    // constrain values to signed 8-bit limits
    if (l > 127) l = 127;
    else if (l < -128) l = -128;
    if (r > 127) r = 127;
    else if (r < -128) r = -128;
    // if turning, boost wheel
    if (u16Buttons & BUTTON_LEFT)
    {
      int L=l, R=r;
      if (abs(L) < 40) // going slow
        R = R*2; // boost right wheel speed to turn left
      L = L/2;
      leftwheel(L);
      rightwheel(R);
    }
    else if (u16Buttons & BUTTON_RIGHT)
    {
      int R=r, L=l;
      if (abs(R) < 40) // going slow
        L = L*2; // boost left wheel speed to turn right
      R = R/2;
      leftwheel(L);
      rightwheel(R);
    }
    else // not turning
    {
      leftwheel(l);
      rightwheel(r);
    }
    delay(10); // let this update loop run at about 100Hz
  } // while connected
} /* monitorActions() */

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  // begin initialization
  if (!BLE.begin())
  {
    led(0, 0xffffff); // white = total failure
    while (1);
  }
  // start scanning for peripherals
  BLE.scan();
  u16Buttons = 0;

} /* setup() */

void loop() {
char szTemp[64];

  bConnected = 0;
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
  // Compare it to known controllers
    iControllerType = CONTROLLER_UNKNOWN;
    if (strcmp(peripheral.localName().c_str(), szACGAM) == 0)
       iControllerType = CONTROLLER_ACGAM_R1;
    else if (strcmp(peripheral.localName().c_str(), szMINIPLUS) == 0)
       iControllerType = CONTROLLER_MINIPLUS;
       
    // Check if the peripheral is a HID device
    if (peripheral.advertisedServiceUuid() == "1812" && iControllerType != CONTROLLER_UNKNOWN) {
      // stop scanning
      BLE.stopScan();

      monitorActions(peripheral); // the action loop

      // peripheral disconnected, start scanning again
      BLE.scan();
      }
    }
} /* loop() */
