#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"
#include <driver/adc.h>
#include "sdkconfig.h"
#include "MPU9250.h"


BLEHIDDevice* hid;
BLECharacteristic* inputMouse;
int16_t ax, ay, az, gx, gy, gz;
int vx, vy;
MPU9250 IMU(Wire,0x68);

#define XACCEL_MIN 0.2      // Minimum range of X axis acceleration, values below
                            // this won't move the mouse at all.
#define XACCEL_MAX 8.0      // Maximum range of X axis acceleration, values above
                            // this will move the mouse as fast as possible.
#define XMOUSE_RANGE 35.0   // Range of velocity for mouse movements.  The higher
                            // this value the faster the mouse will move.
#define XMOUSE_SCALE .5      // Scaling value to apply to mouse movement, this is
                            // useful to set to -1 to flip the X axis movement.

#define YACCEL_MIN XACCEL_MIN
#define YACCEL_MAX XACCEL_MAX
#define YMOUSE_RANGE XMOUSE_RANGE
#define YMOUSE_SCALE .5


const int ledPin =     5;      // the number of the LED pin

bool connected = false;

class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    connected = true;
    BLE2902* desc = (BLE2902*)inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(true);
  }

  void onDisconnect(BLEServer* pServer){
    connected = false;
    BLE2902* desc = (BLE2902*)inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(false);
    digitalWrite(ledPin, HIGH);
  }
};



void taskServer(void*){


    BLEDevice::init("ArmsRaise-GR");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyCallbacks());

    hid = new BLEHIDDevice(pServer);
    inputMouse = hid->inputReport(1); // <-- input REPORTID from report map

    std::string name = "GurleyRalston";
    hid->manufacturer()->setValue(name);

    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00,0x02);

  BLESecurity *pSecurity = new BLESecurity();

  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

// http://www.keil.com/forum/15671/usb-mouse-with-scroll-wheel/
// Wheel Mouse - simplified version - 5 button, vertical and horizontal wheel
//
// Input report - 5 bytes
//
//     Byte | D7      D6      D5      D4      D3      D2      D1      D0
//    ------+---------------------------------------------------------------------
//      0   |  0       0       0    Forward  Back    Middle  Right   Left (Buttons)
//      1   |                             X
//      2   |                             Y
//      3   |                       Vertical Wheel
//      4   |                    Horizontal (Tilt) Wheel
//
// Feature report - 1 byte
//
//     Byte | D7      D6      D5      D4   |  D3      D2  |   D1      D0
//    ------+------------------------------+--------------+----------------
//      0   |  0       0       0       0   |  Horizontal  |    Vertical
//                                             (Resolution multiplier)
//
// Reference
//    Wheel.docx in "Enhanced Wheel Support in Windows Vista" on MS WHDC
//    http://www.microsoft.com/whdc/device/input/wheel.mspx
//



const uint8_t reportMapMouse[] = {

USAGE_PAGE(1),       0x01,
      USAGE(1),         0x02,
       COLLECTION(1),     0x01,
       REPORT_ID(1),  0x01,
       USAGE(1),        0x01,
       COLLECTION(1),     0x00,
       USAGE_PAGE(1),     0x09,
       USAGE_MINIMUM(1),    0x1,
       USAGE_MAXIMUM(1),    0x3,
       LOGICAL_MINIMUM(1),  0x0,
       LOGICAL_MAXIMUM(1),  0x1,
       REPORT_COUNT(1),   0x3,
       REPORT_SIZE(1),    0x1,
       0x80|0x01,        0x2,    // (Data, Variable, Absolute), ;3 button bits
       REPORT_COUNT(1),   0x1,
       REPORT_SIZE(1),    0x5,
       0x80|0x01,        0x1,    //(Constant), ;5 bit padding
       USAGE_PAGE(1),     0x1,    //(Generic Desktop),
       USAGE(1),        0x30,
       USAGE(1),        0x31,
       LOGICAL_MINIMUM(1),  0x81,
       LOGICAL_MAXIMUM(1),  0x7f,
       REPORT_SIZE(1),    0x8,
       REPORT_COUNT(1),   0x2,
       0x80|0x01,        0x6,    //(Data, Variable, Relative), ;2 position bytes (X & Y)
       END_COLLECTION(0),
      END_COLLECTION(0)
};

    hid->reportMap((uint8_t*)reportMapMouse, sizeof(reportMapMouse));
    hid->startServices();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_MOUSE);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();
    hid->setBatteryLevel(7);

    ESP_LOGD(LOG_TAG, "Advertising started!");
    delay(portMAX_DELAY);
  
};


const int button1Pin = 39;     // the number of the pushbutton pin
const int button2Pin = 38;     // the number of the pushbutton pin


float lerp(float x, float x0, float x1, float y0, float y1) {
  // Check if the input value (x) is outside its desired range and clamp to
  // those min/max y values.
  if (x <= x0) {
    return y0;
  }
  else if (x >= x1) {
    return y1;
  }
  // Otherwise compute the value y based on x's position within its range and
  // the desired y min & max.
  return y0 + (y1-y0)*((x-x0)/(x1-x0));
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  
  IMU.begin();

  xTaskCreate(taskServer, "server", 20000, NULL, 5, NULL);
}

void loop() {
  
  if(connected){
    digitalWrite(ledPin, LOW);

    //vTaskDelay(5000);Serial.println("dormindo");

     while (digitalRead(button2Pin) ==  LOW ){ 
      
       IMU.readSensor();
   
      
      Serial.println("mouse Scroll UP");
          //<button>, <x>, <y>, <wheel>

          float x = IMU.getAccelX_mss();
          float y = IMU.getAccelY_mss();
          
          // Use the magnitude of acceleration to interpolate the mouse velocity.
          float x_mag = abs(x);
          float x_mouse = lerp(x_mag, XACCEL_MIN, XACCEL_MAX, 0.0, XMOUSE_RANGE);
          float y_mag = abs(y);
          float y_mouse = lerp(y_mag, YACCEL_MIN, YACCEL_MAX, 0.0, YMOUSE_RANGE);
          // Change the mouse direction based on the direction of the acceleration.
          if (x < 0) {
              x_mouse *= -1.0;
          }
          if (y < 0) {
              y_mouse *= -1.0;
          }
          // Apply any global scaling to the axis (to flip it for example) and truncate
          // to an integer value.
          x_mouse = floor(x_mouse*XMOUSE_SCALE);
          y_mouse = floor(y_mouse*YMOUSE_SCALE);


          Serial.println(y_mouse);

           uint8_t msg[] = {0x00, x_mouse, y_mouse};  
         
          inputMouse->setValue(msg,3);
          inputMouse->notify();
          delay(10);
         
          
     }    
    
  } else{
    digitalWrite(ledPin, HIGH);
  }
  delay(50);
}
