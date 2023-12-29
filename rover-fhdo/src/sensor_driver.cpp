/**
 * ESP32 Sensor Library
 * 
 * Functions to get distance values from the sensors
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */

// This example shows how to use non-blocking code to read
// from all three channels of the distance sensor
// and stores the results in an array.
//
// Each channel is sampled approximately 32 times per second.
//
// In addition to the usual power and I2C connections, you
// will need to connect the GP1 pin to an interrupt on your
// ESP32 board.  in this code, the pin D2 of the NodeMCU board is used
// pin D2 --> GP1. This is already done.


//documentation here: https://github.com/pololu/opt3101-arduino


// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !




#include <OPT3101.h>
#include "stdint.h"
#include <Wire.h>
#include "sensor_driver.h"


const uint8_t dataReadyPin = 2;


OPT3101 sensor; /* create an object of the sensor class */


int16_t distances[3]; /* array where we'll store the values of each sensor pair (Tx-Rx LED) */
volatile bool dataReady = false; /* to determine when the data is ready to be read */

void setDataReadyFlag()
{
  dataReady = true;
}

sclass::sclass() {
}

void sclass::SETUP() {
  /* Wire.begin takes two arguments, first being SDA and second being SCL (Wire.begin(SDA,SCL)) */
  Wire.begin(27,26);


  Serial.println("starting");

  /* Wait for the serial port to be opened before printing */
  /* messages (only applies to boards with native USB) */
  while (!Serial) {}

  sensor.init();
  if (sensor.getLastError())
  {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}
  }

  sensor.setFrameTiming(256);
  sensor.setChannel(0);
  sensor.setBrightness(OPT3101Brightness::Adaptive);

  sensor.startSample();
}

int16_t* sclass::reading() {


  if (sensor.isSampleDone())
  {
    sensor.readOutputRegs();

    distances[sensor.channelUsed] = sensor.distanceMillimeters;

    // if (sensor.channelUsed == 2)
    // {
    //   for (uint8_t i = 0; i < 3; i++)
    //   {
    //     Serial.print(distances[i]);
    //   }
    //   Serial.println();
    // }
    sensor.nextChannel();
    sensor.startSample();
  }
  
  delay(100);
  return distances;
}

sclass sensorobject = sclass(); /* creating an object of class sensor */

// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !
// This file is not in use an the moment. Please ignore !