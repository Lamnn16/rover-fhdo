#include <Arduino.h>
#include "collision.h"
#include "motorDriver.h"
#include <AWS.h>
#include <ArduinoJson.h>
#include <WString.h>

void taskSensorDataGet(void *parameter);
void taskMotorControl(void *parameter);
void taskAWS(void *parameter);

Collision collisionSensor;
mclass motorDriver;

bool obstacleDetected = false;

// Extract roverX and roverY
int roverX, roverY;
int targetX, targetY;
extern String receivedTargetPayload;
extern String receivedRoverPayload;

int previousTargetX;
int previousTargetY;

void setup()
{
  // pinMode(LED_BOARD, OUTPUT);
  Serial.begin(9600);

  motorDriver.SETUP();

  if (collisionSensor.init())
  {
    Serial.println("Collision sensor initialized successfully.");
  }
  else
  {
    Serial.println("Failed to initialize collision sensor.");
  }

  collisionSensor.setThreshDistance(200); // Set threshold distance to 200 mm

  delay(1000);

  // xTaskCreate(
  //                   taskBorder,          /* Task function. */
  //                   "TaskOne",        /* String with name of task. */
  //                   7644,              /* Stack size in bytes. */
  //                   NULL,             /* Parameter passed as input of the task */
  //                   1,                /* Priority of the task. */
  //                   NULL);            /* Task handle. */

  xTaskCreate(
      taskAWS,   /* Task function. */
      "TaskTwo", /* String with name of task. */
      20000,     /* Stack size in bytes. */
      NULL,      /* Parameter passed as input of the task */
      1,         /* Priority of the task. */
      NULL);     /* Task handle. */

  // xTaskCreate(
  //       taskSensorDataGet,   /* Task function. */
  //       "TaskSensorDataGet", /* String with name of task. */
  //       7644,      /* Stack size in bytes. */
  //       NULL,      /* Parameter passed as input of the task */
  //       1,         /* Priority of the task. */
  //       NULL);     /* Task handle. */

  xTaskCreate(
      taskMotorControl,   /* Task function. */
      "taskMotorControl", /* String with name of task. */
      20000,              /* Stack size in bytes. */
      NULL,               /* Parameter passed as input of the task */
      1,                  /* Priority of the task. */
      NULL);              /* Task handle. */
}

void loop()
{
}

void taskAWS(void *parameter)
{
  myawsclass awsobject = myawsclass();
  awsobject.connectAWS();
  for (;;)
  {
    awsobject.stayConnected();
  }

  Serial.println("Ending task 2"); // should not reach this point but in case...
  vTaskDelete(NULL);
}

void extractCoordinates(const String &payload, int &x, int &y)
{
  int a, b;
  int startIndex = payload.indexOf('(') + 1;
  int commaIndex = payload.indexOf(',', startIndex);
  int endIndex = payload.indexOf(')', commaIndex);

  String xStr = payload.substring(startIndex, commaIndex);
  String yStr = payload.substring(commaIndex + 1, endIndex);

  x = xStr.toInt();
  y = yStr.toInt();
}

// Updated PID gains and motor speeds
const float kP = 15;
const float kI = 0.001;
const float kD = 0.001;
const int roverSpeed = 100;

// Updated PID controller variables
float integralError = 0.0;
float previousError = 0.0;
float previousErrorKd = 0.0;

int i = 0;
void taskMotorControl(void *parameter)
{
  for (;;)
  {
    // Extract roverX and roverY from receivedRoverPayload
    extractCoordinates(receivedRoverPayload, roverX, roverY);
    extractCoordinates(receivedTargetPayload, targetX, targetY);

    if (previousTargetX != targetX)
    {
      i++;
      motorDriver.set_speed(MotorRight, Forward, roverSpeed+100);
      motorDriver.set_speed(MotorLeft, Backward, roverSpeed+100);
      
      vTaskDelay(2100 / portTICK_PERIOD_MS);

    }
    else
    {
      // Calculate distance between current rover coordinates and target coordinates
      float distance = sqrt(pow(targetX - roverX, 2) + pow(targetY - roverY, 2));
      Serial.println(distance);
      // Calculate angle between current rover coordinates and target coordinates
      float angle = atan2(targetY - roverY, targetX - roverX) * 180 / PI;

      // PID Control
      float error = angle - previousError;
      integralError += error;
      float derivativeError = error - previousErrorKd;

      // Calculate steering control output
      float steeringOutput = kP * error + kI * integralError + kD * derivativeError;

      // Determine steering direction
      enum SteeringDirection
      {
        Left,
        Straight,
        Right
      };
      SteeringDirection steering;

      // Adjust motor speeds and directions based on steering direction and PID output
      if (steeringOutput > 0)
      {
        motorDriver.set_speed(MotorRight, Forward, roverSpeed + 120);
        motorDriver.set_speed(MotorLeft, Forward, roverSpeed);
      }
      else if (steeringOutput < 0)
      {
        motorDriver.set_speed(MotorRight, Forward, roverSpeed);
        motorDriver.set_speed(MotorLeft, Forward, roverSpeed + 120);
      }

      // Update previous error for next iteration
      previousError = angle;
      previousErrorKd = error;

      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    previousTargetX = targetX;
  }

  vTaskDelete(NULL);
}