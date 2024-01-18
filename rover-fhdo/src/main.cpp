#include <Arduino.h>
#include "opt3101.h"
#include "sensor_driver.h"
#include "collision.h"
#include "motorDriver.h"

#define LED_BOARD 2 

void taskBlinkLed(void *parameter);
void taskSensorDataGet(void *parameter);
void taskMotorControl(void *parameter);

Collision collisionSensor;
mclass motorDriver;

bool obstacleDetected = false;

void setup()
{
  pinMode(LED_BOARD, OUTPUT);
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

  xTaskCreate(
      taskBlinkLed,   /* Task function. */
      "TaskBlinkLed", /* String with name of task. */
      7644,      /* Stack size in bytes. */
      NULL,      /* Parameter passed as input of the task */
      1,         /* Priority of the task. */
      NULL);     /* Task handle. */

  xTaskCreate(
      taskSensorDataGet,   /* Task function. */
      "TaskSensorDataGet", /* String with name of task. */
      7644,      /* Stack size in bytes. */
      NULL,      /* Parameter passed as input of the task */
      1,         /* Priority of the task. */
      NULL);     /* Task handle. */

  xTaskCreate(
      taskMotorControl,   /* Task function. */
      "taskMotorControl", /* String with name of task. */
      7644,        /* Stack size in bytes. */
      NULL,        /* Parameter passed as input of the task */
      1,           /* Priority of the task. */
      NULL);       /* Task handle. */
}

void loop()
{
  delay(1000);
}

void taskBlinkLed(void *parameter)
{
  for (;;)
  {
    digitalWrite(LED_BOARD, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS); 
    digitalWrite(LED_BOARD, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  } 
  vTaskDelete(NULL);
}

void taskSensorDataGet(void *parameter)
{
  Collision::segments segments;
  for (;;)
  {
    segments = collisionSensor.warning();
    if (segments.l)
    {
      Serial.println("Left collision detected!");
      motorDriver.motor_all_stop();
    }
    if (segments.c)
    {
      Serial.println("Center collision detected!");
      motorDriver.motor_all_stop();
    }
    if (segments.r)
    {
      Serial.println("Right collision detected!");
      motorDriver.motor_all_stop();
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.println("Ending task 2"); 
  vTaskDelete(NULL);
}

void taskMotorControl(void *parameter)
{
  for (;;)
  {
    motorDriver.set_speed(MotorA, Forward, 80);
    motorDriver.set_speed(MotorB, Forward, 80);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    motorDriver.set_speed(MotorA, Backward, 80);
    motorDriver.set_speed(MotorB, Backward, 80);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  Serial.println("Ending task 3");
  vTaskDelete(NULL);
}
