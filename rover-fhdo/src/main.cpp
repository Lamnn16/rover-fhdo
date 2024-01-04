#include <Arduino.h>
#include "opt3101.h"
#include "sensor_driver.h"
#include "collision.h"
#include "motorDriver.h"
#include "AWS.h"

void taskOne(void *parameter);
void taskTwo(void *parameter);
#define LED_BOARD 2 // change here the pin of the board to V2

Collision collisionSensor;
mclass motorDriver;


void setup()
{
  pinMode(LED_BOARD, OUTPUT);
  Serial.begin(9600);
  // sensorObject.SETUP();

  motorDriver.SETUP();
  motorDriver.motor_direction(MotorA, Backward);
  motorDriver.set_speed(MotorA, Backward, 200);

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
      taskOne,   /* Task function. */
      "TaskOne", /* String with name of task. */
      5096,      /* Stack size in bytes. */
      NULL,      /* Parameter passed as input of the task */
      1,         /* Priority of the task. */
      NULL);     /* Task handle. */

  xTaskCreate(
      taskTwo,   /* Task function. */
      "TaskTwo", /* String with name of task. */
      5096,      /* Stack size in bytes. */
      NULL,      /* Parameter passed as input of the task */
      1,         /* Priority of the task. */
      NULL);     /* Task handle. */
}

void loop()
{

  // Your loop code here
  delay(1000);
}

void taskOne(void *parameter)
{
  // example of a task that executes for some time and then is deleted
  myawsclass awsobject = myawsclass();
  Serial.println("Hello from task: 2");
  int i = 0;
  awsobject.connectAWS();
  //create an endless loop so the task executes forever
  for (;;)
  {
    // Code section for getting data from AWS
    
     delay(1000);
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    awsobject.publishMessage(i);
    
    //vTaskDelay(500 / portTICK_PERIOD_MS);
    //Serial.println("Hello from task: 2");
    i++;
    awsobject.stayConnected();
    // The blink led code below just be used for simulating an example task
    // Switch on the LED
    digitalWrite(LED_BOARD, HIGH);
    // Pause the task for 1000ms
    // delay(1000); //This delay doesn't give a chance to the other tasks to execute
    vTaskDelay(500 / portTICK_PERIOD_MS); // this pauses the task, so others can execute
    // Switch off the LED
    digitalWrite(LED_BOARD, LOW);
    // Pause the task again for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void taskTwo(void *parameter)
{
  // create an endless loop so the task executes forever
  for (;;)
  {

    Collision::segments segments = collisionSensor.warning();

    if (segments.l)
    {
      Serial.println("Left collision detected!");
      // Control motor here. For example:
      // motorDriver.motor_direction(MotorA, Backward);
      // motorDriver.motor_direction(MotorB, Forward);
    }

    if (segments.c)
    {

      Serial.println("Center collision detected!");
      int16_t cDistance = collisionSensor.c_distance();
      Serial.print("Distance from center sensor: ");
      Serial.print(cDistance);
      Serial.println(" mm");
      
      // Control motor here. For example:
      // motorDriver.motor_direction(MotorA, Backward);
      // motorDriver.motor_direction(MotorB, Backward);
    }

    if (segments.r)
    {
      Serial.println("Right collision detected!");
      
      // Control motor here. For example:
      // motorDriver.motor_direction(MotorA, Forward);
      // motorDriver.motor_direction(MotorB, Backward);
      
    }

    delay(100); // Delay between readings
  }
  Serial.println("Ending task 2"); // should not reach this point but in case...
  vTaskDelete(NULL);
}
