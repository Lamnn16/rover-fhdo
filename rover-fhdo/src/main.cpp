#include <Arduino.h>
#include "opt3101.h"
// #include "sensorDriver.h"
#include "collision.h"
#include "motorDriver.h"
#include <AWS.h>

#define LED_BOARD 2 //change here the pin of the board to V2

void taskBlinkLed(void *parameter);
void taskSensorDataGet(void *parameter);
void taskMotorControl(void *parameter);
void taskAWS( void * parameter);

void moveRover(int leftSpeed, int rightSpeed);

int roverX = 0;
int roverY = 0;
int targetX = 10;
int targetY = 10;

Collision collisionSensor;
mclass motorDriver;

bool obstacleDetected = false;


void setup(){
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
                    taskBlinkLed,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    7644,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

  xTaskCreate(
                    taskAWS,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    7644,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

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

void loop(){
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
 
void taskAWS( void * parameter)
{
  myawsclass awsobject = myawsclass();
  Serial.println("Hello from task: 2");
  int i = 0;
  awsobject.connectAWS();
  //create an endless loop so the task executes forever
  for( ;; )
  {
    
    //awsobject.publishMessage(i);
    delay(1000);
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    //awsobject.publishMessage(i);
    
    //vTaskDelay(500 / portTICK_PERIOD_MS);
    //Serial.println("Hello from task: 2");
    i++;
    awsobject.stayConnected();
  }
  
  Serial.println("Ending task 2"); //should not reach this point but in case...
  vTaskDelete( NULL );
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
    // Calculate the differences between target and rover coordinates
    int diffX = targetX - roverX;
    int diffY = targetY - roverY;
  
    // Calculate the angle to the target
    float angle = atan2(diffY, diffX) * 180.0 / PI;
    if (angle < 0) {
      angle += 360.0;
    }
    
      // Move the rover towards the target
    if (diffX != 0 || diffY != 0) {
      // Adjust the rover's speed based on the angle
      int roverSpeed = 100;
      int rightSpeed = 100;
  
      // Adjust the rover's speed based on the angle
      if (angle <= 45) {
        motorDriver.set_speed(MotorA, Forward, 50);
        motorDriver.set_speed(MotorB, Forward, 50);
      }
      if (angle > 45 && angle <= 135) {
        // Turn left
        motorDriver.set_speed(MotorA, Forward, roverSpeed+40);
        motorDriver.set_speed(MotorB, Forward, roverSpeed);
      } else if (angle > 135 && angle <= 225) {
        // Turn around
        motorDriver.set_speed(MotorA, Forward, roverSpeed);
        motorDriver.set_speed(MotorB, Backward, roverSpeed);
      } else if (angle > 225 && angle <= 315) {
        // Turn right
        motorDriver.set_speed(MotorA, Forward, roverSpeed);
        motorDriver.set_speed(MotorB, Forward, roverSpeed+40);
      }
  
    } else {
      // Stop the rover when it reaches the target
       motorDriver.set_speed(MotorA, Forward, 0);
       motorDriver.set_speed(MotorB, Forward, 0);
    }
  }
  Serial.println("Ending task 3");
  vTaskDelete(NULL);
}

