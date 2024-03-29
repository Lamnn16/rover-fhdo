/**
 * @file
 * 
 * ESP32 Motor Library
 * 
 * Functions to set motor speed
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */

#ifndef motor_h
#define motor_h

enum Motors
{
  MotorRight = 0,   /* Crawler Motor 1 */
  MotorLeft = 1    /* Crawler Motor 2 */
};

enum Direction
{
  Forward = 1,  /* Motor Forward */
  Backward = 0  /* Motor Backward */
};

class mclass {
  public:
    mclass();
    
    void SETUP();   /* Initialize the Motors */
    void SPEED(int motor_speed);
    void motor_direction(Motors motor_ch, Direction dir); /* set direction of rotation of the motors */
    void set_speed(Motors motor_ch, Direction dir, int new_speed);  /* set the speed of the motors */
    void motor_all_stop();  /* stop all motors */
};

extern mclass motorobject;

#endif