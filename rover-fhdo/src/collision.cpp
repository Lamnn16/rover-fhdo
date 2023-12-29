/**
 * @file collision.cpp
 * @author Michael Hoffmann (michael.hoffmann@fh-dortmund.de)
 * @brief Uses an OPT3101 seg_distance sensor to detect collisions.
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <Arduino.h>
#include "collision.h"
#include <Wire.h>

bool Collision::init()
{
    Wire.begin(27, 26);

    Serial.println("starting");

    /* Wait for the serial port to be opened before printing */
    /* messages (only applies to boards with native USB) */
    while (!Serial)
    {
    }

    _sensor.init();
    if (_sensor.getLastError())
    {
        Serial.print(F("Failed to initialize OPT3101: error "));
        Serial.println(_sensor.getLastError());
        return false;
    }
    _sensor.setBrightness(OPT3101Brightness::Low);
    _sensor.setChannel(0);
    _sensor.startSample();
    return true;
}

Collision::segments Collision::warning()
{
    while (_sensor.isSampleDone() == false)
        delay(1);
    _sensor.readOutputRegs();

    enum SEG_NAME
    {
        TX0 = 0,
        TX1,
        TX2
    };
    enum SEG_NAME cur_seg = (enum SEG_NAME)_sensor.channelUsed;
    bool valid_sample = (_sensor.amplitude >= _thresh_amplitude) ? true : false;

    int16_t seg_distance = valid_sample ? _sensor.distanceMillimeters : -1;
    bool warning = false;
    if (valid_sample && (seg_distance <= _thresh_distance))
        warning = true;

    _sensor.nextChannel();
    _sensor.startSample();

    switch (cur_seg)
    {
    case TX0:
        _segments.l = warning;
        break;
    case TX1:
        _segments.c = warning;
        _c_distance = seg_distance;
        break;
    case TX2:
        _segments.r = warning;
        break;
    }
    return _segments;
}
