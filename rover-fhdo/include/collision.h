/**
 * @file distance.h
 * @author Michael Hoffmann (michael.hoffmann@fh-dortmund.de)
 * @brief Uses an OPT3101 distance sensor to detect collisions.
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <OPT3101.h>

class Collision {
public:
    typedef struct {
        uint_fast8_t l : 1; // left field of vision
        uint_fast8_t c : 1; // center field of vision
        uint_fast8_t r : 1; // right field of vision
    } segments;

    // Using undersized thresh_amplitude will result in random
    // distance values.  (100 seems fine for me.)
    Collision() { memset(&_segments, 0, sizeof(_segments)); }
    bool init();

    // Empty field of vision will return -1 otherwise the object
    // distance in millimeter.
    int16_t c_distance() const { return _c_distance; }
    // This MUST be called regularly for the ambient to be scanned.
    segments warning();

    // This is called by init() to set a default value of 150 mm.
    // Objects falling below this threshold should be recognized as
    // obstacles and warning() will set the affected segments as
    // 'true'.
    void setThreshDistance(int16_t d) { _thresh_distance = d; }

protected:
    static constexpr int16_t THRESH_AMPLITUDE_DEFAULT = 100;
    static constexpr int16_t THRESH_DISTANCE_DEFAULT = 150;
    uint16_t _thresh_amplitude = THRESH_AMPLITUDE_DEFAULT;
    int16_t _thresh_distance = THRESH_DISTANCE_DEFAULT;

private:
    OPT3101 _sensor = OPT3101();
    segments _segments;
    int16_t _c_distance = -1;
};
