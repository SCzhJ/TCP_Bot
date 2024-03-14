#ifndef UTIL_HPP
#define UTIL_HPP

#include <Arduino.h>

#define WINDOW_SIZE 3
class MovingAverage {
private:
    float window[WINDOW_SIZE];
    float sum;
    int index;
    int count; // Number of values added
public:
    MovingAverage();
    float addValue(double newValue);
};

// Example usage:
// MovingAverage filter; // Moving average window of size WINDOW_SIZE
// float avg = filter.addValue(10.0); // Adds a value and gets the current average

#endif