#include "util.hpp"

MovingAverage::MovingAverage() : sum(0), index(0), count(0) {
    for (int i = 0; i < WINDOW_SIZE; ++i) {
        window[i] = 0.0;
    }
}
float MovingAverage::addValue(double newValue) {
    // Subtract the old value from the sum
    sum -= window[index];
    
    // Add the new value to the window and to the sum
    window[index] = newValue;
    sum += newValue;

    // Increment the index, and wrap around if necessary
    index = (index + 1) % WINDOW_SIZE;
    
    // Keep track of the count to handle the initial fill of the window
    if (count < WINDOW_SIZE) {
        count++;
    }
    
    // Calculate the average
    return sum / count;
}
