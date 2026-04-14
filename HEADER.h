

#ifndef HEADER_H  
#define HEADER_H

#include <stdint.h>

void measure_temp(bool open_wire_check = false);


void poll_ADC(uint16_t command, bool curr_measure = false);     //curr_measure selects whether a current measurement is taken while polling the ADC (for synchronous Current and voltage measurements to determine cell internal resistance)

template <size_t rows, size_t cols> inline void min_max(float arr_2D[rows][cols], float* min, float* max) {   //finds the max and min values in any static 2D array of floats
    for (size_t i = 0; i < rows; ++i) {                                                                       //min and max must be initalized to sensible values beforehand
        for (size_t j = 0; j < cols; ++j) {
            if (arr_2D[i][j] < *min) {
                *min = arr_2D[i][j];
            }
            if (arr_2D[i][j] > *max) {
                *max = arr_2D[i][j];
            }
        }
    }
}

template <size_t length> inline int search(float arr[length], float value, bool return_lower = false) {       //searches a sorted DECRESAING list for the nearest element and returns its index. the "lower" flag if set will return the nearest element that is equal or lower
    //online function testbench:
    //https://www.programiz.com/online-compiler/5fkt3FMi4yJhY
    if (value >= arr[0]) 
        return(0);
    if (value <= arr[length - 1])
        return(length - 1);
    int low = 0;
    int high = length - 1;
    while (low <= high) {
            int mid = low + (high - low) / 2;
            if (arr[mid] == value)
                return mid;
            if (arr[mid] > value)
                low = mid + 1;
            else
                high = mid - 1;
        }

    if (return_lower || std::abs(arr[low] - value) < std::abs(arr[high] - value)) {
        return low; // arr[low] is closer
    } else {
        return high; // arr[high] is closer
    }
}

template <size_t length> inline float interpolate(float arr_x[length], float arr_y[length], float x_value) {       //linear interpolate an x-value from sorted DECREASING arrays: Y = Y1 + (Y2-Y1)/(X2-X1)*(X-X1)
  int x1_index = 0; 
  int x2_index = 0;
  //out of range cases just return the y-bound of the array
  if(x_value >= arr_x[0]) 
    return arr_y[0];
  if(x_value <= arr_x[length - 1])
    return arr_y[length - 1];

  x1_index = search<length>(arr_x, x_value, true);
  if(x1_index == 0){        //this condition should not be true if the search function works and the edge cases are handled properly
    return arr_y[0];
  }
  x2_index = x1_index - 1;    //decreasing arrays so x2_index is less than x1_index
  float y_value = arr_y[x1_index] + (arr_y[x2_index] - arr_y[x1_index]) / (arr_x[x2_index] - arr_x[x1_index]) * (x_value - arr_x[x1_index]);
  return(y_value);
}

String format_string(const char* format, va_list args) {
    if (format == NULL)
        return "";

    String result = "";
    for (int i = 0; format[i] != '\0'; i++) {
        if (format[i] == '%') {
            i++;
            switch (format[i]) {
            case 'd': // INT
                result += va_arg(args, int);
                break;
            case 'f': // FLOAT OR DOUBLE
                result += va_arg(args, double);
                break;
            case 'c': // CHAR
                result += (char)va_arg(args, int);
                break;
            case 's': // CHAR* (STRING)
                result += va_arg(args, char*);
                break;
            case 'u': // UNSIGNED INT
                result += va_arg(args, unsigned int);
                break;
            default:
                result += '%';
                if (format[i] != '\0')
                    result += format[i];
                break;
            }
        } else {
            result += format[i];
        }
    }
    return result;
}

void print_with_args(const char* format, ...) {
    va_list args;
    va_start(args, format);
    String result = format_string(format, args);
    va_end(args);
    Serial.print(result);
}

void println_with_args(const char* format, ...) {
    va_list args;
    va_start(args, format);
    String result = format_string(format, args);
    va_end(args);
    Serial.println(result);
}

#endif
