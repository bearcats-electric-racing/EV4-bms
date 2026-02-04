#ifndef TUTILS_H
#define TUTILS_H

#include <cstddef>
#include <cmath>

template <size_t rows, size_t cols> inline void min_max(float arr_2D[rows][cols], float &min, float &max) {   //finds the max and min values in any static 2D array of floats
    for (size_t i = 0; i < rows; ++i) {                                                                       //min and max must be initalized to sensible values beforehand
        for (size_t j = 0; j < cols; ++j) {
            if (arr_2D[i][j] < min) {
                min = arr_2D[i][j];
            }
            if (arr_2D[i][j] > max) {
                max = arr_2D[i][j];
            }
        }
    }
}

template <size_t length> inline int search(const float arr[length], const float value, bool return_lower = false) {       //searches a sorted DECRESAING list for the nearest element and returns its index. the "lower" flag if set will return the nearest element that is equal or lower
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

template <size_t length> inline float interpolate(const float arr_x[length], const float arr_y[length], const float x_value) {       //linear interpolate an x-value from sorted DECREASING arrays: Y = Y1 + (Y2-Y1)/(X2-X1)*(X-X1)
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

#endif