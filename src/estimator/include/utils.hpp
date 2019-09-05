#pragma once

#include <iostream>

template <typename T>
struct Point {
  T x;
  T y;

  Point(T x, T y)
  {
    (*this).x = x;
    (*this).y = y;
  }
};
