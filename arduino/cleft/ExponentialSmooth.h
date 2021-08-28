#pragma once

template <typename ValueT> // for <int> or <float>
class ExponentialSmooth {
  // a simple formula to sort-of do averaging: exponential smoothing

  float _smoothed; // need float for converging, i.e. no rounding loss funniness. use "(int) $thisobject", to get value.

  public:
  const float factor; // forces operations to float space, actually a whole number

  // "factor" is kind of like the the number of samples that are averaged.
  // So, "5" is sort of like taking 5 samples and averaging them.
  ExponentialSmooth(const ValueT factor = 1) : factor(factor) {};

  ValueT smoothed() { return (ValueT) _smoothed; }
  ValueT value() { return (ValueT) _smoothed; }
  operator int() const { return (int) _smoothed; }
  operator float() const { return (float) _smoothed; }

  ValueT reset(ValueT v) { _smoothed = v; return v;}

  // we intend it to inline
  ValueT average(const ValueT raw_value) { 
    _smoothed = raw_value / factor + _smoothed - _smoothed / factor; 
    /*
    Serial.print("X"); Serial.print((int) this); Serial.print("/");
    Serial.print(factor); Serial.print(" ");
    Serial.print(raw_value); Serial.print(" ");
    Serial.print(_smoothed); Serial.print("|"); Serial.print((int)_smoothed); Serial.print("|"); Serial.print((int) *this);
    Serial.print(" ");
    */

    return (ValueT) _smoothed;
    }

  };
