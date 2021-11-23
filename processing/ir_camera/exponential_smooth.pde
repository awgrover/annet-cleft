
public class ExponentialSmooth <ValueT extends Number> { // for <int> or <float>
  // a simple formula to sort-of do averaging: exponential smoothing

  private float _smoothed = 0; // need float for converging, i.e. no rounding loss funniness. use "(int) $thisobject", to get value.

  final float factor; // forces operations to float space, actually a whole number

  // "factor" is kind of like the the number of samples that are averaged.
  // So, "5" is sort of like taking 5 samples and averaging them.
  public ExponentialSmooth() { 
    this.factor = 1.0;
  }
  public ExponentialSmooth(final int factor) { 
    this.factor = (float) factor;
  }
  public ExponentialSmooth(final Float factor) { 
    this.factor = factor;
  }

  int to_int() { 
    return int( _smoothed) ;
  }
  float to_float() { 
    return (float) _smoothed;
  }

  float reset(ValueT v) { 
    _smoothed = v.floatValue(); 
    return v.floatValue();
  }

  // we intend it to inline
  float average(final ValueT raw_value) { 
    float x = raw_value.floatValue();
    _smoothed = x / factor + _smoothed - _smoothed / factor; 
    /*
    Serial.print("X"); Serial.print((int) this); Serial.print("/");
     Serial.print(factor); Serial.print(" ");
     Serial.print(raw_value); Serial.print(" ");
     Serial.print(_smoothed); Serial.print("|"); Serial.print((int)_smoothed); Serial.print("|"); Serial.print((int) *this);
     Serial.print(" ");
     */

    return _smoothed;
  }
}
