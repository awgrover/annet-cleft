class ExponentialSmooth(object):
  # a simple formula to sort-of do averaging: exponential smoothing

  def __init__(self, beta, initial=0):
      # "beta" is kind of like the the number of samples that are averaged.
      # So, "5" is sort of like taking 5 samples and averaging them.

    # these need to be float math
    self.beta = float(beta)
    self._smoothed = initial

  @property
  def value(self):
    return int( self._smoothed)

  def reset(self, v):
    self._smoothed = v
    return v

  # we intend it to inline
  def update(self, raw_value):
    self._smoothed = raw_value / self.beta + self._smoothed - self._smoothed / self.beta; 
    # print("  xial %d / %s from %s" % (self.value, self._smoothed, raw_value))

    return self.value
