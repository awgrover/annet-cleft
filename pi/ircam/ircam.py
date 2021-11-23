import time
import busio
import board
import adafruit_amg88xx
import adafruit_tca9548a
i2c = busio.I2C(board.SCL, board.SDA)
mux = adafruit_tca9548a.TCA9548A(i2c)
amg = adafruit_amg88xx.AMG88XX(mux[4])

while True:
    for row in amg.pixels:
        print(['{0:.1f}'.format(temp) for temp in row])
        print("")
    print("\n")
    time.sleep(0.1)
