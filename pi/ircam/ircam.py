#!/usr/bin/env python3
try:
    import sys,pathlib,datetime
    import time
    import busio
    import board

    import adafruit_amg88xx
    import adafruit_tca9548a

    i2c = busio.I2C(board.SCL, board.SDA)

    MUX_CHANNEL = 0 # -1 to not use
    mux = i2c if MUX_CHANNEL==-1 else adafruit_tca9548a.TCA9548A(i2c)[MUX_CHANNEL]
    if MUX_CHANNEL==-1:
        print("I2C Direct (no mux)")
    else:
        print("Mux using channel {}".format(MUX_CHANNEL))

    IR_WIDTH = 8
    IR_HEIGHT = 8
    amg = adafruit_amg88xx.AMG88XX(mux)

    # startup
    print("Start {} {}\n".format( __file__,
        datetime.datetime.fromtimestamp(pathlib.Path(__file__).stat().st_mtime).isoformat()
        )
        )

    while True:
        for row in amg.pixels:
            print(['{0:.1f}'.format(temp) for temp in row])
            print("")
        print("\n")
        time.sleep(0.1)

except KeyboardInterrupt:
    sys.stderr.write("^C\n")
    exit(1)
