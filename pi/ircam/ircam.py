#!/usr/bin/env python3
try:
    import sys,pathlib,datetime
    import time
    import busio
    import board

    import adafruit_amg88xx
    import adafruit_tca9548a

    sys.path.append("./lib")
    print(sys.path)
    from every.every import Every
    from exponential_smooth import ExponentialSmooth

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


    # LOOP

    # we'll smooth the pixels
    smoothed_pixels = [[0] * IR_WIDTH for _ in range(IR_HEIGHT)]
    exp_smooth = 5

    while True:
        # data for processing is stereotyped:
        # dataname[int|float, ... ,] # note final comma for convenience

        say_size = Every(3 * 1000, True) # fire immediately
        ir_framerate = Every(100)
        #cam_rate = ExponentialSmooth(5)
        
        ## check_for_command()

        if say_size():
            print("xy[{},{}]".format(IR_WIDTH,IR_HEIGHT)) # PROCESSING needs this for the display

        if ir_framerate():
            start_read = time.monotonic()

            print("xy[")
            for row_i,row in enumerate(amg.pixels):
                for col_i, temp in enumerate(row):
                    # I choose to round to .1, raw data is rounded to .25's
                    smoothed_pixels[row_i][col_i] = round(
                        (exp_smooth-1)*(smoothed_pixels[row_i][col_i]/exp_smooth) + temp/exp_smooth,
                        1
                        )
                    sys.stdout.write('{:0.1f},'.format( smoothed_pixels[row_i][col_i] ))
                print("")
            print("]")
            print("Read {:0.1f} msec".format( (time.monotonic() - start_read) * 1000))

except KeyboardInterrupt:
    sys.stderr.write("^C\n")
    exit(1)
