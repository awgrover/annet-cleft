#!/usr/bin/env python3
try:
    import sys,pathlib,datetime,time,os
    ### Check for Pi
    if os.path.exists('/sys/firmware/devicetree/base/model'):
        with open('/sys/firmware/devicetree/base/model') as f:
            _basemodel = f.readline()
            if not 'Pi' in _basemodel:
                sys.stderr.write("# Not a pi! {} from {}\n".format(_basemodel, '/sys/firmware/devicetree/base/model'))
                exit(1)
    else:
        sys.stderr.write("# Not a pi! no {}\n".format('/sys/firmware/devicetree/base/model'))
        exit(1)
    import busio
    import board

    import adafruit_amg88xx
    import adafruit_tca9548a

    # our libs
    sys.path.append("./lib")
    from every.every import Every
    from every.every import Timer
    from exponential_smooth import ExponentialSmooth

    import cleft
    from arduino_port import ArduinoPort

    # Constants etc
    WRITE_DATA = True # True to print pixels etc
    MIRRORED = False # mirror left-right of camera == as if facing camera
    INVERTED = True # mirror top-bottom of camera

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

    arduino = ArduinoPort() # default search for likely port
    arduino_warmed_up = Timer(60) # FIXME: wait for arduino to say "ready"

    def celsius(f):
        return (f-32) * 5/9

    # add analyzers, .next() for each frame, call with (x,y, temp)
    analyze = cleft.Analyze() 
    histo = cleft.Histo( (90-60)*2, celsius(60),celsius(90) )
    minmax = cleft.MinMax(histo) 
    firsthigh = cleft.FirstHigh(histo)
    background = cleft.BackgroundTemp(minmax, histo, firsthigh)
    hotspot = cleft.HotSpot(minmax, background)
    analyze.accumulator( histo )
    analyze.accumulator( minmax )
    analyze.post( firsthigh ) # uses histo for analysis
    analyze.post( background ) # uses others for analysis
    analyze.post( hotspot ) # uses others for analysis

    # startup
    sys.stdout.reconfigure(line_buffering=True)
    print("Start {} {}\n".format( __file__,
        datetime.datetime.fromtimestamp(pathlib.Path(__file__).stat().st_mtime).isoformat()
        )
        )


    # LOOP

    # we'll smooth the pixels
    smoothed_pixels = [[0] * IR_WIDTH for _ in range(IR_HEIGHT)]
    exp_smooth = 5
    frame_ct = 0
    was_animation = None
        
    say_size = Every(3.0, True) # fire immediately
    ir_framerate = Every(0.500) # hmm, processing gets way behind at 0.1
    say_stats = Every(1.0);

    while True:
        # data for processing is stereotyped:
        # dataname[int|float, ... ,] # note final comma for convenience

        ## check_for_command() # none right now

        if say_size():
            print("xy[{},{},]".format(IR_WIDTH,IR_HEIGHT)) # PROCESSING needs this for the display

        if ir_framerate():
            start_read = time.monotonic()

            analyze.next()

            # Print the pixels, analyzing as we go
            sys.stdout.write("[") # PROCESSING temp array
            for row_i,row in enumerate(reversed(amg.pixels) if INVERTED else amg.pixels):
                for col_i, temp in enumerate(reversed(row) if MIRRORED else row): # mirrored if facing camera
                    # I choose to round to .1, raw data is rounded to .25's
                    smoothed_pixels[row_i][col_i] = round(
                        (exp_smooth-1)*(smoothed_pixels[row_i][col_i]/exp_smooth) + temp/exp_smooth,
                        1
                        )
                    smoothed_temp = smoothed_pixels[row_i][col_i]

                    analyze( col_i,row_i, smoothed_temp)

                    if WRITE_DATA:
                        sys.stdout.write('{:0.1f},'.format( smoothed_temp ))
                if WRITE_DATA:
                    print("")
            print("]")
            analyze() # for "posts"
            frame_ct += 1

            # wait at least 10 frames to let exponential smooths stabilize
            if say_stats() and frame_ct > 10:
                analyze.print_stats()
                print("endstats[]")

            animation = cleft.choose_animation( hotspot, was_animation )
            if animation == None:
                print("animation None was {}".format(was_animation))
            else:
                was_animation = animation

            if frame_ct > 10:
                if animation != None:
                    print("animation[{},]".format(animation))
                    if animation != None and animation != cleft.IDLE:
                        # update arduino, even if still the same animation
                        if not arduino_warmed_up.running: # i.e. expired
                            arduino.write(str(animation)) # single character assumed

            print("{:0.1f} msec Read & Analyze".format( (time.monotonic() - start_read) * 1000))

except KeyboardInterrupt:
    sys.stderr.write("^C\n")
    exit(1)
