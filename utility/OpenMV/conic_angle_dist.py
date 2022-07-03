# color tracking with conic mirror - By: EmaMaker - wed 15 jan 2020
# Based on:
# color tracking - By: paolix - ven mag 18 2018

# Automatic RGB565 Color Tracking Example
#

import sensor, image, time, pyb, math

from pyb import UART
uart = UART(3,19200, timeout_char = 1000)

CHR_UNKNOWN = '999'

y_found = False
b_found = False

#From Arduino Documentation at: https://www.arduino.cc/reference/en/language/functions/math/map/
def val_map(x, in_min, in_max, out_min, out_max):
    x = int(x)
    in_min = int(in_min)
    in_max = int(in_max)
    out_min = int(out_min)
    out_max = int(out_max)
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Check side
def isInLeftSide(img, x):
    return x < img.height() / 2

def isInRightSide(img, x):
    return x >= img.height() / 2


# LED Setup ##################################################################
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.off()
green_led.off()
blue_led.on()
##############################################################################


thresholds = [  (72, 100, -26, 12, 37, 91),    # thresholds yellow goalz
                (45, 70, -9, 29, -80, -42)]  # thresholds blue goal (6, 31, -15, 4, -35, 0)


roi = (40, 0, 260, 240)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(roi)
sensor.set_contrast(3)
sensor.set_saturation(3)
sensor.set_brightness(3)
sensor.set_auto_whitebal(False, (-6.02073, -5.494869, -0.8559153))
sensor.set_auto_exposure(False, 5845)
#sensor.set_auto_gain(True)
sensor.skip_frames(time = 300)

clock = time.clock()
##############################################################################


while(True):
    clock.tick()

    #print("Exposure: " + str(sensor.get_exposure_us()) + " Gain: " + str(sensor.get_gain_db()) + " White Bal: " + str(sensor.get_rgb_gain_db()))

    blue_led.off()

    y_found = False
    b_found = False

    tt_yellow = [(0,999,0,1)]     ## creo una lista di tuple per il giallo, valore x = 999 : non trovata
    tt_blue = [(0,999,0,2)]       ## creo una lista di tuple per il blue, valore x = 999 : non trovata

    img = sensor.snapshot()
    for blob in img.find_blobs(thresholds, pixels_threshold=100, area_threshold=100, merge = True):
        img.draw_rectangle(blob.rect())
        #img.draw_cross(blob.cx(), blob.cy())

        if (blob.code() == 1):
            tt_yellow = tt_yellow +  [ (blob.area(),blob.cx(),blob.cy(),blob.code() ) ]
            y_found = True
        if (blob.code() == 2):
            tt_blue = tt_blue +  [ (blob.area(),blob.cx(),blob.cy(),blob.code() ) ]
            b_found = True

    tt_yellow.sort(key=lambda tup: tup[0])  ## ordino le liste
    tt_blue.sort(key=lambda tup: tup[0])    ## ordino le liste

    ny = len(tt_yellow)
    nb = len(tt_blue)

    #Compute everything related to Yellow First
    y_area, y1_cx, y1_cy, y_code = tt_yellow[ny-1]
    y_cx = int(y1_cx - img.width() / 2)
    y_cy = int(img.height() / 2 - y1_cy)

    #For simplicity's sake, normalize data between 0 and 99 (100 values in total)
    if y_found == True:
        img.draw_cross(y1_cx, y1_cy)

        #y_cy = val_map(y_cx, -img.height() / 2, img.height() / 2, 99, 0)
        #y_cx = val_map(y_cy, -img.width() / 2, img.width() / 2, 0, 99)

        #compute angle
        y_angle = atan2(y_cy, y_cx)
        y_angle = -90 + (y_angle * 180 / 3.14) #convert to degrees and shift
        y_angle = (y_angle+360)%360 #make sure it's all positive, so we don't have strange things happening when receiving on teensy's side

        #compute dist
        y_dist = sqrt(y_cx*y_cx + y_cy*y_cy)

        data = "{}{}{}{}{}".format("Y", str(y_angle), "-", str(y_dist), "y")
    # il numero 999 indica che non ho trovato nessun blob
    else:
        data = "{}{}{}{}{}".format("Y", "999", "-", "0", "y")


    # trasmetto dati in seriale e test su terminale
    uart.write(data)


    b_area, b1_cx, b1_cy, b_code = tt_blue[nb-1]
    b_cx = int(b1_cx - img.width() / 2)
    b_cy = int(img.height() / 2 - b1_cy)

    if b_found == True:
        img.draw_cross(b1_cx, b1_cy)

        #b_cy = val_map(b_cx, -img.height() / 2, img.height() / 2, 99, 0)
        #b_cx = val_map(b_cy, -img.width() / 2, img.width() / 2, 0, 99)

        #compute angle
        b_angle = atan2(y_cy, y_cx) #to fix to account for camera and robot rotation
        b_angle = -90 + (b_angle * 180 / 3.14) #convert to degrees and shift
        b_angle = (b_angle+360)%360 #make sure it's all positive, so we don't have strange things happening when receiving on teensy's side
        #compute dist
        b_dist = sqrt(y_cx*y_cx + y_cy*y_cy)

        data = "{}{}{}{}{}".format("B", str(b_angle), "-", str(b_dist), "b")
    # il numero 999 indica che non ho trovato nessun blob
    else:
        data = "{}{}{}{}{}".format("B", "999", "-", "0", "b")


    #BLUE FIRST, YELLOW SECOND. ANGLE FIRST, DISTANCE SECOND
    print(str(s_b_angle) + " | " + str(s_b_dist) + "  ---  " + str(s_y_angle) + " | " + str(s_y_dist))
