"""
Consume LIDAR measurement file and create an image for display.
 
Written by Denis FOLLEREAU
Copyright (c) 2021
Licensed under the MIT license.
"""
# RPLIDAR example: https://learn.adafruit.com/slamtec-rplidar-on-pi?view=all

import os
from math import cos, sin, pi, floor
from adafruit_rplidar import *
import RPi.GPIO as GPIO
import time
from serial import SerialException

# GPIO to LCD mapping
LCD_RS = 7 # Pi pin 26
LCD_E = 8 # Pi pin 24
LCD_D4 = 25 # Pi pin 22
LCD_D5 = 24 # Pi pin 18
LCD_D6 = 23 # Pi pin 16
LCD_D7 = 18 # Pi pin 12
# Device constants
LCD_CHR = True # Character mode
LCD_CMD = False # Command mode
LCD_CHARS = 16 # Characters per line (16 max)
LCD_LINE_1 = 0x80 # LCD memory location for 1st line
LCD_LINE_2 = 0xC0 # LCD memory location 2nd line
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # Use BCM GPIO numbers
GPIO.setup(LCD_E, GPIO.OUT) # Set GPIO's to output mode
GPIO.setup(LCD_RS, GPIO.OUT)
GPIO.setup(LCD_D4, GPIO.OUT)
GPIO.setup(LCD_D5, GPIO.OUT)
GPIO.setup(LCD_D6, GPIO.OUT)
GPIO.setup(LCD_D7, GPIO.OUT)

def lcd_init():
    lcd_write(0x33,LCD_CMD) # Initialize
    lcd_write(0x32,LCD_CMD) # Set to 4-bit mode
    lcd_write(0x06,LCD_CMD) # Cursor move direction
    lcd_write(0x0C,LCD_CMD) # Turn cursor off
    lcd_write(0x28,LCD_CMD) # 2 line display
    lcd_write(0x01,LCD_CMD) # Clear display
    time.sleep(0.0005) # Delay to allow commands to process

def lcd_toggle_enable():
    time.sleep(0.0005)
    GPIO.output(LCD_E, True)
    time.sleep(0.0005)
    GPIO.output(LCD_E, False)
    time.sleep(0.0005)

def lcd_write(bits, mode):
    # High bits
    GPIO.output(LCD_RS, mode) # RS
    GPIO.output(LCD_D4, False)
    GPIO.output(LCD_D5, False)
    GPIO.output(LCD_D6, False)
    GPIO.output(LCD_D7, False)
    if bits&0x10==0x10:
        GPIO.output(LCD_D4, True)
    if bits&0x20==0x20:
        GPIO.output(LCD_D5, True)
    if bits&0x40==0x40:
        GPIO.output(LCD_D6, True)
    if bits&0x80==0x80:
        GPIO.output(LCD_D7, True)
    # Toggle 'Enable' pin
    lcd_toggle_enable()
    # Low bits
    GPIO.output(LCD_D4, False)
    GPIO.output(LCD_D5, False)
    GPIO.output(LCD_D6, False)
    GPIO.output(LCD_D7, False)
    if bits&0x01==0x01:
        GPIO.output(LCD_D4, True)
    if bits&0x02==0x02:
        GPIO.output(LCD_D5, True)
    if bits&0x04==0x04:
        GPIO.output(LCD_D6, True)
    if bits&0x08==0x08:
        GPIO.output(LCD_D7, True)
    # Toggle 'Enable' pin
    lcd_toggle_enable()


def lcd_text(message,line):
    # Send text to display
    message = message.ljust(LCD_CHARS," ")
    lcd_write(line, LCD_CMD)
    for i in range(LCD_CHARS):
        lcd_write(ord(message[i]),LCD_CHR)

def process_data(data, max_distance):
    front = 0
    right = 0
    back = 0
    left = 0

    for angle in range(360):
        distance = data[angle]
        if angle==0:
            front = distance / 10
        if angle==90:
            right = distance / 10
        if angle==180:
            back = distance / 10
        if angle==270:
            left = distance / 10

    print("**** RPLIDAR measure **** : [FRONT: "+str(front)+"cm] [RIGHT: "+str(right)+"cm] [BACK: "+str(back)+"cm] [LEFT: "+str(left)+"cm]")
    lcd_text("F:"+str(int(front))+"cm R:"+str(int(right))+"cm", LCD_LINE_1)
    lcd_text("B:"+str(int(back))+"cm L:"+str(int(left))+"cm", LCD_LINE_2)

if __name__ == "__main__":
    # used to scale data to fit on the screen
    max_distance = 0
    scan_data = [0]*360
    # Initialize display
    lcd_init()
    # Setup the RPLidar
    PORT_NAME = '/dev/ttyUSB0'
    lidar = RPLidar(None, PORT_NAME)
    
    try:
        print(lidar.info) # printed once! Laser device infos

        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, floor(angle)])] = distance
            process_data(scan_data, max_distance)

    except KeyboardInterrupt:
        print ("\nCtrl-C pressed from on_message()  Stopping GPIO and exiting properly...")
    except (GMachineException) as e:
        print('ERROR ' + str(e))
    except OSError as err:
        print("OS error: {0}".format(err))
    except FileNotFoundError:
        print("File does not exist")
    except serial.serialutil.SerialException as e:
        print("USB serial error")
        print(str(e))
    except adafruit_rplidar.RPLidarException as e:
        print(str(e))
    except ValueError:
        print("Could not convert data to an integer.")
    except Exception as e:
        print('ERROR ' + str(e))
    except:
        print("Unexpected error:", sys.exc_info()[0])
    finally:
        lcd_text("Prg interrupted!", LCD_LINE_1)
        lcd_text("", LCD_LINE_2)
        GPIO.cleanup()
        lidar.stop()
        lidar.disconnect()