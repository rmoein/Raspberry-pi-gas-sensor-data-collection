##!/usr/bin/python

# Written by Reza Moein Taghavi, August 2018
# if you have any questions, contact me at rezamoeint@gmail.com

# This program collects and stores sensor data (O2, CO2, and Humidity) attached to Raspberry Pi. It also controls a
# motor pump through a relay switch.

# the sensors used are:
# COZIR CM-0123 (CO2 sensor)
# KE-25 (O2 sensor)
# DHT22 (Humidity sensor)




import RPi.GPIO as GPIO
import time, datetime, serial
import numpy as np
import Adafruit_ADS1x15


class ControlMotor:
    """
    This class controls the motor pump through the relay switch.
    """

    # run time of the motor in seconds
    run_time = 10

    def __init__(self, pin=[7]):
        """

        :param pin: it is the pin number on Raspberry Pi where the relay switch controlling the pump is attached to
        """
        self.pin = pin

    def start_motor(self):
        """

        Starts the motor
        """

        GPIO.setmode(GPIO.BOARD)


        # this is the pin where the relay board that controls the motor is attached to
        pinList = self.pin

        # in case there are multiple relay boards attached to the Pi
        for i in pinList:
            GPIO.setup(i, GPIO.OUT)
            GPIO.output(i, GPIO.HIGH)

        try:
            GPIO.output(pinList[0], GPIO.HIGH)
            GPIO.output(pinList[0], GPIO.LOW)  # turn on the relay board


        # End program cleanly with keyboard
        except KeyboardInterrupt:
            print("Quit")

            # Reset GPIO settings
            GPIO.cleanup()


    def stop_motor(self):

        """
        this function stops the motor from running

        """

        GPIO.setmode(GPIO.BOARD)
        # this is the pin where the relay board that controls the motor is attached to
        pinList = self.pin

        # in case there are multiple relay boards attached to the Pi
        for i in pinList:
            GPIO.setup(i, GPIO.OUT)
            GPIO.output(i, GPIO.HIGH)

        try:
            GPIO.cleanup()


        # End program cleanly with keyboard
        except KeyboardInterrupt:
            print("Quit")

            # Reset GPIO settings
            GPIO.cleanup()

    def start_and_stop_motor(self):
        """
        this function runs the motor pump for the duration of the variable run_time
        :return: none
        """
        # this is the amount of time that the motor will run
        SleepTimeL = self.run_time

        try:
            # start motor
            self.start_motor()
            time.sleep(SleepTimeL)

            # stop motor
            self.stop_motor()

        # End program cleanly with keyboard
        except KeyboardInterrupt:
            print("Quit")

            # Reset GPIO settings
            GPIO.cleanup()


class Sensors:
    """"
    this class captures and saves data from the H2, CO2, and humidity sensors
    """

    def __init__(self):
        pass

    def read_o2(self):
        """

        :return: output of o2 sensor
        """

        # Create an ADS1115 ADC (16-bit) instance.
        adc = Adafruit_ADS1x15.ADS1115()

        # Choose a gain of 1 for reading voltages from 0 to 4.09V.
        # Or pick a different gain to change the range of voltages that are read:
        #  - 2/3 = +/-6.144V
        #  -   1 = +/-4.096V
        #  -   2 = +/-2.048V
        #  -   4 = +/-1.024V
        #  -   8 = +/-0.512V
        #  -  16 = +/-0.256V
        # See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
        GAIN = 8

        #	read ke25
        values = adc.read_adc(1, gain=GAIN)
        o2 = (values)
        rndo2 = format(o2, '.2f')
        # o3 = str(rndo2)

        # self.print_h2_co2_values(o2)

        return rndo2

    def read_humidity(self, DHT_PIN = 27):
        """
        DHT_PIN: PIN where the DHT sensor is connected to
        :return: the humidity value collected from the DHT22 sensor
        """
        import Adafruit_DHT

        # reading data from the DHT 22 sensor
        DHT_TYPE = Adafruit_DHT.DHT22



        humidity, temp = Adafruit_DHT.read(DHT_TYPE, DHT_PIN)
        error_num = 0
        if humidity is None or temp is None:
            # this if statement makes sure data is received from the sensor. if not, it will keep going to the beginning of
            # the while loop
            error_num += 1
            if error_num > 5:
                print ("Something's wrong...please check the connection to the sensor")
        # I am putting this in a try/except block because sometimes the sensor doesn't return data so instead of
        # the program crashing, you see the error.
        try:
            humidity = format(humidity, '.2f')
        except Exception as e:
            print (e)

        return humidity


    def read_co2(self):
        """

        :return: output of co2 sensor
        """

        ser = serial.Serial("/dev/ttyUSB0")
        ser.write("K 2\r\n")
        ser.flushInput()
        time.sleep(.6)

        for i in range(2):
            ser.write("Z\r\n")
            time.sleep(.01)
            resp = ser.read(10)
            resp = resp[:8]
            fltCo2 = float(resp[2:])
            scaledCo2 = fltCo2
            rndCo2 = format(scaledCo2, '.2f')
            time.sleep(.1)

        # self.print_h2_co2_values(rndCo2)

        return rndCo2

    def print_o2_co2_humidity_values(self):
        """
        Prints the O2, CO2, and humidity sensor values. Note that O2 and CO2 are RAW sensor values that are not calibrated.

        """

        print("Starting the program...")

        while True:
            o2 = self.read_o2()
            co2 = self.read_co2()
            humidity = self.read_humidity()

            time.sleep(.5)
            # display values

            print("O2 Sensor Value: {}".format(o2))
            print("CO2 Sensor Value: {}".format(co2))
            print("Humidity Sensor Value: {}".format(humidity))

            print('-' * 30)
            print('\n')

            time.sleep(.3)

    def print_calibrated_o2_co2_humidity_values(self):
        """
        this function prints the calibrated o2 and co2 sensor values as opposed to the raw values
        which is provided by function 'print_o2_co2_humidity_values'

        Calibration data can be found on the database
        """
        print("Starting the program...")

        while True:
            o2 = float(self.read_o2()) * 0.0254 - 0.2277
            co2 = float(self.read_co2()) * 0.0012 - 0.003
            humidity = self.read_humidity()

            o2 = format(o2, '.2f')
            co2 = format(co2, '.2f')

            time.sleep(.5)

            # display values
            print("O2 Sensor percentage: {}%".format(o2))
            print("CO2 Sensor percentage: {}%".format(co2))
            print("Humidity Sensor percentage: {}%".format(humidity))

            print('-' * 30)
            print('\n')

            time.sleep(.3)

    def save_o2_co2_values(self, display_values='n', file_name='co2_o2_sensor_data'):
        """
        this function can both print (optional) and save the output values of the sensors in a .txt file

        :param display_values: if it's 'n', the values won't be displayed. if it's 'y', values will be displayed.
        :param file_name: the file where the sensor data will be appended to.

        """

        while True:

            o2 = self.read_o2()
            co2 = self.read_co2()

            time.sleep(.5)

            if display_values == 'y':
                # display values
                print("Starting the program...")

                print("O2 Sensor Value: {}".format(o2))
                print("CO2 Sensor Value: {}".format(co2))

                print('-' * 30)
                print('\n')

            with open(file_name + '.txt', 'a') as f:
                f.write(str(datetime.datetime.now()) + ', ' + co2 + ' ,' + o2 + '\n')

            time.sleep(.3)

    def co2_o2_humidity_reader_and_saver(self, save_data='n', interval = 3, length_running = 10):

        """
        Saves and prints sensor data every (interval) seconds for x amount of seconds (length_running)

        :param save_data: if it's 'y' the progeam will save the data
        :param interval: the interval where you would like to save the data (in seconds)
        :param length_running: for how long you would like the program to keep running (in seconds)

        """

        time_begin = time.time()
        while True:
            o3 = str(self.read_o2())
            strCo2 = str(self.read_co2())
            humidity = str(self.read_humidity())

            # every how many seconds do you want to view and save the data
            time.sleep(interval)

            print("O2 Sensor Value: {}".format(o3))
            print("CO2 Sensor Value: {}".format(strCo2))
            print("Humidity Sensor Value: {}".format(humidity))

            print('-' * 30)
            print('\n')

            if time.time() - time_begin <= length_running:
                if save_data == 'y':
                    with open('co2_o2_sensor_data.txt', 'a') as f:
                        f.write(str(datetime.datetime.now()) + ', ' + strCo2 + ' ,' + o3 + ', ' + humidity + '\n')
            else:
                break




class AnalyzeGasSample(ControlMotor, Sensors):

    """
    This class inherits from both ControlMotor and Sensors Classes so that it can control both the sensors and the
    motor pump
    """
    def __init__(self):
        # inheriting the above classes
        ControlMotor.__init__(self)
        Sensors.__init__(self)

    def begin_experiment(self):
        # start the pump
        ControlMotor().start_motor()
        print('Waiting for gas to reach the sensors')
        time.sleep(2)
        Sensors().co2_o2_humidity_reader_and_saver()
        ControlMotor().stop_motor()
        print('Data collection complete!')




AnalyzeGasSample().begin_experiment()