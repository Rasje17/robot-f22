#!/usr/bin/env python3

from time import sleep
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor, LightSensor
from ev3dev2.led import Leds
from ev3dev2.motor import MoveSteering
import route

import numpy as np

#ev3 port setup
LEFT_MOTOR = OUTPUT_D
RIGHT_MOTOR = OUTPUT_A
FRONT_COLOUR = INPUT_2
BACK_COLOUR = INPUT_1
LIGHT = INPUT_3

# PID Values
PROPORTIONAL_GAIN = 0.5
INTEGRAL_GAIN = 0.01
DERIVATIVE_GAIN = 0.01
LOOP_INTERVAL_TIME = 0.01
RUNOFF_MAX_COUNT = 10
MOVESPEED_PROCENT = -10 # wheel's are mounted backwards

class Sokoban_robot:
    def __init__(self) -> None:
        #setting up system interfaces
        self.right_motor = LargeMotor(RIGHT_MOTOR)
        self.right_motor.polarity = LargeMotor.POLARITY_INVERSED
        self.left_motor = LargeMotor(LEFT_MOTOR)
        self.left_motor.polarity = LargeMotor.POLARITY_INVERSED
        self.steering = MoveSteering(LEFT_MOTOR, RIGHT_MOTOR)
        self.front_col = ColorSensor(FRONT_COLOUR)
        self.back_col = ColorSensor(BACK_COLOUR)
        self.front_col.mode = ColorSensor.MODE_COL_COLOR
        self.back_col.mode = ColorSensor.MODE_COL_COLOR
        self.light_sensor = LightSensor(LIGHT)

        #setter up internal variables
        self.light_readings = []
        self.light_black_intensity =0
        self.light_white_intensity = 0
        self.light_line_SetPoint = 0
        
        # find a way to handle reverse where revese uses back_col
    def _linefollower(self, stop_sensor, stop_condition, move_forward = True):
        #PID CONTROLLER
        if move_forward:
            speed = MOVESPEED_PROCENT
            #used_colour_sensor = self.front_col
        else:
            speed = -1 * MOVESPEED_PROCENT
            #used_colour_sensor = self.back_col
        previous_error = 0
        integral = 0
        runoff_counter = 0
        #while(used_colour_sensor.color != ColorSensor.COLOR_BLACK):
        while(stop_sensor.color != stop_condition):
            #print(used_colour_sensor.color)

            light_ProcessVariable = self.light_sensor.reflected_light_intensity
            #print(light_ProcessVariable)
            error = self.light_line_SetPoint - light_ProcessVariable
            integral += error
            derivative = error - previous_error
            stering_value = PROPORTIONAL_GAIN * error + INTEGRAL_GAIN * integral + DERIVATIVE_GAIN * derivative
            previous_error = error

            # move robot based on change
            #possibly change to be direct motor input
            self.steering.on(stering_value*-1, SpeedPercent(speed))

            # runoff counter
            if light_ProcessVariable >= self.light_white_intensity:
                runoff_counter += 1

                if runoff_counter >= RUNOFF_MAX_COUNT:
                    self.line_finder()
                    runoff_counter = 0
                else:
                    runoff_counter = 0


            sleep(LOOP_INTERVAL_TIME)
        self.steering.stop()
        # move forward until back is black

    #loop until front_col = black
    #error handle line loast by running linefinder and then call linefollower again


    def line_finder(self):
        light_readings = []
        #print("Left encoder %s right encoder %s" %(self.left_motor.position, self.right_motor.position))
        self.steering.on_for_degrees(-100, MOVESPEED_PROCENT, 90)
        left_encoder_start = self.left_motor.position
        right_encoder_start = self.right_motor.position
        #print("Left encoder %s right encoder %s" %(self.left_motor.position, self.right_motor.position))
        while(self.right_motor.position < right_encoder_start + 180):
            light_readings.append([self.light_sensor.reflected_light_intensity, self.left_motor.position, self.right_motor.position])
            self.steering.on(100, MOVESPEED_PROCENT)

        self.steering.stop()
        #print(light_readings)
        #print(len(light_readings))
        #print(min(light_readings[0]))
        left_decoder, right_decoder = self._determine_light_and_line_point(light_readings)

        while self.right_motor.position > right_decoder:
            self.steering.on(-100, MOVESPEED_PROCENT)
        self.steering.stop() 
        
        #line locator/ light calibrator
        #make leds yellow
        #turn in back and fourth to gather light sensor data, based on data gathered from lightsens + front coulor sense (possibly encoder data frm motors) 
        # determin line and colorsettings, 
        # turn to line
        #make leds green
        #if line not found leds, red, raise error

        pass
    def _determine_light_and_line_point(self, lightreadings):
        np_readings = np.array(lightreadings)
        readings = np_readings[: , 0]
        
        min = np.min(readings)
        max = np.max(readings)
        middle = ((max -min)/2)+ min

        self.light_white_intensity = min + ((max-min)/10)
        self.light_black_intensity = max - + ((max-min)/10)
        self.light_line_SetPoint = middle

        #calculate line start point
        #change light intencity to be the diffrence in intencity from middle
        for datapoint in np_readings:
            datapoint[0] = abs(middle - datapoint[0])
        
        # sort datapoints based om 0 value (difference from middle)
        sorted_dif_mid = np_readings[np_readings[:,0].argsort()]

        #slice to get top x closest point
        top_closest = sorted_dif_mid[:5, :]

        sort_most_right = top_closest[top_closest[:,2].argsort()]

        left_decoder_value = sort_most_right[0][1]
        right_decoder_value = sort_most_right[0][2]

        return left_decoder_value, right_decoder_value 



    def start(self):
        self.line_finder()
    
    def move_to_next_intersection(self):
        self._linefollower(self.front_col, ColorSensor.COLOR_BLACK)
        self._linefollower(self.back_col, ColorSensor.COLOR_BLACK)

    #posible to imple4ment check for overrotation based on motor encoding in the case of lightsensor data being off
    def turn(self, right = True):
        if(right):
            move_direction = -100
        else:
            move_direction = 100
        print("start tuen")
        self.steering.on_for_degrees(move_direction, MOVESPEED_PROCENT, 90)
        while(self.front_col.color != ColorSensor.COLOR_BLACK):
            self.steering.on(move_direction, MOVESPEED_PROCENT)
        print("passt black detection")
        while(self.light_sensor.reflected_light_intensity > self.light_line_SetPoint):
            self.steering.on(100, MOVESPEED_PROCENT)
        self.steering.stop()

    
    def deliver_can(self):
        self._linefollower(self.front_col, ColorSensor.COLOR_BLACK)
        self._linefollower(self.back_col, ColorSensor.COLOR_BLACK, move_forward=False)
        

    def run_track(self, track_solution):
        self.line_finder()
        for direction in track_solution:
            if direction == "f":
                self.move_to_next_intersection()
            elif direction == "r":
                self.turn()
            elif direction == "l":
                self.turn(right=False)
            elif direction == "d":
                self.deliver_can()
            else:
                raise ValueError ("input not supported")

        pass
if __name__ =="__main__":
    test = Sokoban_robot()
    #test.linefollower()
    #test.start()
    #test.move_to_next_intersection()
    #test.turn(right=False)
    #test.deliver_can()
    test.run_track(route.test_route)