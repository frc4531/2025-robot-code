import time

import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard


class LedSubsystem(SubsystemBase):
    # Create a new ArmSubsystem
    led_buffer = 85

    def __init__(self) -> None:
        super().__init__()
        # PWM Port 1
        # Must be a PWM header, not MXP or DIO
        self.led = wpilib.AddressableLED(1)

        # LED Data
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(self.led_buffer)]

        # Store what the last hue of the first pixel is (rainbow)
        self.rainbowFirstPixelHue = 0

        # Store what the last hue of the first pixel is (pulse)
        self.pulseFirstPixelVal = 0

        # Store what the last hue of the first pixel is (pulse all)
        self.pulseAllFirstPixelVal = 0
        self.pulse_all_increasing = True

        self.blink_tracker = 0

        # Default to a length of 60, start empty output
        # Length is expensive to set, so only set it once, then just update data
        self.led.setLength(self.led_buffer)

        # Set the data
        self.led.setData(self.ledData)
        self.led.start()

    def periodic(self) -> None:
        new_game_piece = SmartDashboard.getBoolean("LED_NewGamePiece", False)
        tracking_april_tag = SmartDashboard.getBoolean("LED_TrackingGoal", False)
        april_tag_tracked =

        blink_tracker = 0

        if tracking_game_piece is True:
            self.rainbow(10)
        elif tracking_goal and shooter_on is False:
            self.rgb_blink(0, 30)
        elif tracking_goal and shooter_on is True:
            self.pulse_all(100, 3)
        elif not tracking_goal and shooter_on:
            self.pulse_all(14, 30)
        elif new_game_piece is True and self.blink_tracker < 50:
            self.rgb_blink(60, 200)
            self.blink_tracker = self.blink_tracker + 1
        else:
            self.pulse_along(14, 5)
            self.blink_tracker = 0
            SmartDashboard.putBoolean("LED_NewGamePiece", False)

        # Flash Green 20 times, quickly
        # if self.test == 0:
        #     self.rgb_blink(0, 255, 0, 0.05, 20)
        #     self.test = 1

        # Flash red quickly, constantly
        # self.rgb_blink(255, 0, 0, 0.1, 20)

        # Pulse individually along string
        # self.pulse_along(14, 5)

        # Solid color (orange)
        # self.rgb_solid(255, 64, 0)

        # Pulse Color simultaneously for all LEDs (Hues: Gold = 14, Red = 1, Green = 60, orange = 7)
        # self.pulse_all(7, 3)

        # Fill the buffer with a rainbow
        # self.rainbow(25)

        # Set the LEDs
        # self.led.setData(self.ledData)

    def rainbow(self, speed_adj):
        # For every pixel
        for i in range(self.led_buffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + (i * 180 / self.led_buffer)) % 180

            # Set the value
            self.ledData[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += speed_adj

        # Check bounds
        self.rainbowFirstPixelHue %= 180

        self.led.setData(self.ledData)

    def pulse_along(self, hue, speed_adj):
        # For every pixel
        for i in range(self.led_buffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            val = (self.pulseFirstPixelVal + (i * 180 / self.led_buffer)) % 255

            # Set the value
            self.ledData[i].setHSV(hue, 255, int(val))

        # Increase value
        self.pulseFirstPixelVal += speed_adj

        # Check bounds
        self.pulseFirstPixelVal %= 255

        self.led.setData(self.ledData)

    def pulse_all(self, hue, speed_adj):
        # For every pixel
        for i in range(self.led_buffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            val = self.pulseAllFirstPixelVal  # % 255

            if val > 255:
                val = 255
            elif val < 0:
                val = 0

            # Set the value
            self.ledData[i].setHSV(hue, 255, int(val))

        # Increase or decrease value
        if self.pulse_all_increasing:
            self.pulseAllFirstPixelVal += speed_adj
        elif not self.pulse_all_increasing:
            self.pulseAllFirstPixelVal -= speed_adj

        if self.pulseAllFirstPixelVal >= 255:
            self.pulse_all_increasing = False
        elif self.pulseAllFirstPixelVal <= 0:
            self.pulse_all_increasing = True

        # Check bounds
        # self.pulseAllFirstPixelVal %= 255

        self.led.setData(self.ledData)

    def rgb_blink(self, hue, speed_adj):
        for i in range(self.led_buffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            val = self.pulseAllFirstPixelVal  # % 255

            if val > 255:
                val = 255
            elif val < 0:
                val = 0

            # Set the value
            self.ledData[i].setHSV(hue, 255, int(val))

        # Increase or decrease value
        if self.pulse_all_increasing:
            self.pulseAllFirstPixelVal += speed_adj
        elif not self.pulse_all_increasing:
            self.pulseAllFirstPixelVal -= speed_adj

        if self.pulseAllFirstPixelVal >= 255:
            self.pulse_all_increasing = False
        elif self.pulseAllFirstPixelVal <= 0:
            self.pulse_all_increasing = True

        # Check bounds
        # self.pulseAllFirstPixelVal %= 255

        self.led.setData(self.ledData)

    def rgb_solid(self, r_val, g_val, b_val):
        for i in range(self.led_buffer):
            # Set the value
            self.ledData[i].setRGB(r_val, g_val, b_val)
        self.led.setData(self.ledData)