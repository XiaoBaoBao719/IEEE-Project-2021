# IEEE-Project-2021
Social Robot Otto Project for UC Davis IEEE Organization

# Social Robots
Over the course of the COVID-19 pandemic, many individuals (myself included) felt quite isolated from our friends and loved ones. While it was obviously possible for individuals to interact with one another virtually due to the countless digital peripherals that we have at our disposal, it's often important for humans to have phyiscal interaction. This comes in many forms, such as pets; but there's also potential for us to interact with synthetic buddies. Enter social robots like Otto.

## What is Otto?
Otto is a small friendly social robot that was developed by Nestworks based largely on the designs and programs of BoB the Biped https://www.instructables.com/BoB-the-BiPed/. The design is entirely open-source and was therefore a good candidate for a personal project due to the wealth of documentation available. 

![Image of Otto](https://content.instructables.com/ORIG/FOF/WVU6/JN3HXMRV/FOFWVU6JN3HXMRV.jpg?auto=webp&frame=1&fit=bounds&md=7b16cb41988c6ae1c740bc964af5e166)

## Building Otto
In order to construct my own version of Otto, I used the following supplies:
* Arduino Uno
* Adafruit 16-Channel PWM Servo Driver
* 4x 4k Ohm Variable Potentiometers
* 4x SG92R Tower Pro Servo Motors
* 9V DC Female Header Connector

Some of these components were common hobbyist parts that I had lying around but several of the key components, most notably the Adafruit PWM Driver, were generously provided by the UC Davis IEEE Chapter for usuage. 

Here are some of the parts up close!

![Image of project](https://raw.githubusercontent.com/XiaoBaoBao719/IEEE-Project-2021/main/Pictures/IMG_20210528_013522839.jpg)

The Arduino Uno is an ATmega32P open-source microcontroller that has 14 digital I/O pins, 5 analog pins, 2 interrupt pins, and a 16MHz resonator timer that allows for users to perform AVR interrupts. It's a versatile MCU that forms the "brains" of the robot by handling user input and passing along relevant signals for processing in the servo driver. 

![Image of Arduino](https://raw.githubusercontent.com/XiaoBaoBao719/IEEE-Project-2021/main/Pictures/IMG_20210528_014017896.jpg)

Next we have the Adafruit 16-Channel PWM Servo Driver. This component is probably the next most crucial element in the entire robot since without it, the servo motors would not be able to move properly. See, Servo motors are essentially small DC motors with a small integrated encoder built into the entire assembly. In other words, the complex circutry that has to keep track of the DC motor's position, velocity, and voltage is all built into the servo motor. All we need to provide to the servo motor is a signal. This signal is called the *Pulse Width Modulation* signal; and is a square wave. The width of the square wave is known as the *duty cycle* and by varying the duty cycle, we can change the PWM signal as well. 

![Image of Driver](https://raw.githubusercontent.com/XiaoBaoBao719/IEEE-Project-2021/main/Pictures/IMG_20210528_014052153.jpg)

We also need our means of controlling the servo motors directly. So to address that, I quickly wired four variable resistor potentiometers into the Arduino's analog input pins so we can read signals from the potentiometers. 

(https://raw.githubusercontent.com/XiaoBaoBao719/IEEE-Project-2021/main/Pictures/IMG_20210528_014251615.jpg)
(https://raw.githubusercontent.com/XiaoBaoBao719/IEEE-Project-2021/main/Pictures/IMG_20210528_014258858.jpg)
(https://raw.githubusercontent.com/XiaoBaoBao719/IEEE-Project-2021/main/Pictures/IMG_20210528_014306499.jpg)

