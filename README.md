# Obstacle-avoiding-robot-car-using-TM4C123GHPM

Obstacle Avoidance is the ability of any programmed device in motion to detect and avoid obstacles in its path. Obstacle can be detected by a robot using sensors such as IR sensor, Ultrasonic or Lidar sensors. In this project we are using a Tiva C Series microcontroller - TM4C123GH6PM as the processing unit of the car. The ultrasonic sensor reads the distance between the car and the obstacle in front of it and the microcontroller makes the car change its direction of motion so that it can avoid the obstacle detected in front of it. The ultrasonic sensor can detect obstacle upto 4 metres ahead. This helps to decide the path of motion of the car with minimum overheads

## List of Components:

1. TM4C123GH6PM Microcontroller
2. HC-SR04 ultrasonic distance sensor
3. L298N Drive Control Module
4. SG90 Servo Motors
5. 2 DC Motors
6. 18650 3.7v Battery ( 2 nos) or any power source with 3.7v output
7. Swivel wheel
8. 2 Big wheels
9. Chassis
10. Breadboard
11. Jumper Wires

## Circuit Diagram

![alt text](https://github.com/DheerajNair/Obstacle-avoiding-robot-car-using-TM4C123GH6PM/blob/main/circuit_diagram.png?raw=true)

## Assembly : 

Step 1: Connect the wheels to the DC motors.<br />
Step 2: Connect the wire from DC motor to L298N driver board<br />
Step 3: From the L298N driver board connect IN1, IN2 to PA2,PA3 of the TM4C microcontroller.<br />
Step 4: Connect IN3,IN4 from the driver board to PA5,PA6 of the TM4C microcontroller.<br />
Step 5: Connect the ground from L298N board to the breadboard( short all the grounds from the components to a single location)<br />
Step 6: Connect the ground and VCC of the servo motor to the breadboard.( have a common point for  ground and VCC respectively)<br />
Step 7: Connect the data pin of the servo motor to PA4 of the TM4C microcontroller.<br />
Step 8: Mount the ultrasonic on top of the servo motor.<br />
Step 9: Connect the ground and VCC of the servo motor to the breadboard.<br />
Step 10: Connect the TRIG and ECHO to PB4,PB6 of the TM4C microcontroller.<br />
Step 11: Connect the power source to the driver board(12V pin) ( in case of batteries, connect them serially)<br />
Step 12: Connect the 5V output from the driver board to the breadboard point where all the VCC wires are shorted.<br />
Step 13: Connect a switch to control the power (OPTIONAL)<br />

Flash the code to the TM4C microcontroller and done!

![alt text](https://github.com/DheerajNair/Obstacle-avoiding-robot-car-using-TM4C123GH6PM/blob/main/obstacle%20avoiding%20robot%20car.jpeg?raw=true)
