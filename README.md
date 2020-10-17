# Dz3
## The Dżordż version 3

This is an Adruino based Robot experiment. It works with python PC path maker and work little bit like CNC

This project is all about experimenting with Arduino based small driving robot unit that can be controlled by set of commands sent by uart/serial communication.
The basics are:
- Robot is using 2 stepper motors oneforeach wheel
- Root is usingAccelStepper library from Ardruino ecosystem (https://www.airspayce.com/mikem/arduino/AccelStepper/)
- The set of commands can be sent by any means - now it uses a Bluetooth BLE connection 

More in depth description will be added.

## Dz3 code

The main ideaof the used communication code is based on sending 4 number sets in form:
**<C,a,b,c>**

where:

- C - the command
- a - firts parameter - in general distance to reavel in [cm]
- b - second parameter - in general angle to turn in [degrees] (in mathematical direction, means +90deg is "turn left 90deg")
- c - third parameter - in general set the speed of drive, if 0 it dont change previously set speed.

### Available commands:
- 0 - STOP, just stop the action asap. Other params ignored, can be used as <0>.
- 1 - FREE MOVE - just make the move based on te parameters.
- 9 - UNDO  MOVE - move back to previous location.Other params ignored, can be used as <9>
- 
- 20 - ADD to SEQUENCE - adding the parameters as next move in sequence. This dont make the robot moves, just add to the memory
- 21 - ADD LAST MOVE to SEQUENCE - this adds last executed move (in exampleby command 1)to the memory as next onein sequence
- 22 - MAKES MOVE& ADD to SEQ. - makes the move just like command 1 and add it to the sequence
- 29 - REMOVE LAST STORED MOVE from SEQUENCE
-
- 30 - RUN the SEQUENCE - just starts to execute memorized commands
- 31 - RESUME SEQUENCE - starts the sequence from where it was stopped (by sending <0>whileexecuting)
- 33 - RUN in LOOP - execute the seqience like command 30 but in infinite loop (can be named "patrol mode")

### Examples of commands:
- <1,100,0,0> - move 1m ahead
- <1,0,360,0> - make 360 in place
- <21> - memorize last executed move
- <22, 100,90,300>- move 1m turning left (making arc) slowly (the 300) and savethis movein sequence
- ...

## The python app

The python app allows to draw path and ten convert it to the set of commands that are sent via comm port to the Dz3 as sequence steps.
More details will be added here...

to be continued...
