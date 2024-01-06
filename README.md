#smart traffic light

##self-diagnostics system

**this system checks if there is a camera failure**

###The first step is
changing state NRF24L0 changes from receiving to sending state to send an error message to the diagnostic chip and return to the receiving state again

###The second step is 
receiving the error message the diagnostic chip receives the error message which contains the diagnostic chip address in the Camera address and sends the to node MCU the cam address

###The third step is 
the nude MCU writes to the fire base the address of the broken cam and 
sends a pulse to the nucleo board that runs the ultrasonic system to count cars and send pulse to the Arduino board to stop the main system

###The fourth step is :
the nude MC check, the error variable in the firebase if the error variable is zero then we go back to the main system when the STM32 board receives a pulse on the off wire it will send a:
- pulse to the nuclear board to turn it off 
- ⁠pulse to turn on the Arduino to return to the main system
