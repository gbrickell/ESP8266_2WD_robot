# ESP8266 2WD Robot
 A 2WD robot using a ESP8266 NodeMCUv1.0 microcontroller module to:
 - drive two motors controlled using a L293 ESP-12E motor drive module;
 - provide object detection using a HC-SR04P ultrasonic sensor (the 3V3 version!); and
 - to provide system control/operation using 3 slide switches

 ![ESP8266 2WD robot](https://onlinedevices.co.uk/dl1209?display&x=253&y=250)    ![ESP8266 2WD robot](	https://onlinedevices.co.uk/dl1208?display&x=267&y=250)   

 This project is documented in some more detail at https://onlinedevices.co.uk/ESP8266+2WD+Robot and a basic User Guide is downloadable from this repository.
 
 The designs for all the 3D prints are also available from https://www.prusaprinters.org/prints/67808-esp8266-2wd-robot-components

 The code, developed on the Arduino IDE, uses the 3 slide switches to 'set' a particular operational mode and uses a set of key parameters held in SPIFFS files that are set up with a separate short initialisation program.

