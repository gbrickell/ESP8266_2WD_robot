# ESP8266 2WD Robot
 A 2WD robot using an ESP8266 NodeMCUv1.0 microcontroller module to:
 - drive two motors controlled using a L293 ESP-12E motor drive module;
 - provide object detection using a HC-SR04P ultrasonic sensor (the 3V3 version!); and
 - to provide system control/operation using 3 slide switches.

 ![ESP8266 2WD robot](https://onlinedevices.org.uk/dl1209?display&x=253&y=250)  &nbsp; &nbsp;  ![ESP8266 2WD robot](https://onlinedevices.org.uk/dl1208?display&x=267&y=250)   

 This project is documented in some more detail [here](https://onlinedevices.org.uk/ESP8266_2WD_Robot) and a basic User Guide, plus a set of templates for cut-outs to decorate the robot, are downloadable from the documentation folder at this repository.
 
 The designs for all the 3D prints are also available from [here](https://www.prusaprinters.org/prints/67808-esp8266-2wd-robot-components).

 The main code (NMms_web_server_20.ino), developed on the Arduino IDE, uses the 3 slide switches to 'set' a particular operational mode and uses a set of key parameters held in SPIFFS files that are set up with the separate short initialisation program (SPIFFS_setup01.ino).

A 'plug-replaceable' upgrade to this project to use the ESP32 microcontroller has now been developed and is documented [here](https://onlinedevices.org.uk/ESP32_upgrade_to_ESP8266_2WD_Robot), with the GitHub repository for its updated software and the custom PCB designs being accessible [here](https://github.com/gbrickell/ESP32_upgrade_to_ESP8266_2WD_robot).
