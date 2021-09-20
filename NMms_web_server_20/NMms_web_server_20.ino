// ESP8266 2WD robot project
// NMms_web_server_20.ino - web server version 20 for the NodeMCU + motor shield robot
//  code developed and compiled using the Arduino IDE v1.8.5
//  Node MCU 1.0 (ESP-12E Module) set as the Board in the IDE - using board version 2.7.4
//  Flash Size set to: 4MB(FS:3MB OTA:~512kB)  - other setting will allocate different amounts of the 4MB Falsh to SPIFFS
//   lots of other Board settings! but none experimented with

// SPIFF memory initially set up using SPIFFS_setup01.ino

// different versions of this code can be used for different robots where the following hard coded parameters are changed:
// - line 40:    int IPnum = 10;
// - line 54:    String namehost = "NodeMCU01";
// - line 245:   Serial.println("program version 20: starting ...........");
// - line 1333:  Serial.println(WiFi.softAP("ESPsoftAPhome", "pswd12345") ? "Ready" : "Failed!");

// code uses the slide switch settings to determine what action is taken
// ** opmodes for 3 switches, 1 of which one is the GO/STOP **
// opmode = 4 means AD  softAP web server mode
// opmode = 5 means AC  local WiFi web server mode
// opmode = 6 means BC  autonomous run around mode
// opmode = 8 means BD  demo mode
// opmode = 9 means an undefined operational mode

// both options 4 and 5 use a web server to provide a web interface to control the motors, update files, etc: 
//  now with:
//  - auto stop added to the run-around mode and extensive use of default parameters stored in SPIFFS
//  - demo mode run from a 'action' list file that can be edited from a web page
//  - auto run  and demo run added to web options

#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <WiFiClient.h>
#include <ESP8266mDNS.h>        // Include the mDNS library
#include <ESP8266WebServer.h>
#include <FS.h>

// some WiFi variables set here to be global but the WiFi config is done in 
//   the void loop so that it can be either SoftAP or connection to a local WiFi

// IP addresses for the **soft AP** operation option 4 (AD) switch setting
int IPnum = 10;
IPAddress local_IP(10,0,5,IPnum);     // using a 10.x.x.x IP address space
IPAddress gateway(10,0,5,1);
IPAddress subnet(255,255,255,0);

ESP8266WebServer server(80);   //instantiate the web server at port 80 (http port#)

// -------------------------------------------------------
// define all the main variables here to make them global
//  most of these variables need ONLY be defined here as 
//   their values are 'read' later from SPIFFS files
// -------------------------------------------------------

// hostname as a variable
String namehost = "NodeMCU01";

// network credentials to connect to local WiFi networks for the option 5 (AC) switch setting
//  these variables are simply initialised here since it is assumed that all the values have
//  already been 'written' to the appropraite SPIFFS files or will be populated via the web 
//  interface using the softAP option
String ssid_selected ="";
int NSSIDs = 5;   // number of local WiFi credentials that can be set up: the first one found is used
String ssid1 = " ";
String ssid2 = " ";
String ssid3 = " ";
String ssid4 = " ";
String ssid5 = " ";
String password1 = " ";
String password2 = " ";
String password3 = " ";
String password4 = " ";
String password5 = " ";

// demo action variables: option 8 - slide switch BD, runs these actions on a continuous basis when in GO mode
int num_demo = 40;              // maximum number of demo actions allowed
String demo_controls[40] ={};   // pairs of strings in an array that define the demo actions
String demo_actions[8] ={};     // array of all the potential demo action command strings

// variable to indicate whether either of the two WiFi options have been set up
// this is needed since the setup is different per switch option, so the set up
//   must be done within the "void loop()" and reset if the switch settings change
const char* WiFiup = "no";

String page_content = "";   // initialised global variable for web page content
String web_server = "off";  // 'off' just means the robot is in logical STOP mode so a limited web page is shown

// ************************************************
// define all the motor operation variables
//  again just initialised here as the values are
//  assumed to be 'read' from SPIFFS files
// ************************************************
int turntime;     // time in ms to do a 90-deg turn (read later from SPIFFS)
int spintime;     // time in ms to do a 90-deg spin (read later from SPIFFS)
int turnspeed;    // turning speed as a % (read later from SPIFFS)
int spinspeed;    // spinning speed as a % (read later from SPIFFS)

// power adjustment for left/right motors (read later from SPIFFS) i.e.
// set motorL to <1 if turning to the right or 
//     motorR to <1 if turning to the left
float motorL;   
float motorR;

int r_speed;         // default speed % as an integer value  0-100%

// ** motor drive **
// pins used when NodeMCUv1.0 is inserted into the motor drive module - no need
//   to set these as variables as the motor drive commands always use these pins
// pin [D1] GPIO#5 -  motor A: HIGH for 100% forward or INT 0-1000 for PWM%
// pin [D2] GPIO#4 -  motor B: HIGH for 100% forward or INT 0-1000 for PWM%
// pin [D3] GPIO#0 -  motor A direction: LOW = FWD - HIGH = BACK
// pin [D4] GPIO#2 -  motor B direction: LOW = FWD - HIGH = BACK

// motor A wired as the left motor when facing forwards
// motor B wired as the right motor when facing forwards


// ***************************************************************
// set the robot state variable used in the HTML page generation
//  for various logic checks e.g.  to colour the buttons
// 0 means STOP
// 1 means FORWARDS
// 2 means BACKWARDS
// 3 means EMERGENCY STOP
// ***************************************************************
int robot_state = 0;

// *************************************
// set all the slide switch variables
// *************************************
int s_debug = 1;       // 1 = debug on  0 = debug off
int num_switches = 3;  // this variable is hard-coded as either 3 or 4 so that the same 
                       //  logic/function can be used with different numbers of switches
                       //  the ESP8266/NodeMCU robot build can only use 3 switches 
                       //  because of thge limited number of avauilable GPIO pins
int opmode = 0;        // set to an initial IDLE state
int opmode_last = 10;  // set to an initial impossible value
int swmode = 0;        // set to an initial OFF state
int swmode_last = 10;  // set to an initial impossible value

// set slide switch pin variables
//  these are fully set initially here otherwise nothing can be operated!!
//   even though all the GPIO pins can (exceptionally!) be updated
//   via the web interface if for some reason the robot is rewired
int s_AB  = 14;
int s_CD  = 12;
int s_EF  = 3;   // not used in early NodeMCU builds so num_switches should be set to 3
int onoff = 16;

// set initial switch states to something undefined so that the program checks can run from the start
int state_onoff = 2;
int state_AB = 2;
int state_CD = 2;
int state_EF = 2;

// set which switch setting modes are active, i.e. programmed (1) or inactive (0)
//  will need to updated in the code if more switch setting options are coded
int mode_1 = 0;
int mode_2 = 0;
int mode_3 = 0;
int mode_4 = 1;  // softAP web interface
int mode_5 = 1;  // local WiFi web interface
int mode_6 = 1;  // autonomous run around
int mode_7 = 0; 
int mode_8 = 1;  // demo mode

// ***************************
// ultrasonic sensor variables
// ***************************
int trigPin = 13;   // pin [D7] GPIO#13 used for the sensor initiate pin (trig)
int echoPin = 15;   // pin [D8] GPIO#15 used for the sensor response pin (echo)
unsigned long duration, inches;
int indec, cmdec;
int inchconv = 147; // ratio between pulse width and inches
int cmconv = 59;    // ratio between pulse width and cm
int avoid_distance = 20;   // set a defined distance to take avoidance action
int stop_distance = 10;    // set the emergency stop distance for web use
int avoid_limit = 3;       // number of times distance limit is reached before action taken - avoids spurious readings
int avoid_count = 0;       // number of times that distance limit is reached
int cm, lcm, rcm;
const char* leftright = "right";
int avoidspintime;         // time in ms to spin in the avoidance routine
int ReverseTime;           // time in ms to reverse in the avoidance routine
String stopauto = "yes";   // set whether autostop should be used
String cm_str = "";

// ********************************************************
// String versions of the various integer variables
// - easier to deal with in SPIFFS and web pages! even 
//   though this creates overhead with various conversions
// ********************************************************
String str_NSSIDs;
String str_mode_1;
String str_mode_2;
String str_mode_3;
String str_mode_4;
String str_mode_5;
String str_mode_6;
String str_mode_7;
String str_mode_8;

String str_num_switches;
String str_s_AB;
String str_s_CD;
String str_s_EF;
String str_onoff;

String str_trigPin;
String str_echoPin;

String str_inchconv;
String str_cmconv;
String str_avoid_distance;
String str_stop_distance;
String str_avoid_limit;
String str_avoidspintime;
String str_turntime;
String str_spintime;
String str_turnspeed;
String str_spinspeed;
String str_r_speed;
String str_ReverseTime;

String str_motorL;   
String str_motorR;

// *********************
// web page variables
// *********************
String header_content;



// *****************************
// ** initial set up function **
// *****************************
void setup() {

    // ******************************************************
    // start serial monitor for all the 'printed' debug info
    // ******************************************************
    Serial.begin(115200);
    delay(2000);              // short pause for the Serial Monitor to be set up before printing
    Serial.println("    ");   // do a few prints to the Serial Monitor as the first few are sometimes missed
    Serial.println("    ");
    Serial.println("    ");
    Serial.println("program version 20: starting ...........");

    // ********************************
    // start the SPI Flash File System
    // ********************************
    if (!SPIFFS.begin())
    {
        // Serious problem
        Serial.println("SPIFFS mount failed");
    } else {
        Serial.println("SPIFFS mounted OK");
    }

    // ************************************
    // List all the files in SPIFFS
    // ************************************
    String filestr = "";
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {
        filestr += dir.fileName();
        filestr += " / ";
        filestr += dir.fileSize();
        filestr += "\r\n";
    }
    Serial.println("SPIFFS file listing start:");
    Serial.print(filestr);
    Serial.println("SPIFFS file listing end:");
    Serial.println("  ");

    // *****************************************
    // List all the SPIFFS system info (FSinfo)
    // *****************************************

    FSInfo fs_info;
    SPIFFS.info(fs_info);
    printf("SPIFFS: %lu of %lu bytes used.\n",
         fs_info.usedBytes, fs_info.totalBytes);
    printf("SPIFFS blockSize: %lu \n",
         fs_info.blockSize);
    printf("SPIFFS pageSize: %lu \n",
         fs_info.pageSize);
    printf("SPIFFS maxOpenFiles: %lu \n",
         fs_info.maxOpenFiles);
    printf("SPIFFS maxPathLength: %lu \n",
         fs_info.maxPathLength);

    // **************************************************************
    // read all the default setting data from the SPIFFS text files
    //  with the exception of the slide switch GPIO pins all these 
    //  files are assumed to already exist - further code versions 
    //  could make this more robust !    
    // **************************************************************
    Serial.println("-------------------------------");
    read_strings("/demo_controls.txt", demo_controls, num_demo);
    Serial.println("-------------------------------");
    Serial.println("reprint read demo_controls");
    for (int i=0; i<=num_demo-1; i++){
        Serial.print(i);
        Serial.print(":");
        Serial.print(demo_controls[i].length());
        Serial.print(":");
        Serial.println(demo_controls[i]);
    }
   
    Serial.println("-------------------------------");
    demo_actions[0] = "FWD";
    demo_actions[1] = "BACK";
    demo_actions[2] = "DELAY";
    demo_actions[3] = "STOP";
    demo_actions[4] = "SPINL";
    demo_actions[5] = "SPINR";
    demo_actions[6] = "TURNL";
    demo_actions[7] = "TURNR";

    // check demo_controls has sensible demo_actions
    Serial.println("checking demo_controls");
    for (int i=0; i<=num_demo-1; i=i+2){
        for (int j=0; j<=7; j=j+1) {
            if (demo_controls[i] == demo_actions[j]) {
                Serial.print(demo_actions[j]); 
                Serial.println(" action found");
                break;
            }
        }
    }
    Serial.println("demo_controls check finished");    

    Serial.println("-------------------------------"); 
    str_NSSIDs = read_text("/str_NSSIDs.txt");
    Serial.println("-------------------------------");
    ssid1 = read_text("/ssid1.txt");
    ssid2 = read_text("/ssid2.txt");
    ssid3 = read_text("/ssid3.txt");
    ssid4 = read_text("/ssid4.txt");
    ssid5 = read_text("/ssid5.txt");
    Serial.println("-------------------------------");
    password1 = read_text("/password1.txt");
    password2 = read_text("/password2.txt");
    password3 = read_text("/password3.txt");
    password4 = read_text("/password4.txt");
    password5 = read_text("/password5.txt");
    Serial.println("-------------------------------");
    str_r_speed = read_text("/str_r_speed.txt");
    Serial.println("-------------------------------");
    str_mode_1 = read_text("/str_mode_1.txt");
    str_mode_2 = read_text("/str_mode_2.txt");
    str_mode_3 = read_text("/str_mode_3.txt");
    str_mode_4 = read_text("/str_mode_4.txt");
    str_mode_5 = read_text("/str_mode_5.txt");
    str_mode_6 = read_text("/str_mode_6.txt");
    str_mode_7 = read_text("/str_mode_7.txt");
    str_mode_8 = read_text("/str_mode_8.txt");
    Serial.println("-------------------------------");

    if (SPIFFS.exists("/str_num_switches.txt")) {
        str_num_switches = read_text("/str_num_switches.txt");
    } else {
        str_num_switches = String(num_switches);
        write_text("/str_num_switches.txt", str_num_switches);
    }

    if (SPIFFS.exists("/str_s_AB.txt")) {
        str_s_AB = read_text("/str_s_AB.txt");
    } else {
        str_s_AB = String(s_AB);
        write_text("/str_s_AB.txt", str_s_AB);
    }

    if (SPIFFS.exists("/str_s_CD.txt")) {
        str_s_CD = read_text("/str_s_CD.txt");
    } else {
        str_s_CD = String(s_CD);
        write_text("/str_s_CD.txt", str_s_CD);
    }

    if (SPIFFS.exists("/str_s_EF.txt")) {
        str_s_EF = read_text("/str_s_EF.txt");
    } else {
        str_s_EF = String(s_EF);
        write_text("/str_s_EF.txt", str_s_EF);
    }

    if (SPIFFS.exists("/str_onoff.txt")) {
        str_onoff = read_text("/str_onoff.txt");
    } else {
        str_onoff = String(onoff);
        write_text("/str_onoff.txt", str_onoff);
    }

    Serial.println("-------------------------------");
    str_trigPin = read_text("/str_trigPin.txt");
    str_echoPin = read_text("/str_echoPin.txt");
    Serial.println("-------------------------------");
    str_inchconv = read_text("/str_inchconv.txt");
    str_cmconv = read_text("/str_cmconv.txt");
    str_avoid_distance = read_text("/str_avoid_distance.txt");
    str_stop_distance = read_text("/str_stop_distance.txt");
    str_avoid_limit = read_text("/str_avoid_limit.txt");
    str_avoidspintime = read_text("/str_avoidspintime.txt");
    str_turntime = read_text("/str_turntime.txt");
    str_spintime = read_text("/str_spintime.txt");
    str_turnspeed = read_text("/str_turnspeed.txt");
    str_spinspeed = read_text("/str_spinspeed.txt");
    str_r_speed = read_text("/str_r_speed.txt");
    str_motorL = read_text("/str_motorL.txt");
    str_motorR = read_text("/str_motorR.txt");
    str_ReverseTime = read_text("/str_ReverseTime.txt");
    stopauto = read_text("/str_stopauto.txt");
    Serial.println("-------------------------------");

    // ********************************************************************
    // convert the integer and float strings back into integers and floats
    // ********************************************************************
    mode_1 = str_mode_1.toInt();
    mode_2 = str_mode_2.toInt();
    mode_3 = str_mode_3.toInt();
    mode_4 = str_mode_4.toInt();
    mode_5 = str_mode_5.toInt();
    mode_6 = str_mode_6.toInt();
    mode_7 = str_mode_7.toInt();
    mode_8 = str_mode_8.toInt();

    num_switches = str_num_switches.toInt();
    s_AB = str_s_AB.toInt();
    s_CD = str_s_CD.toInt();
    s_EF = str_s_EF.toInt();
    onoff = str_onoff.toInt();

    trigPin = str_trigPin.toInt();
    echoPin = str_echoPin.toInt();

    inchconv = str_inchconv.toInt();
    cmconv = str_cmconv.toInt();
    avoid_distance = str_avoid_distance.toInt();
    stop_distance = str_stop_distance.toInt();
    avoid_limit = str_avoid_limit.toInt();
    avoidspintime = str_avoidspintime.toInt();
    turntime = str_turntime.toInt();
    spintime = str_spintime.toInt();
    turnspeed = str_turnspeed.toInt();
    spinspeed = str_spinspeed.toInt();
    r_speed = str_r_speed.toInt();
    motorL = str_motorL.toFloat();
    motorR = str_motorR.toFloat();
    ReverseTime = str_ReverseTime.toInt();

    // *****************************************************
    // prepare Motor control pins with both motors 'stopped'
    // *****************************************************
    pinMode(5, OUTPUT);
    digitalWrite(16, LOW);
  
    pinMode(4, OUTPUT);
    digitalWrite(5, LOW);
  
    pinMode(0, OUTPUT);
    digitalWrite(4, LOW);
  
    pinMode(2, OUTPUT);
    digitalWrite(0, LOW);

    // ***************************************
    // prepare ultrasonic sensor control pins 
    // ***************************************
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    Serial.println();
    Serial.println("Ready to start auto-stop web server tests...");

    // ****************************************************************
    // interrupt 'handlers' triggered by various Web server requests:
    // ****************************************************************
    // ** do this if web root i.e. / is requested
    server.on("/", handle_root);

    // ****** main selection actions ******
    // ** do this if /run_about is requested
    server.on("/run_about", handle_run_about);
    // ** do this if /auto_run is requested
    server.on("/auto_run", handle_auto_run);
    // ** do this if /demo_run is requested
    server.on("/demo_run", handle_demo_run);
    // ** do this if /run_tests is requested
    server.on("/run_tests", handle_run_tests);
    // ** do this if /parameters is requested
    server.on("/parameters", handle_parameters);
    // ** do this if /sysinfo is requested
    server.on("/sysinfo", handle_sysinfo);


    // ****** sub selection parameter update actions ******
    // ** do this if /WiFi_params is requested
    server.on("/WiFi_params", handle_WiFi_params);
    // ** do this if /robot_updates is requested
    server.on("/robot_updates", handle_robot_updates);
    // ** do this if /GPIO_updates is requested
    server.on("/GPIO_updates", handle_GPIO_updates);
    // ** do this if /demo_updates is requested
    server.on("/demo_updates", handle_demo_updates);

    // ****** detailed parameter update submission actions ******
    // ** do this if /WiFi_updates1 is requested
    server.on("/WiFi_updates1", handle_WiFi_updates1);
    // ** do this if /WiFi_updates2 is requested
    server.on("/WiFi_updates2", handle_WiFi_updates2);
    // ** do this if /WiFi_updates3 is requested
    server.on("/WiFi_updates3", handle_WiFi_updates3);
    // ** do this if /WiFi_updates4 is requested
    server.on("/WiFi_updates4", handle_WiFi_updates4);
    // ** do this if /WiFi_updates5 is requested
    server.on("/WiFi_updates5", handle_WiFi_updates5);

    // ** do this if /robot_params is requested
    server.on("/robot_params", handle_robot_params);
    // ** do this if /GPIO_pins is requested
    server.on("/GPIO_pins", handle_GPIO_pins);
    // ** do this if /demo_file is requested
    server.on("/demo_file", handle_demo_file);

    // ****** robot auto run control actions ******
    // ** do this if /run_auto is requested
    server.on("/run_auto", handle_run_auto);
    // do this if /run_stop is requested
    server.on("/run_stop", handle_run_stop);
    // ****** web input actions ******
    // do this if /autospeed is requested as part of an input form
    server.on("/autospeed", handle_autospeed);

    // ****** robot demo run control actions ******
    // ** do this if /run_demo is requested
    server.on("/run_demo", handle_run_demo);
    // do this if /stop_demo is requested
    server.on("/stop_demo", handle_stop_demo);

    // ****** robot run about control actions ******
    // ** do this if /forward is requested
    server.on("/forward", handle_forward);
    // do this if /backward is requested
    server.on("/backward", handle_backward);
    // do this if /halt is requested
    server.on("/halt", handle_stop);
    // do this if /turnleft is requested
    server.on("/turnleft", handle_turnleft);
    // do this if /turnright is requested
    server.on("/turnright", handle_turnright);
    // do this if /spinleft is requested
    server.on("/spinleft", handle_spinleft);
    // do this if /spinright is requested
    server.on("/spinright", handle_spinright);
    // ****** web input actions ******
    // do this if /postspeed is requested as part of an input form
    server.on("/postspeed", handle_postspeed);

    // ****** component testing control actions ******
    // ** do this if /testspeed is requested
    server.on("/testspeed", handle_testspeed);
    // ** do this if /leftfwd is requested
    server.on("/leftfwd", handle_leftfwd);
    // ** do this if /leftback is requested
    server.on("/leftback", handle_leftback);
    // ** do this if /rightfwd is requested
    server.on("/rightfwd", handle_rightfwd);
    // ** do this if /rightback is requested
    server.on("/rightback", handle_rightback);
    // ** do this if /teststop is requested
    server.on("/teststop", handle_teststop);
    // ** do this if /USread is requested
    server.on("/USread", handle_USread);

    // ** start web server
    server.begin();
    Serial.println("Web server started!");

    // ** populate the common header HTML used in all web pages
    header_content = HTMLheader();
}



// ***********************************
// ** continuous operation function **
// ***********************************
void loop() {
    // the code here runs repeatedly in a loop

    // get the latest swmode and opmodes from the switch settings
    swmode = check_onoff(swmode_last, s_debug, onoff);
    opmode = check_slideswitch(num_switches, opmode_last, s_debug, s_AB, s_CD, s_EF);
    if (opmode != opmode_last) {  // print out the opmode if it has changed but don't reset so it can be done later
        Serial.print (" new switch settings and therefore new opmode: ");
        Serial.println(opmode);
        // but change WiFi status since this may have to be reset
        WiFiup = "no";
    }

    // ** only opmodes 4, 5, 6 & 8 are possible with 3 switches so only these are currently checked in the code **

    // ##############################################################
    // ## opmode 4 inactive: run web server with the softAP option ##
    // ##############################################################
    if (opmode == 4 and swmode == 0) {
        stop();  // stop the motors just in case something has been left running
        if (mode_4 == 0) {
            if (opmode_last != 4 or swmode_last !=0) {
                Serial.println ("opmode 4 - (AD) not set up yet - CODE INACTIVE ");
                opmode_last = 4;
                swmode_last = 0;
            }
        } else {           
            if  (opmode_last != 4 or swmode_last !=0 ) {           
                Serial.println ("opmode 4 - 3 switches: (AD) web server operation - in logical STOP mode");
                opmode_last = 4;
                swmode_last = 0;
                web_server = "off";  // set flag to show minimal web content
            }
            // run limited opmode4 actions for STOP condition

            if (WiFiup == "no") {
                setupSoftAP();
            }

            server.handleClient();  // look for an HTTP request from a browser
        }

    }

    // ##############################################################
    // ##  opmode 4 active: run web server with the softAP option  ##
    // ##############################################################
    else if (opmode == 4 and swmode == 1) {
        if (mode_4 == 0) {
            stop();  // stop the motors just in case something has been left running
            if (opmode_last != 4 or swmode_last !=1) {
                Serial.println ("opmode 4 - (AD) not set up yet - CODE INACTIVE ");
                opmode_last = 4;
                swmode_last = 1;
            }
        } else {           
            if  (opmode_last != 4 or swmode_last !=1 ) {           
                Serial.println ("opmode 4 - 3 switches: (AD) web server operation - in logical GO mode");
                stop();             // stop the motors just as a start condition
                web_server = "on";  // set flag to show full web content
                opmode_last = 4;
                swmode_last = 1;
            }

            // run opmode4 actions for GO condition
            if (WiFiup == "no") {
                setupSoftAP();
            }

            int obdist = getDistance();     // check distance for potential need for an autostop
            // keep checking the distance from the ultrasonic sensor but only when going forward
            if (robot_state == 1 and obdist < stop_distance and stopauto == "yes") 
            {
               handle_diststop();      // stops robot 'outside' of the web interface
               Serial.print ("emergency stop distance: ");
               Serial.println (obdist);
            }

            server.handleClient();  // look for an HTTP request from a browser
        }

    }

    // #########################################################################
    // ## opmode 5 inactive: run web server with local WiFi connection option ##
    // #########################################################################
    else if (opmode == 5 and swmode == 0) {
        stop();  // stop the motors just in case something has been left running
        if (mode_5 == 0) {
            if (opmode_last != 5 or swmode_last !=0) {
                Serial.println ("opmode 5 - (AD) not set up yet - CODE INACTIVE ");
                opmode_last = 5;
                swmode_last = 0;
            }
        } else {           
            if  (opmode_last != 5 or swmode_last !=0 ) {           
                Serial.println ("opmode 4 - 3 switches: (AD) web server operation - in logical STOP mode");
                opmode_last = 5;
                swmode_last = 0;
                web_server = "off";  // set flag to show minimal web content
            }
            // run limited opmode5 actions for STOP condition

            if (WiFiup == "no") {
                setuplocalWiFi();
            }

            server.handleClient();  // look for an HTTP request from a browser to provide limited content
        }
    }

    // ########################################################################
    // ##  opmode 5 active: run web server with local WiFi connection option ##
    // ########################################################################
    else if (opmode == 5 and swmode == 1) {
        if (mode_5 == 0) {
            if (opmode_last != 5 or swmode_last !=1) {
                Serial.println ("opmode 5 - 3 switches: (AC) not set up yet - CODE INACTIVE ");
                opmode_last = 5;
                swmode_last = 1;
            }
        } else {           
            if  (opmode_last != 5 or swmode_last !=1 ) {           
                Serial.println ("opmode 5 - 3 switches: (AC) in logical GO mode");
                opmode_last = 5;
                swmode_last = 1;
                web_server = "on";  // set flag to show full web content
            }
            // run opmode5 actions for GO condition

            if (WiFiup == "no") {
                setuplocalWiFi();
            }

            int obdist = getDistance();     // check distance for potential need for an autostop
            // keep checking the distance from the ultrasonic sensor but only when going forward
            if (robot_state == 1 and obdist < stop_distance and stopauto == "yes")
            {
               handle_diststop();    // stops robot 'outside' of the web interface
               Serial.print ("emergency stop distance: ");
               Serial.println (obdist);
            }

            server.handleClient();  // look for an HTTP request from a browser to provide limited content
        }

    }

    // #####################################################
    // ## opmode 6 inactive: autonomous run around mode   ##
    // #####################################################
    else if (opmode == 6 and swmode == 0) {
	      stop();  // stop the motors just in case something has been left running
        // if here then opmode 6 selected but it is inactive                                       
        if (opmode_last != 6 or swmode_last !=0) { 
            Serial.println ("opmode 6 - 3 switches: (BC) autonomous run around mode - in logical STOP mode");
            opmode_last = 6;
            swmode_last = 0;
        }
    }

    // ####################################################
    // ##  opmode 6 active: autonomous run around mode   ##
    // ####################################################
    else if (opmode == 6 and swmode == 1) {

        if (mode_6 == 0) {
            if (opmode_last != 6 or swmode_last !=1) {
                Serial.println ("opmode 6 - 3 switches: (BC) autonomous run around mode - CODE INACTIVE ");
                opmode_last = 6;
                swmode_last = 1;
            }
        } else {           
            if  (opmode_last != 6 or swmode_last !=1 ) {           
                Serial.println ("opmode 6 - 3 switches: (BC) autonomous run around mode - in logical GO mode");
                opmode_last = 6;
                swmode_last = 1;
            }
            // run opmode6 actions

            forwards(80);
            delay(100);       // keep going forward and check the distance ahead every 100 ms
            cm = getDistance();

            if (cm < avoid_distance) {  // if an object is less than the avoid distance then 'count' the # of times this has happened
                avoid_count = avoid_count + 1;
                Serial.print(" distance: ");
                Serial.println(cm);
                Serial.print(" avoid count: ");
                Serial.println(avoid_count);
            }
  
            if(avoid_count >= avoid_limit) {  // if avoid count limit is reached then take avoidance action
                avoid_count = 0;  // reset the count limit first
                stop();
                auto_avoid_static();
            }

        }

    }

    // ################################################################
    // ## opmode 8 inactive: demo mode using the demo_controls file  ##
    // ################################################################
    else if (opmode == 8 and swmode == 0) {
        stop();  // stop the motors just in case something has been left running
        // if here then opmode 8 selected but it is inactive                                       
        if (opmode_last != 8 or swmode_last !=0) { 
            Serial.println ("opmode 8 - 3 switches: (BD) simple demo mode - in logical STOP mode");
            opmode_last = 8;
            swmode_last = 0;
        }
    }

    // ################################################################
    // ##  opmode 8 active: demo mode using the demo_controls file   ##
    // ################################################################
    else if (opmode == 8 and swmode == 1) {
        if (mode_8 == 0) {
            if (opmode_last != 8 or swmode_last !=1) {
                Serial.println ("opmode 8 - 3 switches: (BD) simple demo mode - CODE INACTIVE ");
                opmode_last = 8;
                swmode_last = 1;
            }
        } else {           
            if  (opmode_last != 8 or swmode_last !=1 ) {           
                Serial.println ("opmode 8 - 3 switches: (BD) simple demo mode - in logical GO mode");
                opmode_last = 8;
                swmode_last = 1;
            }
            // run opmode8 actions

            for (int i=0; i<=num_demo-1; i=i+2){
                //action = demo_controls[i];
                //act_int = demo_controls[i+1].toInt();
                // decode the action and execute it
                Serial.print ("demo action:");
                //Serial.print (action);
                Serial.print (demo_controls[i]);
                Serial.print ("-");
                Serial.print (demo_controls[i].length());
                Serial.print ("-");
                //Serial.println (act_int);
                Serial.println (demo_controls[i+1].toInt());
                demo_act(demo_controls[i], demo_controls[i+1].toInt());

            }


        }
    }

}

//
// *****************************************************************************************
// simple routine to just check the ON/OFF slide switch state and return the swmode variable
// type int as it returns the swmode as a simple integer
// debug set to 1 gives additional outputs - but the routine is otherwise 'silent'
// **************************************************************************************
//
int check_onoff(int swmode_last, int s_debug, int onoff)
{
    // check for just the ON/OFF slide switch setting
    // swmode_last is a simple passed parameter of the current/last swmode setting
    // swmode = 0 means 'OFF' and swmode = 1 means 'ON'

  	int swmode=10;  // initially set to a non-normal value

    int state_onoff = digitalRead(onoff);
    // if the on/off switch is low (=0), it's OFF
    if (state_onoff == 0)
    {
        if (swmode_last!=0 && s_debug==1)
        {
            Serial.println("** The on/off switch is now OFF \n");
            Serial.println("** slide switches set to IDLE mode \n");

        }
		    swmode=0;    //  switch mode is OFF
    }
	
    else if (state_onoff == 1)
    {
        if (swmode_last!=1 && s_debug==1)
        {
            Serial.println("** The on/off switch is now ON \n");
            Serial.println("** slide switches set to an active mode for whatever opmode is set \n");

        }
		    swmode=1;    //  switch mode is ON
    }

    return swmode; 

}

//
// *****************************************************************************************
// motor control functions using PWM speed control
// battery power provides about 8V but a motor will generally not run with less than 4V 
// so only accept values >50% (512)
// and take the 'percent' % input value passed as a parameter as the 512 - 1023 range
// **************************************************************************************
//

void forwards(int percent){
    // percent is a passed % value used to calculate a pwr value from 512 - 1023
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    Serial.print("left motor power (512-1023): ");
    Serial.println(pwrL);    
    Serial.print("right motor power (512-1023): ");
    Serial.println(pwrR); 
    analogWrite(5, pwrL);  // motor A (left)  PWM speed adjusted for L/R balance
    analogWrite(4, pwrR);  // motor B (right) PWM speed adjusted for L/R balance
    digitalWrite(0, LOW);  // motor A forward
    digitalWrite(2, LOW);  // motor B forward
    }

void leftforwards(int percent){
    // percent is a passed % value used to calculate a pwr value from 512 - 1023
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    // adjust left power
    int pwrL = pwr*motorL;
    Serial.print("left motor power (512-1023): ");
    Serial.println(pwrL);    
    analogWrite(5, pwrL);  // motor A (left)  PWM speed adjusted for L/R balance
    digitalWrite(0, LOW);  // motor A forward
    }

void rightforwards(int percent){
    // percent is a passed % value used to calculate a pwr value from 512 - 1023
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    // adjust right power
    int pwrR = pwr*motorR;
    Serial.print("right motor power (512-1023): ");
    Serial.println(pwrR); 
    analogWrite(4, pwrR);  // motor B (right) PWM speed adjusted for L/R balance
    digitalWrite(2, LOW);  // motor B forward
    }

void stop(){
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
    digitalWrite(0, LOW);
    digitalWrite(2, LOW);
    }

void backwards(int percent){
    // percent is a passed % value used to calculate a pwr value from 512 - 1023
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    Serial.print("left motor power (512-1023): ");
    Serial.println(pwrL);    
    Serial.print("right motor power (512-1023): ");
    Serial.println(pwrR);
    analogWrite(5, pwrL); // motor A (left) PWM speed adjusted for L/R balance
    analogWrite(4, pwrR); // motor B (right) PWM speed adjusted for L/R balance
    digitalWrite(0, HIGH); // motor A backward
    digitalWrite(2, HIGH); // motor B backward
    }

void leftbackwards(int percent){
    // percent is a passed % value used to calculate a pwr value from 512 - 1023
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    int pwrL = pwr*motorL;
    Serial.print("left motor power (512-1023): ");
    Serial.println(pwrL);    
    analogWrite(5, pwrL);  // motor A (left) PWM speed adjusted for L/R balance
    digitalWrite(0, HIGH); // motor A backward
    }

void rightbackwards(int percent){
    // percent is a passed % value used to calculate a pwr value from 512 - 1023
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    int pwrR = pwr*motorR;
    Serial.print("right motor power (512-1023): ");
    Serial.println(pwrR);
    analogWrite(4, pwrR);  // motor B (right) PWM speed adjusted for L/R balance
    digitalWrite(2, HIGH); // motor B backward
    }


void turn_left(int t_time, int percent){
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    // adjust left/right power
    int pwrR = pwr*motorR;
    digitalWrite(5, LOW);  // turn motor A (left) off
    analogWrite(4, pwrR);  // set motor B (right) to a speed that might need to be fine tuned
    digitalWrite(0, LOW);  // set motor A to the zero speed set above
    digitalWrite(2, LOW);  // run motor B (right) forwards
    delay(t_time);         // amount of time (ms) the motor is run to turn 90-deg - that might need to be fine tuned
    stop();
    }

void turn_right(int t_time, int percent){
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    analogWrite(5, pwrL);  // set motor A (left) to a speed that might need to be fine tuned
    digitalWrite(4, LOW);  // turn motor B (right) off
    digitalWrite(0, LOW);  // run motor A (left) forwards
    digitalWrite(2, LOW);  // set motor B to the zero speed set above
    delay(t_time);
    stop();
    }
  
void spin_left(int s_time, int percent){
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    analogWrite(5, pwrL);   // set motor A (left) to a fixed speed that might need to be fine tuned
    analogWrite(4, pwrR);   // set motor B (right) to a fixed speed that might need to be fine tuned
    digitalWrite(0, HIGH);  // run motor A (left) backwards
    digitalWrite(2, LOW);   // run motor B (right) forwards
  	delay(s_time);          // amount of time (ms) the motor is run to turn 90-deg - that might need to be fine tuned
    stop();
    }

void spin_right(int s_time, int percent){
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = 512 + percent*(1023-512)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    analogWrite(5, pwrL);   // set motor A (left) to a fixed speed that might need to be fine tuned
    analogWrite(4, pwrR);   // set motor B (right) to a fixed speed that might need to be fine tuned
    digitalWrite(0, LOW);   // run motor A (left) forwards
    digitalWrite(2, HIGH);  // run motor B (right) backwards
    delay(s_time);
    stop();
    }


//
// **************************************************************************************
// routine to check slide switch state and change the opmode setting accordingly
// type int as it returns the opmode as a simple integer where the options are set out below
//  n_switch: provides the number of switches in use (usually just 3 for early NodeMCU builds
//  opmode_last: provides the last value of opmode for various logic checks
//  s_debug: if set to 1 gives additional outputs - but the routine is otherwise 'silent'
//  s_AB, s_CD and s_EF: provide the GPIO# for the switches, 2 or 3 of which set activity options
//
// switch state calculation:  state_AB   state_CD   state_EF are either 0 or 1
// 
// **************************************************************************************
//
int check_slideswitch(int n_switch, int opmode_last, int s_debug, int s_AB, int s_CD, int s_EF)
{
    // check for each slide switch setting
    // opmode_last is a simple passed parameter of the current/last opmode setting
    // opmode = 0 means 'idle' with the ON/OFF switch in the OFF position - but not checked here
    // ** opmodes for 4 switches 1 of which is the GO/STOP **
    // opmode = 1 means ACE
    // opmode = 2 means BCE
    // opmode = 3 means ADE
    // opmode = 4 means ADF
    // opmode = 5 means ACF
    // opmode = 6 means BCF
    // opmode = 7 means BDE
    // opmode = 8 means BDF
    // opmode = 9 means an undefined operational mode

    // ** opmodes for 3 switches 1 of which one is the GO/STOP **
    // opmode = 4 means AD  softAP web server mode
    // opmode = 5 means AC  local WiFi web server mode
    // opmode = 6 means BC  autonomous run around mode
    // opmode = 8 means BD  demo mode
    // opmode = 9 means an undefined operational mode

    opmode=10;  // initially set to a non-normal value

    // read the current switch states
    state_AB = digitalRead(s_AB);
    state_CD = digitalRead(s_CD);
    if (n_switch == 4) {
        state_EF = digitalRead(s_EF);
    } else {
        state_EF = 0;	
    }

    // Now check all the various switch state permutations
	
    // *** ON/OFF slide switch *** not checked here - now in a separate routine (see above)
    //     -------------------
       

    // *** ACE combination - opmode 1 ***
    //     --------------------------
    if (state_AB == 1 && state_CD == 1 && state_EF == 1)
  	{
        if (s_debug==1 && opmode_last != 1)
        {
            Serial.println("** 4 slide switches set to 'ACE'- opmode 1");
        }
        opmode=1;
  	}

    // *** BCE combination - opmode 2 ***
    //     --------------------------
    else if (state_AB == 0 && state_CD == 1 && state_EF == 1)
    {
        if (s_debug==1 && opmode_last != 2)
        {
            Serial.println("** 4 slide switches set to 'BCE'- opmode 2");
        }
        opmode=2;
  	}

    // *** ADE combination - opmode 3 ***
    //     --------------------------
    else if (state_AB == 1 && state_CD == 0 && state_EF == 1)
  	{
        if (s_debug==1 && opmode_last != 3)
        {
            Serial.println("** 4 slide switches set to 'ADE'- opmode 3");
        }
        opmode=3;
  	}

    // *** ADF combination - opmode 4 ***
    //     --------------------------
  	else if (state_AB == 1 && state_CD == 0 && state_EF == 0)
  	{
        if (s_debug==1 && opmode_last != 4 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'ADF'- opmode 4");
        } 
        else if (s_debug==1 && opmode_last != 4 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'AD'- opmode 4");
        }

        opmode=4;
	  }

    // *** ACF combination - opmode 5 ***
    //     --------------------------
  	else if (state_AB == 1 && state_CD == 1 && state_EF == 0)
    {
        if (s_debug==1 && opmode_last != 5 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'ACF'- opmode 5");
        } 
        else if (s_debug==1 && opmode_last != 5 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'AC'- opmode 5");
        }
        opmode=5;
  	}

    // *** BCF combination - opmode 6 ***
    //     --------------------------
	  else if (state_AB == 0 && state_CD == 1 && state_EF == 0)
  	{
        if (s_debug==1 && opmode_last != 6 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'BCF'- opmode 6");
        } 
        else if (s_debug==1 && opmode_last != 6 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'BC'- opmode 6");
        }
        opmode=6;
	  }

    // *** BDE combination - opmode 7 ***
    //     --------------------------
  	else if (state_AB == 0 && state_CD == 0 && state_EF == 1)
    {
        if (s_debug==1 && opmode_last != 7)
        {
            Serial.println("** 4slide switches set to 'BDE'- opmode 7");
        }
        opmode=7;
  	}

    // *** BDF combination - opmode 8 ***
    //     --------------------------
  	else if (state_AB == 0 && state_CD == 0 && state_EF == 0)
  	{
        if (s_debug==1 && opmode_last != 8 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'BDF'- opmode 8");
        } 
        else if (s_debug==1 && opmode_last != 8 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'BD'- opmode 8");
        }
        opmode=8;
	  }

    // *** opmode 99 ***
    //     --------------------------
    else  // should never be here!!
  	{
        if (s_debug==1 && opmode_last != 99 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to an impossible combination! ");
        } 
        else if (s_debug==1 && opmode_last != 99 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to an impossible combination! ");
        }
        opmode=99;
  	}

    return opmode;

}

// *******************************************************
// function to sense the distance to an obstacle
// uses the time taken for pulse to be sensed coming back
// and the speed of sound in air as 34326cm/s
// returns a value in cm
// *******************************************************
int getDistance()
{
  int rval;
  // send trigger pulse and time how long it takes to 'hear' it come back
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);   // set the lLOW for a 'stabilising' period so that the HIGH signal is 'clean'
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // if pulseIn times out it probably means the sensor is too close
  //  for the electronics to 'catch' the return signal in time or 
  //  maybe the outbound pulse just got lost i.e. an object is too 
  //  far away so reset the U/S with a LOW and return  a silly distance
  //  that could if necessary be 'checked for'
  duration = pulseIn(echoPin, HIGH, 38000L);  // Set timeout to 38mS
                                              // which sets duration to zero
  if (duration == 0)
  {
    return 999; 
  }
  rval = microsecondsToCentimeters(duration);
//  Serial.println(rval);
  return rval;
}

// **************************************
// ultrasonic sensor conversion function
// **************************************
long microsecondsToCentimeters(long microseconds)
{
  // total distance there and back is usecs*34326/1000000 
  // so half distance is usecs*34326/(1000000*2) = usecs/58.26
  return microseconds / cmconv;
}


// ****************************************
// autonomous obstacle avoidance function
//  for a static mounted ultrasonic sensor
// ****************************************
void auto_avoid_static()
{
    // Move back a little, then turn right or left on alternate tries

    // Back off a little 
    Serial.println(" moving backwards") ;
    backwards(70); 
    delay(ReverseTime);
    stop();

   // Turn right or left
    if (leftright == "right") {
        Serial.println("** spin Right to avoid obstacle **");
        spin_right(avoidspintime, spinspeed);
        leftright = "left";
    } else {
        Serial.println("** spin Left to avoid obstacle **");
        spin_left(avoidspintime, spinspeed);
        leftright = "right";
    }

}
  

// ******************************************************
// ****          set up 'soft AP' WiFi               ****
// ******************************************************
void setupSoftAP()
{
    Serial.print("Setting soft-AP IP configuration ... ");
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

    Serial.print("Setting soft-AP ssid and password ... ");
    Serial.println(WiFi.softAP("ESPsoftAP", "pswd12345") ? "Ready" : "Failed!");

    Serial.print("Soft-AP IP address = ");
    Serial.println(WiFi.softAPIP());     // IP address shouldn't change so could be used in a browser

    Serial.printf("Soft-AP MAC address = %s\n", WiFi.softAPmacAddress().c_str());

    if (!MDNS.begin(namehost)) {        // Start the mDNS responder for namehost ....local
        Serial.println("Error setting up MDNS responder!");
    }
    Serial.print("mDNS responder started for domain: ");
    Serial.print(namehost);
    Serial.println(".local");    
    delay(2000);
    WiFiup = "yes";
}


// ********************************************
// **** set up WiFi with host name as well ****
// ********************************************
void setuplocalWiFi()
{
    Serial.print("trying to connect to WiFi with hostname: ");
    Serial.println(namehost);
    WiFi.hostname(namehost);

    // scan to find all the 'live' broadcast SSID's ....
    int n = WiFi.scanNetworks();
    Serial.print("Number of WiFi networks found: ");
    Serial.println(n);
    ssid_selected ="";

    Serial.print("trying to connect to: ");
    Serial.println(ssid1);
    // try to use ssid1 first
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid1 ) {
            ssid_selected = ssid1;
            break;
        }
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid2 if ssid1 not already selected
    Serial.print("trying to connect to: ");
    Serial.println(ssid2);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid2 ) {
            ssid_selected = ssid2;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid3 
    Serial.print("trying to connect to: ");
    Serial.println(ssid3);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid3 ) {
            ssid_selected = ssid3;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid4
    Serial.print("trying to connect to: ");
    Serial.println(ssid4);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid4 ) {
            ssid_selected = ssid4;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid5
    Serial.print("trying to connect to: ");
    Serial.println(ssid5);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid5 ) {
            ssid_selected = ssid5;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // if here then no allowed local WiFi found 
    Serial.println(" No allowed WiFi found");


}

// ******************************
// **** make WiFi connection ****
// ******************************
void connectWiFi()
{

    Serial.print(ssid_selected);
    Serial.println(" selected - now trying to connect");
	
	  if (ssid_selected == ssid1 ) {
            WiFi.begin(ssid1,password1);    //initiate connection to ssid1
            Serial.print("SSID: ");
            Serial.println(ssid1);

    } else if (ssid_selected == ssid2 ) {
            WiFi.begin(ssid2,password2);    //initiate connection to ssid2
            Serial.print("SSID: ");
            Serial.println(ssid2);

    } else if (ssid_selected == ssid3 ) {
            WiFi.begin(ssid3,password3);    //initiate connection to ssid3
            Serial.print("SSID: ");
            Serial.println(ssid3);

    } else if (ssid_selected == ssid4 ) {
            WiFi.begin(ssid4,password4);    //initiate connection to ssid4
            Serial.print("SSID: ");
            Serial.println(ssid4);

    } else if (ssid_selected == ssid5 ) {
            WiFi.begin(ssid5,password5);    //initiate connection to ssid5
            Serial.print("SSID: ");
            Serial.println(ssid5);

    }
    Serial.println("");
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // Connected to the first available/defined WiFi Access Point
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid_selected);
    Serial.print("IP address: ");
    delay(500);
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    delay(500);
    Serial.println(WiFi.macAddress());
    Serial.print("hostname: ");
    delay(500);
    Serial.println(WiFi.hostname());
    delay(2000);
    WiFiup = "yes";

}


// **********************************************************************
// Function to create/open an existing file and write a single string to 
//  it - the file name and text are passed strings to the function 
// **********************************************************************
void write_text(String wfile, String wtext) {
    //w=Write Open file for writing
    File SPIfile = SPIFFS.open(wfile, "w");
  
    if (!SPIfile) {
        Serial.println("file open failed");
    }
    else
    {
        //Write data to file
        Serial.print("Writing Data to File: ");
        Serial.println(wtext);
        SPIfile.print(wtext);
        SPIfile.close();  //Close file

    }
}


// **********************************************************
// Function to read a single string from a written text file
// **********************************************************
String read_text(String rfile) {
    int i;
    String str_read;
    //open the file for reading
    File f = SPIFFS.open(rfile, "r");
  
    if (!f) {
        str_read = "file open failed";
        Serial.println(str_read);
    }
    else
    {
        Serial.print("Reading Text Data from File: ");
        //read string from file
        str_read = f.readStringUntil('\n');
        if (rfile.substring(1,5) == "pass") {
            Serial.println("password not shown");
        } else {
            Serial.println(str_read);
        }
        f.close();  //Close file
        Serial.println("File Closed");
    }
    return str_read;

}


// ************************************************************************************
// Function to create/open an existing multiple string file and write data to it
// the local variables: 'wfile' is passed the file name, 'strarray' the string array
//  to be written to the file and int num is the number of array elements to be written
// ************************************************************************************
void write_strings(String wfile, String strarray[], int num) {
    //w=Write: open file for writing from the beginning overwriting whatever is already there
    File SPIfileStr = SPIFFS.open(wfile, "w");
  
    if (!SPIfileStr) {
        Serial.println("file open failed");
    }
    else
    {
        //Write string array data to file
        Serial.println("  ");
        Serial.println("Writing String array data to the file");
        for (int i=0; i<=num-1; i++){
            SPIfileStr.println(strarray[i]);    // writes an individual string with a LF at the end
        }
        SPIfileStr.close();  //Close file
        Serial.println("String array writing complete ");
    }
}

// *****************************************************************************************
// Function to read an existing multiple string file and extract the strings from it
// the local variables: 'rfile' is passed the file name, 'strarray' the string array
//  'pointed' to by reference and int num is the number of string array elements to be read
// *****************************************************************************************
void read_strings(String rfile, String strarray[], int num) {
    String tempstr;
    //r=Read Open file for reading
    File SPIfileStr = SPIFFS.open(rfile, "r");
  
    if (!SPIfileStr) {
        Serial.println("multiple string file open failed");
        strarray[0] = "multiple string file open failed";

    }
    else
    {
        //Read string data from file looking for the LF's that separate each string
        Serial.println("  ");
        Serial.println("Function read_strings: reading 'num' strings from file");
        for (int i=0; i<=num-1; i++){                     // loop thru the 'num' strings in the file
            tempstr=SPIfileStr.readStringUntil('\n'); // [i] string read upto the \n terminator
                               //  the terminator is discarded from the stream but adds a space
            if (tempstr.length() > 2) {
                tempstr.remove(tempstr.length()-1,1);
            }
            strarray[i] = tempstr;
            Serial.print(i);
            Serial.print(": ");
            Serial.println(strarray[i]);
        }
        SPIfileStr.close();  //Close file
        Serial.println("Function read_strings: string reading complete ");

    }
}


// **********************************************************************
// ***  function to decode the demo_control 'pairs' and execute them  ***
// **********************************************************************
void demo_act(String act, int a_int) {
    Serial.print ("action requested:");
    Serial.print (act);
    Serial.print ("-");
    Serial.print (act.length());
    Serial.print ("-");
    Serial.println (a_int);
    
    if (act == demo_actions[0]) {
        forwards(a_int);
    } else if (act == demo_actions[1]) {
        backwards(a_int);
    } else if (act == demo_actions[2]) {
        delay(a_int);
    } else if (act == demo_actions[3]) {
        stop();
    } else if (act == demo_actions[4]) {
        spin_left(a_int, turnspeed);
    } else if (act == demo_actions[5]) {
        spin_right(a_int, turnspeed);
    } else if (act == demo_actions[6]) {
        turn_left(a_int, turnspeed);
    } else if (act == demo_actions[7]) {
        turn_right(a_int, turnspeed);
    } else {
        Serial.println("No recognised action found ");
    }
}


// *****************************************************************
// ***  this section is for all the browser response 'handlers'  ***
// *****************************************************************
void handle_root() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("web root - robot stopped");
    server.send(200, "text/html", HTMLmain()); 
}

// **********************************
// *** main selection 'handlers' ****
// **********************************
void handle_run_about() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting robot run-about control");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_auto_run() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting robot autonomous operation control");
    server.send(200, "text/html", HTMLrobot_auto_run()); 
}
void handle_demo_run() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting robot demo operation control");
    server.send(200, "text/html", HTMLrobot_demo_run()); 
}
void handle_run_tests() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting component testing");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_parameters() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting parameter update");
    server.send(200, "text/html", HTMLparameter_selection()); 
}
void handle_sysinfo() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting system information display");
    server.send(200, "text/html", HTMLsysinfo()); 
}


// ******************************************************
// *** sub selection  demo operation 'handlers' ***
// ******************************************************
void handle_run_demo() {
    stop();  // stop robot just for now
    robot_state = 1;
    Serial.println("putting robot into demo operation");

    while (robot_state == 1) {
        for (int i=0; i<=num_demo-1; i=i+2){
            //action = demo_controls[i];
            //act_int = demo_controls[i+1].toInt();
            // decode the action and execute it
            Serial.print ("demo action:");
            //Serial.print (action);
            Serial.print (demo_controls[i]);
            Serial.print ("-");
            Serial.print (demo_controls[i].length());
            Serial.print ("-");
            //Serial.println (act_int);
            Serial.println (demo_controls[i+1].toInt());
            demo_act(demo_controls[i], demo_controls[i+1].toInt());
            server.send(200, "text/html", HTMLrobot_demo_run()); 
            server.handleClient();  // look for an HTTP request from a browser within the 'while' 
            //  and the 'for' loops so that the demo mode can be interupted after each demo command
            //  e.g. clicking the STOP button stops the robot and sets robot_state to 0 to stop the 
            //  'while' loop - but the check below is additionally needed to immediately break the 'for' loop
            if (robot_state != 1) {
                break;
            }
        }
    }
}
void handle_stop_demo() {
    stop();  // stop robot 
    robot_state = 0;
    Serial.println("stopping robot's demo operation");
    server.send(200, "text/html", HTMLrobot_demo_run()); 
}


// ******************************************************
// *** sub selection  autonomous operation 'handlers' ***
// ******************************************************
void handle_run_auto() {
    stop();  // stop robot just for now
    robot_state = 1;
    Serial.println("putting robot into autonomous operation");

    while (robot_state == 1) {
        forwards(80);
        delay(100);       // keep going forward and check the distance ahead every 100 ms
        cm = getDistance();

        if (cm < avoid_distance) {  // if an object is less than the avoid distance then 'count' the # of times this has happened
            avoid_count = avoid_count + 1;
            Serial.print(" distance: ");
            Serial.println(cm);
            Serial.print(" avoid count: ");
            Serial.println(avoid_count);
        }
  
        if(avoid_count >= avoid_limit) {  // if avoid count limit is reached then take avoidance action
            avoid_count = 0;  // reset the count limit first
            stop();
            auto_avoid_static();
        }
        server.send(200, "text/html", HTMLrobot_auto_run()); 
        server.handleClient();  // look for an HTTP request from a browser within the while loop
                                //  so that the auto-run mode can be interupted e.g. clicking the
                                //  web STOP button causes the robot to stop and robot_state set 
                                //  to 0 so the while loop stops as well
    }
}
void handle_run_stop() {
    stop();  // stop robot 
    robot_state = 0;
    Serial.println("stopping robot's autonomous operation");
    server.send(200, "text/html", HTMLrobot_auto_run()); 
}
// *** web browser robot auto-run input 'handlers' ***
void handle_autospeed() {
    str_r_speed = server.arg("r_speed_str");  // get string from browser response
    r_speed = str_r_speed.toInt();            // convert string to the integer version
    Serial.print("robot speed input integer: ");
    Serial.println(r_speed);
    robot_state = 0;
    server.send(200, "text/html", HTMLrobot_auto_run());
}


// **************************************************
// *** sub selection  parameter update 'handlers' ***
// **************************************************
void handle_WiFi_params() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting WiFi updates");
    server.send(200, "text/html", HTMLWiFi_params()); 
}
void handle_robot_updates() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting robot parameter updates");
    server.send(200, "text/html", HTMLrobot_updates()); 
}
void handle_GPIO_updates() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting GPIO pin updates");
    server.send(200, "text/html", HTMLGPIO_updates()); 
}
void handle_demo_updates() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("selecting demo file updates");
    server.send(200, "text/html", HTMLdemo_updates()); 
}


// *******************************************************
// *** parameter update submission detailed 'handlers' ***
// *******************************************************
void handle_demo_file() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("demo file update");

    // get updated demo action details from browser response 
    int numdemo;
    numdemo = server.arg("demo_num_str").toInt(); 
    // check demo_act_str is an allowable command
    Serial.println("checking demo_controls");
    for (int j=0; j<=7; j=j+1) {
        if (server.arg("demo_act_str") == demo_actions[j]) {
            Serial.print(demo_actions[j]); 
            Serial.println(" action found");
            demo_controls[numdemo*2] = server.arg("demo_act_str");
            break;
        } else {
            Serial.println(" no allowable action found");
            demo_controls[numdemo*2] = "ERROR";
        }
    }
    // to be added later: a value check on 'demo_val_str' as well    
    demo_controls[numdemo*2+1] = server.arg("demo_val_str").toInt();

    // write the updated 'demo_controls' back out to the SPIFFS file
    write_strings("/demo_controls.txt", demo_controls, num_demo);

    server.send(200, "text/html", HTMLdemo_updates());
}



void handle_robot_params() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("robot parameter update");

    // get robot parameter strings from the browser response
    str_avoid_distance = server.arg("avoid_distance_str"); 
    str_stop_distance = server.arg("stop_distance_str"); 
    str_avoid_limit = server.arg("avoid_limit_str");  
    str_avoidspintime = server.arg("avoidspintime_str"); 
    str_turntime = server.arg("turntime_str"); 
    str_spintime = server.arg("spintime_str");
    str_turnspeed = server.arg("turnspeed_str"); 
    str_spinspeed = server.arg("spinspeed_str");
    str_r_speed = server.arg("r_speed_str"); 
    str_ReverseTime = server.arg("ReverseTime_str"); 
    stopauto = server.arg("stopauto_str");
    str_motorL = server.arg("motorL_str"); 
    str_motorR = server.arg("motorR_str"); 

    // convert strings back to integers
    avoid_distance = str_avoid_distance.toInt();
    stop_distance = str_stop_distance.toInt();
    avoid_limit = str_avoid_limit.toInt();
    avoidspintime = str_avoidspintime.toInt();
    turntime = str_turntime.toInt();
    spintime = str_spintime.toInt();
    turnspeed = str_turnspeed.toInt();
    spinspeed = str_spinspeed.toInt();
    r_speed = str_r_speed.toInt();
    motorL = str_motorL.toFloat();
    motorR = str_motorR.toFloat();
    ReverseTime = str_ReverseTime.toInt();

    // resave the robot parameter strings
    Serial.println("-------------------------------");
    write_text("/str_avoid_distance.txt", str_avoid_distance);
    write_text("/str_stop_distance.txt", str_stop_distance);
    write_text("/str_avoid_limit.txt", str_avoid_limit);
    write_text("/str_avoidspintime.txt", str_avoidspintime);
    write_text("/str_turntime.txt", str_turntime);
    write_text("/str_spintime.txt", str_spintime);
    write_text("/str_turnspeed.txt", str_turnspeed);
    write_text("/str_spinspeed.txt", str_spinspeed);
    write_text("/str_r_speed.txt", str_r_speed);
    write_text("/str_ReverseTime.txt", str_ReverseTime);
    write_text("/str_stopauto.txt", stopauto);
    Serial.println("-------------------------------");
    write_text("/str_motorL.txt", str_motorL);
    write_text("/str_motorR.txt", str_motorR);
    Serial.println("-------------------------------");
    Serial.println("robot parameter data resaved ");

    server.send(200, "text/html", HTMLrobot_updates()); 
}

// *** WiFi update input 'handlers' ***
// -----------------------------------
void handle_WiFi_updates1() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("WiFi SSID1 settings update");
    ssid1 = server.arg("ssid_1");          // get string from browser response
    password1 = server.arg("password_1");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid1.txt", ssid1);
    write_text("/password1.txt", password1);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates2() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("WiFi SSID2 settings update");
    ssid2 = server.arg("ssid_2");          // get string from browser response
    password2 = server.arg("password_2");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid2.txt", ssid2);
    write_text("/password2.txt", password2);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates3() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("WiFi SSID3 settings update");
    ssid3 = server.arg("ssid_3");          // get string from browser response
    password3 = server.arg("password_3");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid3.txt", ssid3);
    write_text("/password3.txt", password3);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates4() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("WiFi SSID4 settings update");
    ssid4 = server.arg("ssid_4");          // get string from browser response
    password4 = server.arg("password_4");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid4.txt", ssid4);
    write_text("/password4.txt", password4);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates5() {
    stop();  // stop robot just in case
    robot_state = 0;
    Serial.println("WiFi SSID5 settings update");
    ssid5 = server.arg("ssid_5");          // get string from browser response
    password5 = server.arg("password_5");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid5.txt", ssid5);
    write_text("/password5.txt", password5);
    server.send(200, "text/html", HTMLWiFi_params());
}

// *** GPIO pin# update input 'handler' ***
// ----------------------------------------
void handle_GPIO_pins() {
    str_s_AB = server.arg("s_AB_str");        // get string from browser response
    str_s_CD = server.arg("s_CD_str");        // get string from browser response
    str_onoff = server.arg("onoff_str");      // get string from browser response
    str_trigPin = server.arg("trigPin_str");  // get string from browser response
    str_echoPin = server.arg("echoPin_str");  // get string from browser response

    // convert strings back to integers
    s_AB = str_s_AB.toInt();
    s_CD = str_s_CD.toInt();
    s_EF = str_s_EF.toInt();
    onoff = str_onoff.toInt();
    trigPin = str_trigPin.toInt();
    echoPin = str_echoPin.toInt();

    // resave the GPIO pin# settings as strings
    Serial.println("-------------------------------");
    write_text("/str_s_AB.txt", str_s_AB);
    write_text("/str_s_CD.txt", str_s_CD);
    write_text("/str_onoff.txt", str_onoff);
    write_text("/str_trigPin.txt", str_trigPin);
    write_text("/str_echoPin.txt", str_echoPin);;
    Serial.println("-------------------------------");
    Serial.println("GPIO pin# data resaved ");

    server.send(200, "text/html", HTMLGPIO_updates());
}


// ************************************
// *** component testing 'handlers' ***
// ************************************
void handle_testspeed() {
    str_r_speed = server.arg("r_speed_str");  // get string from browser response
    r_speed = str_r_speed.toInt();           // convert string to the integer version
    Serial.print("component testing speed input integer: ");
    Serial.println(r_speed);
    robot_state = 0;
    server.send(200, "text/html", HTMLrun_tests());
}
void handle_USread() {
    stop();  // stop both motors just in case
    robot_state = 0;
    Serial.println("reading U/S sensor");
    cm = getDistance();
    cm_str = String(cm);
    server.send(200, "text/html", HTMLrun_tests()); 
}

void handle_teststop() {
    stop();  // stop both motors
    robot_state = 0;
    Serial.println("motors stopped");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_leftfwd() {
    leftforwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 4;
    Serial.println("left motor forwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_rightfwd() {
    rightforwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 4;
    Serial.println("right motor forwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_leftback() {
    leftbackwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 4;
    Serial.println("left motor backwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_rightback() {
    rightbackwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 4;
    Serial.println("right motor bcakwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}


// ********************************
// *** robot control 'handlers' ***
// ********************************
void handle_stop() {
    stop();  // stop robot
    robot_state = 0;
    Serial.println("robot stopped");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_diststop() {  // special version used for auto stop with object detection
    stop();  // stop robot
    robot_state = 3;
    Serial.println("robot emergency stopped");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_forward() {
    forwards(r_speed);   // run robot forwards at r_speed% speed
    robot_state = 1;
    Serial.println("robot going forwards");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_backward() {
    backwards(r_speed);   // run robot backwards at r_speed% speed
    robot_state = 2;
    Serial.println("robot going backwards");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_turnleft() {
    turn_left(turntime, turnspeed);  // turn the robot left 90-deg
    robot_state = 0;
    Serial.println("robot turned left");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_turnright() {
    turn_right(turntime, turnspeed);  // turn the robot right 90-deg
    robot_state = 0;
    Serial.println("robot turned right");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_spinleft() {
    spin_left(spintime, turnspeed);  // spin the robot left 90-deg
    robot_state = 0;
    Serial.println("robot spun left");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_spinright() {
    spin_right(spintime, turnspeed);  // spin the robot left 90-deg
    robot_state = 0;
    Serial.println("robot spun right");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
// *** web browser robot control input 'handlers' ***
void handle_postspeed() {
    str_r_speed = server.arg("r_speed_str");  // get string from browser response
    r_speed = str_r_speed.toInt();           // convert string to the integer version
    Serial.print("robot speed input integer: ");
    Serial.println(r_speed);
    robot_state = 0;
    server.send(200, "text/html", HTMLrobot_run_about());
}



// ****************************************************************
// ******  create the various web pages that are being used  ******
// ******  this is done as code rather than SPIFFS HTML files *****
// ******  so that logic can be easily built-in to each page  *****
// ****************************************************************

// --------------------------------------------------------------------------------
// create the header area used in all the web pages - done once in the setup stage
// --------------------------------------------------------------------------------
String HTMLheader() {
    String h_content = "<!DOCTYPE html> <html>\n";
    h_content +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">\n";
    h_content +="<title>Robot Control</title>\n";
	
    // style content in the page header ( to be replaced by a css file eventually )
    h_content +="<style>html { font-family: Verdana; display: inline-block; margin: 0px auto; text-align: center; font-size: 15px;}\n";
    h_content +="body{margin-top: 50px;} h1 {color: #444444; margin: 10px auto 10px; font-size: 32px;} h3 {color: #444444; margin: 10px auto 10px; font-size: 24px;}\n";
    h_content +=".button {display: block; width: 90px; background-color: #1abc9c;border: none;color: white; padding: 13px 30px; text-decoration: none; font-size: 32px; margin: 5px auto 5px; cursor: pointer; border-radius: 4px;}\n";
    h_content +=".btninline {display: inline-block; }\n";
    h_content +=".button-on {background-color: #1abc9c;}\n";
    h_content +=".button-on:active {background-color: #16a085;}\n";
    h_content +=".button-off {background-color: #34495e;}\n";
    h_content +=".button-off:active {background-color: #2c3e50;}\n";
    h_content +=".button-red {background-color: #f51031;}\n";
    h_content +=".button-red:active {background-color: #d20e2a;}\n";
    h_content +="p {font-size: 18px;color: #888; margin: 5px;}\n";
    h_content +="</style>\n";
    h_content +="</head>\n";
    return h_content;
}

// -----------------------------------------
// create the main selection web page
// -----------------------------------------
String HTMLmain(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>main selections</h1>\n";
    page_content +="<p><a class=\"button btninline button-off\" href=\"run_about\"><button>Run-about controls</button></a>&nbsp; &nbsp; &nbsp;";
    page_content +="<a class=\"button btninline button-off\" href=\"auto_run\"><button>Auto-run controls</button></a></p>\n";
    page_content +="<p><a class=\"button btninline button-off\" href=\"demo_run\"><button>Demo-run controls</button></a>&nbsp; &nbsp; &nbsp;";
    page_content +="<a class=\"button btninline button-off\" href=\"run_tests\"><button>Component testing</button></a></p>\n";
    page_content +="<p><a class=\"button btninline button-off\" href=\"parameters\"><button>Update parameters</button></a>&nbsp; &nbsp; &nbsp;";
    page_content +="<a class=\"button btninline button-off\" href=\"sysinfo\"><button>System information</button></a></p>\n";

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the system information display web page
// ---------------------------------------------
String HTMLsysinfo(){
    String page_content = header_content;
    FSInfo fs_info;
    SPIFFS.info(fs_info);

    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>System Information</h3>\n";

    // **** networking information ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px;\"><b>Networking:</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";
    page_content +="<tr><td style=\"width: 225px; \">connected to WiFi SSID:</td><td>" ;
    if (opmode == 5) {
        page_content +=ssid_selected;
    } else {
        page_content +="ESPsoftAP";
    }
    page_content +="</td></tr>\n";
    page_content +="<tr><td>host name:</td><td>" ;
    if (opmode == 5) {
        page_content +=WiFi.hostname();
    } else {
        page_content +="esp8266";
    }
    page_content +="</td></tr>\n";
    page_content +="<tr><td>assigned IP address:</td><td>" ;
    if (opmode == 5) {
        page_content +=WiFi.localIP().toString();
    } else {
        page_content +=WiFi.softAPIP().toString();
    }
    page_content +="</td></tr>\n";
    page_content +="<tr><td>WiFi MAC address:</td><td>" ;
    if (opmode == 5) {
        page_content +=WiFi.macAddress().c_str();
    } else {
        page_content +=WiFi.softAPmacAddress().c_str();
    }
    page_content +="</td></tr>\n";
    page_content +="</table>\n";

    page_content +="<p>&nbsp;</p>\n";

    // **** file system (SPIFFS) ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px;\"><b>File System (SPI Flash File System):</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";
    page_content +="<tr><td style=\"width: 225px; \">Total KB:</td><td>" ;
    page_content +=String((float)fs_info.totalBytes / 1024.0);
    page_content +="</td></tr>\n";
    page_content +="<tr><td style=\"width: 225px; \">Used KB:</td><td>" ;
    page_content +=String((float)fs_info.usedBytes / 1024.0);
    page_content +="</td></tr>\n";
    page_content +="</table>\n";

    page_content +="<p>&nbsp;</p>\n";

    // **** memory information ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px;\"><b>Memory information:</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";

    page_content +="<tr><td style=\"width: 225px; \">free heap measure(1):</td><td>" ;
    page_content +=system_get_free_heap_size();
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">free heap measure(2):</td><td>" ;
    page_content +=String(ESP.getFreeHeap());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">% heap fragmentation:</td><td>" ;
    page_content +=String(ESP.getHeapFragmentation());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">max allocatable ram block size:</td><td>" ;
    page_content +=String(ESP.getMaxFreeBlockSize());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">Sketch thinks Flash RAM (MB) is:</td><td>" ;
    page_content +=String((float)ESP.getFlashChipSize() / 1024.0 / 1024.0);
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">Actual Flash RAM (MB):</td><td>" ;
    page_content +=String((float)ESP.getFlashChipRealSize() / 1024.0 / 1024.0);
    page_content +="</td></tr>\n";

    page_content +="</table>\n";

    page_content +="<p>&nbsp;</p>\n";

    // **** firmware information ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px;\"><b>Firmware:</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";
    page_content +="<tr><td style=\"width: 225px; \">chip Id::</td><td>" ;
    page_content +=String(ESP.getChipId());
    page_content +="</td></tr>\n";
    page_content +="<tr><td>core version::</td><td>" ;
    page_content +=ESP.getCoreVersion();
    page_content +="</td></tr>\n";
    page_content +="<tr><td>SDK version::</td><td>" ;
    page_content +=String(ESP.getSdkVersion());
    page_content +="</td></tr>\n";
    page_content +="</table>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


// ---------------------------------------------
// create the parameter type selection web page
// ---------------------------------------------
String HTMLparameter_selection(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>parameter type selection</h3>\n";
    page_content +="<p><a class=\"button button-off\" href=\"WiFi_params\"><button>WiFi parameters</button></a></p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"robot_updates\"><button>Robot parameters</button></a></p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"demo_updates\"><button>Demo action list</button></a></p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"GPIO_updates\"><button>GPIO pin numbers</button></a></p>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";


    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}

// ---------------------------------------------
// create the demo action list update web page
// ---------------------------------------------
String HTMLdemo_updates(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>Demo action list update</h3>\n";

    // list in a table all the current 20 demo actions that are defined 
    page_content +="<table align=\"center\">\n";
    for (int i=0; i<=19; i++){
        page_content +="<tr><td width=\"100px\">Action [";
        page_content +=i;
        page_content +="]</td><td width=\"100px\">";
        page_content +=demo_controls[i*2];
        page_content +="</td><td width=\"60px\">";
        page_content +=demo_controls[i*2+1];
        page_content +="</td></tr>\n";
    }
    page_content +="</table>\n";
    page_content +="<p>&nbsp;</p>\n";

    // input section of the web page
    page_content +="<form method=\"post\" action=\"/demo_file\"> \n";
    page_content +="Action number: \n";
    page_content +="<input type=\"text\" name=\"demo_num_str\" size=\"6\"> &nbsp; &nbsp;";
    page_content +="<input type=\"text\" name=\"demo_act_str\" size=\"12\"> &nbsp; &nbsp;";
    page_content +="<input type=\"text\" name=\"demo_val_str\" size=\"6\"> \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</form>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


// ---------------------------------------------
// create the WiFi parameter update web page
// ---------------------------------------------
String HTMLWiFi_params(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>WiFi parameter update</h3>\n";
    page_content +="<h3>input/update SSID name and WEP key</h3>\n";

    // input sections of the web page

    page_content +="<form method=\"post\" action=\"/WiFi_updates1\"> \n";
    page_content +="SSID1: \n";
    page_content +="<input type=\"text\" name=\"ssid_1\" size=\"12\" value= ";
    page_content +=ssid1;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_1\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates2\"> \n";
    page_content +="SSID2: \n";
    page_content +="<input type=\"text\" name=\"ssid_2\" size=\"12\" value= ";
    page_content +=ssid2;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_2\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates3\"> \n";
    page_content +="SSID3: \n";
    page_content +="<input type=\"text\" name=\"ssid_3\" size=\"12\" value= ";
    page_content +=ssid3;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_3\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates4\"> \n";
    page_content +="SSID4: \n";
    page_content +="<input type=\"text\" name=\"ssid_4\" size=\"12\" value= ";
    page_content +=ssid4;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_4\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates5\"> \n";
    page_content +="SSID5: \n";
    page_content +="<input type=\"text\" name=\"ssid_5\" size=\"12\" value= ";
    page_content +=ssid5;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_5\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


// ---------------------------------------------
// create the robot demo-run control web page
// ---------------------------------------------
String HTMLrobot_demo_run(){
    String page_content = header_content;
    // start of robot demo control web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>demo run controls</h3>\n";
    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show demo-run web controls</h3>\n";
    } else {
        if(robot_state == 1) {
            page_content +="<p><a class=\"button button-on\" href=\"run_demo\"><button>DEMO RUN</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"run_demo\"><button>DEMO RUN</button></a></p>\n";
        }

        if(robot_state == 0) {
            page_content +="<p><a class=\"button button-on\" href=\"stop_demo\"><button>DEMO STOP</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"stop_demo\"><button>DEMO STOP</button></a></p>\n";
        }
    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the robot auto-run control web page
// ---------------------------------------------
String HTMLrobot_auto_run(){
    String page_content = header_content;
    // start of robot auto-run control web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>auto-run controls</h3>\n";
    // input sections of the web page
    // input the robot speed to be used as a % from 0-100
    page_content +="<form method=\"post\" action=\"/autospeed\"> \n";
    page_content +="Robot speed: \n";
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"3\" value= ";
    page_content +=str_r_speed;
    page_content += " >\n";
    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show auto-run web controls</h3>\n";
    } else {
        if(robot_state == 1) {
            page_content +="<p><a class=\"button button-on\" href=\"run_auto\"><button>AUTO RUN</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"run_auto\"><button>AUTO RUN</button></a></p>\n";
        }

        if(robot_state == 0) {
            page_content +="<p><a class=\"button button-on\" href=\"run_stop\"><button>STOP</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"run_stop\"><button>STOP</button></a></p>\n";
        }
    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the robot run-about control web page
// ---------------------------------------------
String HTMLrobot_run_about(){
    String page_content = header_content;
    // start of robot run-about control web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>run about controls - autostop: ";
    page_content +=stopauto;
    page_content +="</h3>\n";
    // input sections of the web page
    // input the robot speed to be used as a % from 0-100
    page_content +="<form method=\"post\" action=\"/postspeed\"> \n";
    page_content +="Robot speed: \n";
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"3\" value= ";
    page_content +=str_r_speed;
    page_content += " >\n";
    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show run-about web controls</h3>\n";
    } else {
        if(robot_state == 1) {
            page_content +="<p><a class=\"button button-on\" href=\"forward\"><button>FWD</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"forward\"><button>FWD</button></a></p>\n";
        }
        page_content +="<p><a class=\"button btninline button-off\" href=\"turnleft\"><button>TURN<br/>LEFT</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"turnright\"><button>TURN<br/>RIGHT</button></a></p>\n";
        page_content +="<p><a class=\"button button-on\" href=\"halt\"><button>STOP</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-off\" href=\"spinleft\"><button>SPIN<br/>LEFT</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"spinright\"><button>SPIN<br/>RIGHT</button></a></p>\n";
        if(robot_state == 2) {
            page_content +="<p><a class=\"button button-on\" href=\"backward\"><button>BACK</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"backward\"><button>BACK</button></a></p>\n";
        }
    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the component testing web page
// ---------------------------------------------
String HTMLrun_tests(){
    String page_content = header_content;
    // start of component testing web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>component testing</h3>\n";
    // input sections of the web page
    // input the motor speed to be used as a % from 0-100
    page_content +="<form method=\"post\" action=\"/testspeed\"> \n";
    page_content +="Motor test speed: \n";
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"3\" value= ";
    page_content +=str_r_speed;
    page_content += " >\n";
    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show component testing controls</h3>\n";
    } else {
        page_content +="<p><a class=\"button btninline button-off\" href=\"leftfwd\"><button>LEFT<br/>FWD</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"leftback\"><button>LEFT<br/>BACK</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-off\" href=\"rightfwd\"><button>RIGHT<br/>FWD</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"rightback\"><button>RIGHT<br/>BACK</button></a></p>\n";
        page_content +="<p><a class=\"button button-on\" href=\"teststop\"><button>STOP</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-on\" href=\"USread\"><button>U-SONIC<br/>SENSOR<br/>";
        page_content +=cm_str;
        page_content +=" cm</button></a></p>\n";

    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the GPIO pin parameter update web page
// ---------------------------------------------
String HTMLGPIO_updates(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>GPIO pin usage update</h3>\n";
    page_content +="<h3>** these pin# settings should rarely be changed **</h3>\n";
    page_content +="<h3>input/update GPIO pin number</h3>\n";

    // input sections of the web page
    page_content +="<form method=\"post\" action=\"/GPIO_pins\"> \n";

    page_content +="slide switch AB: \n";
    page_content +="<input type=\"text\" name=\"s_AB_str\" size=\"6\" value= ";
    page_content +=str_s_AB;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="slide switch CD: \n";
    page_content +="<input type=\"text\" name=\"s_CD_str\" size=\"6\" value= ";
    page_content +=str_s_CD;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="slide switch GO-STOP: \n";
    page_content +="<input type=\"text\" name=\"onoff_str\" size=\"6\" value= ";
    page_content +=str_onoff;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="ultrasonic trig: \n";
    page_content +="<input type=\"text\" name=\"trigPin_str\" size=\"6\" value= ";
    page_content +=str_trigPin;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="ultrasonic echo: \n";
    page_content +="<input type=\"text\" name=\"echoPin_str\" size=\"12\" value= ";
    page_content +=str_echoPin;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</form>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


// -----------------------------------------------------
// create the robot operation parameter update web page
// -----------------------------------------------------
String HTMLrobot_updates(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP8266 Robot Web Server</h1>\n";
    page_content +="<h3>robot operation parameters update</h3>\n";
    page_content +="<h3>input/update the individual operational parameters</h3>\n";

    // input sections of the web page
    page_content +="<form method=\"post\" action=\"/robot_params\"> \n";


    write_text("/str_motorL.txt", str_motorL);
    write_text("/str_motorR.txt", str_motorR);


    page_content +="take avoidance action distance (cm): \n";
    page_content +="<input type=\"text\" name=\"avoid_distance_str\" size=\"8\" value= ";
    page_content +=str_avoid_distance;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="avoidance sensor repeats: \n";
    page_content +="<input type=\"text\" name=\"avoid_limit_str\" size=\"8\" value= ";
    page_content +=str_avoid_limit;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="avoidance action spin time (ms): \n";
    page_content +="<input type=\"text\" name=\"avoidspintime_str\" size=\"8\" value= ";
    page_content +=str_avoidspintime;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="avoidance action reverse time (ms): \n";
    page_content +="<input type=\"text\" name=\"ReverseTime_str\" size=\"8\" value= ";
    page_content +=str_ReverseTime;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="activate autostop (yes/no): \n";
    page_content +="<input type=\"text\" name=\"stopauto_str\" size=\"8\" value= ";
    page_content +=stopauto;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="autostop distance (cm): \n";
    page_content +="<input type=\"text\" name=\"stop_distance_str\" size=\"8\" value= ";
    page_content +=str_stop_distance;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="robot turn time (ms): \n";
    page_content +="<input type=\"text\" name=\"turntime_str\" size=\"8\" value= ";
    page_content +=str_turntime;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="robot turn speed (%): \n";
    page_content +="<input type=\"text\" name=\"turnspeed_str\" size=\"8\" value= ";
    page_content +=str_turnspeed;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="robot spin time (ms): \n";
    page_content +="<input type=\"text\" name=\"spintime_str\" size=\"8\" value= ";
    page_content +=str_spintime;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="robot spin speed (%): \n";
    page_content +="<input type=\"text\" name=\"spinspeed_str\" size=\"8\" value= ";
    page_content +=str_spinspeed;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="default robot speed (%): \n";
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"8\" value= ";
    page_content +=str_r_speed;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="left motor balance ratio: \n";
    page_content +="<input type=\"text\" name=\"motorL_str\" size=\"8\" value= ";
    page_content +=str_motorL;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="right motor balance ratio: \n";
    page_content +="<input type=\"text\" name=\"motorR_str\" size=\"8\" value= ";
    page_content +=str_motorR;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";


    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</form>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


