// £15 robot project
// SPIFFS_setup01.ino - sets all the default robot operation parameters into SPIFFS files

#include <ESP8266WiFi.h>
#include <FS.h>

// use a global variable to simplify a text function for writing to a file
String passed_text = "";

int NSSIDs = 5;
String ssid1 = "ssid01";     // 1st SSID
String ssid2 = "ssid02";     // 2nd SSID
String ssid3 = "ssid03";     // 3rd SSID
String ssid4 = "ssid04";     // 4th SSID
String ssid5 = "ssid05";     // 5th SSID
String password1 = "pass01";   // 1st SSID password
String password2 = "pass02";   // 2nd SSID password
String password3 = "pass03";   // 3rd SSID password
String password4 = "pass04";   // 4th SSID password
String password5 = "pass05";   // 5th SSID password

int mode_1 = 0;
int mode_2 = 0;
int mode_3 = 0;
int mode_4 = 1;  // softAP web interface
int mode_5 = 1;  // local WiFi web interface
int mode_6 = 1;  // autonomous run around
int mode_7 = 0; 
int mode_8 = 1;  // demo mode

int num_switches = 3;
int s_AB  = 14;
int s_CD  = 12;
int s_EF  = 3;   // not used in this build so num_switches should be set to 3
int onoff = 16;

int trigPin = 13;   // pin [D7] GPIO#13 used for the sensor initiate pin (trig)
int echoPin = 15;   // pin [D8] GPIO#15 used for the sensor response pin (echo)

int inchconv = 147; // ratio between pulse width and inches
int cmconv = 59;    // ratio between pulse width and cm
int avoid_distance = 20;   // set a defined distance to take avoidance action
int stop_distance = 15;    // set the emergency stop distance for web use
int avoid_limit = 3;       // number of times distance limit is reached before action taken - avoids spurious readings
int avoidspintime = 1200;   // time in ms to spin in the avoidance routine
int turntime = 1200;     // time in ms to do a 90-deg turn
int spintime = 1200;     // time in ms to do a 90-deg spin
int turnspeed = 90;      // turning speed as a %
int spinspeed = 90;      // spinning speed as a %
int r_speed = 90;       // default speed % as an integer value  0-100%
int ReverseTime = 350;  // time in ms to reverse in the avoidance routine
String stopauto = "yes";   // set whether autostop should be used 

String str_r_speed = "90";  // string version of the speed used in the web interface
float motorL = 1.0;   
float motorR = 1.0;

int num_demo = 40;
String demo_controls[40] ={};
String demo_actions[8] ={};

void setup() {

    Serial.begin(115200);
    delay(1000);
    Serial.println("  ");
    Serial.println("  ");
    Serial.println("-------------------------------");
    Serial.println("Program started: Serial Monitor set up");
    Serial.println("-------------------------------");

    String str_NSSIDs = String(NSSIDs);
    String str_mode_1 = String(mode_1);
    String str_mode_2 = String(mode_2);
    String str_mode_3 = String(mode_3);
    String str_mode_4 = String(mode_4);
    String str_mode_5 = String(mode_5);
    String str_mode_6 = String(mode_6);
    String str_mode_7 = String(mode_7);
    String str_mode_8 = String(mode_8);

    String str_num_switches  = String(num_switches);
    String str_s_AB = String(s_AB);
    String str_s_CD = String(s_CD);
    String str_s_EF = String(s_EF);
    String str_onoff = String(onoff);

    String str_trigPin = String(trigPin);
    String str_echoPin = String(echoPin);

    String str_inchconv = String(inchconv);
    String str_cmconv = String(cmconv);
    String str_avoid_distance = String(avoid_distance);
    String str_stop_distance = String(stop_distance);
    String str_avoid_limit = String(avoid_limit);
    String str_avoidspintime = String(avoidspintime);
    String str_turntime = String(turntime);
    String str_spintime = String(spintime);
    String str_turnspeed = String(turnspeed);
    String str_spinspeed = String(spinspeed);
    String str_r_speed = String(r_speed);
    String str_ReverseTime = String(ReverseTime);

    String str_motorL = String(motorL);   
    String str_motorR = String(motorR);

    demo_controls[0] = "FWD";
    demo_controls[1] = "60";
    demo_controls[2] = "DELAY";
    demo_controls[3] = "1000";
    demo_controls[4] = "STOP";
    demo_controls[5] = "0";
    demo_controls[6] = "BACK";
    demo_controls[7] = "60";
    demo_controls[8] = "DELAY";
    demo_controls[9] = "1000";
    demo_controls[10] = "STOP";
    demo_controls[11] = "0";
    demo_controls[12] = "SPINL";
    demo_controls[13] = "1200";
    demo_controls[14] = "BACK";
    demo_controls[15] = "60";
    demo_controls[16] = "DELAY";
    demo_controls[17] = "1000";
    demo_controls[18] = "STOP";
    demo_controls[19] = "0";
    demo_controls[20] = "SPINR";
    demo_controls[21] = "1200";
    demo_controls[22] = "BACK";
    demo_controls[23] = "60";
    demo_controls[24] = "DELAY";
    demo_controls[25] = "1000";
    demo_controls[26] = "STOP";
    demo_controls[27] = "0";

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
    //  before writing to file
    Serial.println("checking demo_controls before writing to file");
    for (int i=0; i<=num_demo-1; i=i+2){
        Serial.print("checking demo_controls: "); 
        Serial.println(demo_controls[i]); 
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


    // ********************************
    // start the SPI Flash File System
    // ********************************
    if (!SPIFFS.begin()) {
        // Serious problem
        Serial.println("SPIFFS mount failed");
    } else {
        Serial.println("SPIFFS mounted OK");
    }

    // ***********************************************************************
    // Format File System  - only do this once for the individual NodeMCU
    //  because it completely wipes what ever may have been stored previously
    // ***********************************************************************
    if (SPIFFS.format()) {
        Serial.println("File System Formatted");
    } else {
        Serial.println("File System Formatting Error");
    }


    // *********************************************************
    // Create/Open multiple string file and write strings to it
    // *********************************************************
    Serial.println("-------------------------------");
    write_strings("/demo_controls.txt", demo_controls, num_demo);
    Serial.println("-------------------------------");


    // *******************************************************
    // Create/Open Existing Files And Write Text Data To Them
    // *******************************************************

    write_text("/str_NSSIDs.txt",str_NSSIDs);
    Serial.println("-------------------------------");
    write_text("/ssid1.txt", ssid1);
    write_text("/ssid2.txt", ssid2);
    write_text("/ssid3.txt", ssid3);
    write_text("/ssid4.txt", ssid4);
    write_text("/ssid5.txt", ssid5);
    Serial.println("-------------------------------");    
    write_text("/password1.txt", password1);
    write_text("/password2.txt", password2);
    write_text("/password3.txt", password3);
    write_text("/password4.txt", password4);
    write_text("/password5.txt", password5);
    Serial.println("-------------------------------");
    write_text("/str_r_speed.txt", str_r_speed);
    Serial.println("-------------------------------");
    write_text("/str_mode_1.txt", str_mode_1);
    write_text("/str_mode_2.txt", str_mode_2);
    write_text("/str_mode_3.txt", str_mode_3);
    write_text("/str_mode_4.txt", str_mode_4);
    write_text("/str_mode_5.txt", str_mode_5);
    write_text("/str_mode_6.txt", str_mode_6);
    write_text("/str_mode_7.txt", str_mode_7);
    write_text("/str_mode_8.txt", str_mode_8);
    Serial.println("-------------------------------");
    // switch GPIO numbers NOT written to file so that
    //  they can have a default set of values in the main code
    //write_text("/str_num_switches.txt", str_num_switches);
    //write_text("/str_s_AB.txt", str_s_AB);
    //write_text("/str_s_CD.txt", str_s_CD);
    //write_text("/str_s_EF.txt", str_s_EF);
    //write_text("/str_onoff.txt", str_onoff);
    Serial.println("-------------------------------");
    write_text("/str_trigPin.txt", str_trigPin);
    write_text("/str_echoPin.txt", str_echoPin);
    Serial.println("-------------------------------");
    write_text("/str_inchconv.txt", str_inchconv);
    write_text("/str_cmconv.txt", str_cmconv);
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

    // ************************************
    // List all the FSinfo
    // ************************************

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


    // ****************************************************
    // read the multiple string data from the written file
    // ****************************************************
    // reset the demo_controls[i] array to null before reading it back in
    for (int i=0; i<=num_demo-1; i++){
        demo_controls[i] = " ";
    }
    read_strings("/demo_controls.txt", demo_controls, num_demo);
    Serial.println("-------------------------------");
    Serial.println("demo_control strings: - read from file");
    for (int i=0; i<=num_demo-1; i++){
        Serial.print(i);
        Serial.print(": ");
        Serial.println(demo_controls[i]);
    }
    Serial.println(" ");
    Serial.println(" ");
    Serial.println("-------------------------------");
    // print out the individual characters of the 1st string to check it in more detail
    String checkstr;
    checkstr = demo_controls[0];
    Serial.print("length of demo_control string: ");
    Serial.println(demo_controls[0].length());
    Serial.print("length of checkstr string: ");
    Serial.println(checkstr.length());
    for (int i=0; i<=checkstr.length()-1; i++) {
        Serial.print(checkstr[i]);
    }
    Serial.println("-------------------------------");

    // check demo_controls has sensible demo_actions
    //  after writing to and reading back from file
    Serial.println("checking demo_controls after writing to and reading back from file");
    for (int i=0; i<=num_demo-1; i=i+2){           // loop thru all the demo_controls 2 at a time
        Serial.print("checking demo_controls: "); 
        Serial.println(demo_controls[i]); 
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


    // ************************************************
    // now read the individual strings from their files
    // ************************************************
    Serial.println("-------------------------------"); 
    read_text("/str_NSSIDs.txt");
    Serial.println("-------------------------------");
    ssid1 = read_text("/ssid1.txt");
    ssid2 = read_text("/ssid2.txt");
    ssid3 = read_text("/ssid3.txt");
    ssid4= read_text("/ssid4.txt");
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
    //str_num_switches = read_text("/str_num_switches.txt");
    //str_s_AB = read_text("/str_s_AB.txt");
    //str_s_CD = read_text("/str_s_CD.txt");
    //str_s_EF = read_text("/str_s_EF.txt");
    //str_onoff = read_text("/str_onoff.txt");
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

    //num_switches = str_num_switches.toInt();
    //s_AB = str_s_AB.toInt();
    //s_CD = str_s_CD.toInt();
    //s_EF = str_s_EF.toInt();
    //onoff = str_onoff.toInt();

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
    r_speed = str_r_speed.toInt();
    motorL = str_motorL.toFloat();
    motorR = str_motorR.toFloat();
    ReverseTime = str_ReverseTime.toInt();

    // *************************************************
    // print out the integers and floats to check them
    // *************************************************
    Serial.println("-------------------------------");
    Serial.print("mode_1 = ");
    Serial.println(mode_1);
    Serial.print("mode_2 = ");
    Serial.println(mode_2);
    Serial.print("mode_3 = ");
    Serial.println(mode_3);
    Serial.print("mode_4 = ");
    Serial.println(mode_4);   
    Serial.print("mode_5 = ");
    Serial.println(mode_5);   
    Serial.print("mode_6 = ");
    Serial.println(mode_6);    
    Serial.print("mode_7 = ");
    Serial.println(mode_7);   
    Serial.print("mode_8 = ");
    Serial.println(mode_8);    
    Serial.println("-------------------------------");
    //Serial.print("num_switches = ");
    //Serial.println(num_switches);
    //Serial.print("s_AB = ");
    //Serial.println( s_AB);
    //Serial.print("s_CD = ");
    //Serial.println(s_CD);
    //Serial.print("s_EF = ");
    //Serial.println(s_EF);
    //Serial.print("onoff = ");
    //Serial.println(onoff);
    Serial.println("-------------------------------");
    Serial.print("trigPin = ");
    Serial.println(trigPin);
    Serial.print("echoPin = ");
    Serial.println(echoPin);
    Serial.println("-------------------------------");
    Serial.print("inchconv = ");
    Serial.println(inchconv);
    Serial.print("cmconv = ");
    Serial.println(cmconv);
    Serial.print("avoid_distance = ");
    Serial.println(avoid_distance);
    Serial.print("stop_distance = ");
    Serial.println(stop_distance);
    Serial.print("avoid_limit = ");
    Serial.println(avoid_limit);
    Serial.print("avoidspintime = ");
    Serial.println(avoidspintime);
    Serial.print("turntime = ");
    Serial.println(turntime);
    Serial.print("spintime = ");
    Serial.println(spintime);
    Serial.print("turnspeed = ");
    Serial.println(turnspeed);
    Serial.print("spinspeed = ");
    Serial.println(spinspeed);
    Serial.print("r_speed = ");
    Serial.println(r_speed);
    Serial.print("motorL = ");
    Serial.println(motorL);
    Serial.print("motorR = ");
    Serial.println(motorR);
    Serial.print("ReverseTime = ");
    Serial.println(ReverseTime);
    Serial.print("stopauto = ");
    Serial.println(stopauto);

    Serial.println("-------------------------------");
    Serial.println("Program finished");
    Serial.println("-------------------------------");

}

void loop() {

    // put your main code here, to run repeatedly:

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
        Serial.println(str_read);
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
            tempstr=SPIfileStr.readStringUntil('\n');     // [i] string read upto the \n terminator
            //    the terminator is discarded from the stream but adds a space which must be removed
            if (tempstr.length() > 2) {   /// only remove the space from an 'action' that has been set
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
