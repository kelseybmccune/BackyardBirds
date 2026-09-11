/*
  Data logging sketch for the ETAG RFID Reader
  ETAG READER Version 10.0
  PROCESSOR SAMD21J
  USE BOARD DEFINITION: ETAG RFID V2 D21
  Code by:
   Jay Wilhelm
   Tho Trinh
   Eli Bridge
   Alexander Moreno
   David Mitchell
  Modified by Jonathon Valente Mar 2025

  Sept 2019

  Updates for 2022
  Ignore line returns in menu

  Licenced in the public domain

  REQUIREMENTS:
  Power supply for the board should come from the USB cable or a 5V battery or DV power source.
  A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date
      and time when the board is disconnected from a primary power source.

  LIBRARIES:
  The RV3129 library is accessible on github here:
  https://github.com/OUIDEAS/RV-3129_Arduino_Library.git

  FLASH MEMORY STRUCTURE:
  Adesto® AT45DB321E
  34,603,008 bits of memory are organized as 8,192 pages of 528 bytes each.
  23-bit address sequence - The first 13 bits (PA12 - PA0) specify the page.
  The last 10 bits (BA9 - BA0) specify the byte address within the page.
  Data can be written/erased as pages, blocks (8 pages) or sectors (usually 128 pages).
  First sector = pages 0-7; others are much larger
  Pages count up as 0x00000000, 0x00000400, 0x00000800, 0x00000C00, 0x00001000, 0x00001400, 0x00001800, 0x00001C00, 0x00002000
                    page 0      page 1      page 2      page 3      page 4      page 5      page 6      page  7     page 8
  To get page address, take just the page number (1, 2, 3) and shft it 10 bits left.

  Page 0 is reserved for parameters and counters (Addresses 0x00000400 to ??
    byte 3 - (0x403) Set to 0xAA once the memory address counter is intialized. (important mainly for the very first initialization process with a completely blank Flash memory)
    bytes 4-7 = (0x404-0x407) = reader ID charaters
    byte 13 = Logging mode - log to SD in real time or just use Flash
    byte 14 = Feeder mode (feedMode) -- food dispensing mode
    byte 30 = radio network number
    byte 31 = network group ID
    byte 32 = radio node ID
    byte 33 = target radio ID
    byte 34 = number of nodes in network (must start with 1 and count up)
    byt3 35 = maximum group number (min is always 1) 
    byte 36/37 = number of tags in storage - 16 bit integer
    byte 36 = number of nodes in network (must start with 1 and count up)
    byt3 37 = maximum group number (min is always 1) 
    bytes 40 to 59 -- Group progression bytes (could just have max node and make progression go in order.

  Pages 1-7 are reserved for RFID tag codes. (sector 1a is pages 0-7)
    Each tag line is 5 RFID bytes, 1 current group btye, 2 trial history bytes, and 1 byte for number of reversals + 1 empty byte = 10 per line
  Pages 8 (0x00 00 20 00) to 127 (all of sector 1b) are for logging and error codes.
  Remaining pages (and sectors) are for RFID data. First address for data storage is page 128 (0x00 02 00 00)

  Radio RF69CHW communication works within a single network. Comms among feeders happens in a loop--each feeder transmits only to one target
  node, which then transmits to its target and so on until the message gets back to the originator. Each node has its own node number and taget
  node number

  Flash Data structure for RFID logging;
  0438D4C2A6,07/13/2021 09:45:34,A101,1,001,100101111011
  RFID tag (5 bytes, 0 to 4); Date(4 Bytes, 5 to 8); FeederID (3 bytes, 9 to 11); Current Group (1 Byte, 12); Trial (1 Byte, 13); history (2 bytes, 14 to 15)

  Flash Data Structure for Trial Information 
  {RFarr[0], RFarr[1], RFarr[2],          // Create an array representing an entire line of data
                          RFarr[3], RFarr[4], c2, 0,
                          0, 0, 0xFF
  0438D4C2A6 1 4 100101111011 255
  RFID tag (5 bytes, 0 to 4), Current group (1 byte), hisotry (2 bytes), trials (1 byte), dummy (255) = 10 bytes

  
   

  Nov 8, 2019 - Added Memory address lookup - address pointer no longer used.
  Nov 10, 2019 - Added dual logging modes.
  Nov 10, 2019 - Fixed bug in sleepTimer function that stopped the clock.
  Dec 15, 2019 - Changed default mode to no-SD, added LED flash when logging starts
  July 2022 - revised memory structure to allow for more tag storage.
          - revised memory to compress times using unix format
          - set up serial input code to ignore line feeds and returns
          - revised data transfer to generate old and new files.

  log events are coded as follows
  1 - "Logging_started"
  2 - "Go_to_sleep"
  3 - "Wake_from_sleep"

  Build in clock error detection??

*/

/* Motor for grackle feeder - hook up motor positive (red) to output A1 on motor controller
   digitalWrite(MOTRA, HIGH); digitalWrite(MOTLA, LOW) -- elevator goes down
   digitalWrite(MOTRA, LOW); digitalWrite(MOTLA, HIGH) -- elevator goes up

   Motor swith on pin 12
   Motor switch is open when plunger is extended, and will read high on the motor switch pin.
   Motor swith is closed when the button is depressed and will read low.
   
*/


// ***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
#include "RV3129.h"          // include library for the real time clock - must be installed in libraries folder
#include <Wire.h>            // include the standard wire library - used for I2C communication with the clock
#include <SD.h>              // include the standard SD card library
#include <SPI.h>             // include standard SPI library
#include "Manchester.h"
#include <RFM69.h>


#define Serial SerialUSB       // Designate the USB connection as the primary serial comm port - note lowercase "serial"
#define SD_FAT_TYPE         3  // Type 3 Reads all card formats
#define SPI_SPEED          SD_SCK_MHZ(4) .  //Finds fastest speed.
#define SDselect           46  // (PA21) Chip select for SD card. Make this pin low to activate the SD card for SPI communication
#define SDon               45  // (PA06) Provides power to the SD via a high side mosfet. Make this pin low to power on the SD card
#define FlashCS            44  // (PA05) Chip select for flash memory. Make this pin low to activate the flash memory for SPI communication
#define LED_RFID           43  // (PA27) Pin to control the LED indicator.
#define INT1               47  // (PA20) Clock interrupt for alarms and timers on the RTC
#define MOTR               2   // used for sleep function (need to investigate this). 
#define RF69_CS            8   // chip select pin for radio

//Motor control pins
#define MOTPWMA      5  //motor pulse width
#define MOTLA        4  //motor left
#define MOTRA        3  //motor right
#define mStby        6  //motor controller standby
#define MOTPWMB      0  //motor controller pulse width B 
#define MOTLB        2  //GPS power control - connected to B_in_1 of motor controller
#define MOTRB        1  //GPS power control - connected to B_in_2 of motor controller 
// #define mSwitch      12 //motor switch input
// #define feedInt      13  //food sensor 

//Radio defines
#define FREQUENCY     RF69_433MHZ
#define USEACK        true // Request ACKs or not
#define ENCRYPT       false // Set to "true" to use encryption


RV3129 rtc;   //Initialize an instance for the RV3129 real time clock library.
RFM69 radio;

// ************************* initialize variables******************************
char deviceID[5] = "RFID";            // User defined name of the device
String deviceIDstr;                   // String verion of device name.
String dataFile;                      // Stores text file name for SD card writing.
String logFile;                       // Stores text file name for SD card writing.

union             //Make a union structure for dealing with unix time conversion
{struct
  {uint8_t b1; uint8_t b2; uint8_t b3; uint8_t b4;}; //four bytes that make up unix time 
  uint32_t unixLong;                                //Name for 32 bit unix time value
} unixTime;                                         //Name of the union variable (use unixTime.unixLong or unixTime.b3)

union                          // Make a union structure for feeding success history
{struct
  {uint8_t h1; uint8_t h2;};    // 2 bytes that make up unix time 
  uint16_t h16b;               // Name for 16bbit variable
} history;                     // Name of the union variable (use history.h16b or history.h1)

uint32_t unixPast = 0;     // stores the past unix time - for use with delayTime

String currRFID;                      // stores current RFID number and RF circuit - for use with delayTime
String pastRFID = "XXXXXXXXXX1";      // stores past RFID number and RF circuit - for use with delayTime

uint32_t flashAddr;
uint32_t logFlashAddr;
uint32_t dataLine;                   // tracks current data line for writing to Flash memory - starts at 0
uint32_t logLine;
uint16_t pageAddress;             // page address for flash memory
uint16_t byteAddress;             // byte address for flash memory
uint16_t logPageAddress;          // page address for flash memory log messages
uint16_t logByteAddress;          // byte address for flash memory log messages
uint16_t lastDataAddr = 510;       // last byte address used for writing RFID data (10 bytes per line)
uint16_t lastLogAddr = 520;        // last byte address used for writing RFID data (5 bytes per line)
const uint8_t SDLineLength = 56;   // includes carriage return and line feed. 

byte RFcircuit = 1;                   // Used to determine which RFID circuit is active. 1 = primary circuit, 2 = secondary circuit.
String currentDate;                   // USed to get the current date in mm/dd/yyyy format (we're weird in the US)
String currentTime;                   // Used to get the time
String currentDateTime;               // Full date and time string
String timeString;                    // String for storing the whole date/time line of data
unsigned int timeIn[12];              // Used for incoming serial data during clock setting
String logMessage;
byte menu;                            // Keeps track of whether the menu is active.

// Global variable for tag codes

char RFIDstring[10];                  // Stores the TagID as a character array (10 character string)
byte RFIDtagUser = 0;                 // Stores the first (most significant) byte of a tag ID (user number)
uint32_t RFIDtagNumber = 0;      // Stores bytes 1 through 4 of a tag ID-omits first byte (user number)
byte RFIDtagArray[5];                 // Stores the five individual bytes of a tag ID.

// Character arrays for text handling
char cArray1[64];                     // Global general purpose character array - Can store 64 characters
char cArray2[64];                     // Global general purpose character array - Can store 64 characters

// ***************FOOD DISPENSER VARIABLES**************
uint8_t feedState = 0;       // status of feeding mechanism: 1=ready, 2=open, 3=reloading (elevator high) 
uint8_t matchFound = 0;      // results from searching for tag ID match among stored IDs
uint8_t foodOK = 0;          // detection of food falling
uint8_t motOn = 1;          // make zero to turn motor off for testing
// ***************RADIO VARIABLES***************

char radioNet = '1';   //radio network number (on flash page 0, byte 30)
uint8_t netGrp = '1';     //group id within network (on flash page 0, byte 31)
uint8_t nodeID = 0;     //radio node number for this feeder (on flash page 0, byte 32)
uint8_t targetNode = 0; //target node for all transmissions (on flash page 0, byte 33)
uint16_t numTagsStored = 0;  // number of tags stored in Flash memory
uint8_t maxGrp;    // maxium group number
uint8_t numNodes;  // number of nodes in the network.
uint32_t syncTime; // unix time value used for clock synchonization.
uint8_t transArray[15]; // Array for transmission of data.

// ********************CONSTANTS (SET UP LOGGING PARAMETERS HERE!!)*******************************
const byte checkTime = 30;                          // How long in milliseconds to check to see if a tag is present (Tag is only partially read during this time -- This is just a quick way of detirmining if a tag is present or not
const unsigned int pollTime1 = 200;                 // How long in milliseconds to try to read a tag if a tag was initially detected (applies to both RF circuits, but that can be changed)
const unsigned int delayTime = 8;                   // Minimim time in seconds between recording the same tag twice in a row (only applies to data logging--other operations are unaffected)
const unsigned long pauseTime = 100;                // CRITICAL - This determines how long in milliseconds to wait between reading attempts. Make this wait time as long as you can and still maintain functionality (more pauseTime = more power saved)
uint16_t pauseCountDown = pauseTime / 31.25;        // Calculate pauseTime for 32 hertz timer
byte pauseRemainder = ((100 * pauseTime) % 3125) / 100; // Calculate a delay if the pause period must be accurate
//byte pauseRemainder = 0 ;                         // ...or set it to zero if accuracy does not matter
const uint32_t userTimeoutDuration = 15000; // Timeout duration in milliseconds (15 seconds)

const byte slpH = 22;                            // When to go to sleep at night - hour
const byte slpM = 0;                            // When to go to sleep at night - minute
const byte wakH = 5;                            // When to wake up in the morning - hour
const byte wakM = 0;                            // When to wake up in the morning - minute
const unsigned int slpTime = slpH * 100 + slpM;  // Combined hours and minutes for sleep time
const unsigned int wakTime = wakH * 100 + wakM;  // Combined hours and minutes for wake time

/* The reader will output Serial data for a certain number of read cycles;
   then it will start using a low power sleep mode during the pauseTime between read attempts.
   The variable stopCycleCount determines how many read cycles to go
   through before using the low-power sleep.
   Once low-power sleep is enabled, the reader will not be able to output
   serial data, but tag reading and data storage will still work.
*/
unsigned int cycleCount = 0;          // counts read cycles
unsigned int stopCycleCount = 500;     // How many read cycles to maintain serial comminications
bool Debug = 1;                       // Use to stop serial messages once sleep mode is used.
byte SDOK = 1;
char logMode;
char feedMode;

//////SETUP//////////////SETUP//////////////SETUP//////////////SETUP////////

void setup() {  // setup code goes here, it is run once before anything else

  // ********************SET UP INPUT & OUTPUT PINS*******************************
  pinMode(LED_RFID, OUTPUT);      // pin for controlling the on-board LED
  digitalWrite(LED_RFID, HIGH);   // turn the LED off (LOW turns it on)

  pinMode(RF69_CS, OUTPUT);      // Chip select pin for radio set to outpur
  digitalWrite(RF69_CS, HIGH);        // turns off radio card.
  
  pinMode(SDselect, OUTPUT);      // Chip select pin for SD card must be an output
  pinMode(SDon, OUTPUT);          // Chip select pin for SD card must be an output
  pinMode(FlashCS, OUTPUT);       // Chip select pin for Flash memory
  digitalWrite(SDon, LOW);        // turns on the SD card.
  digitalWrite(SDselect, HIGH);   // Make both chip selects high (not selected)
  digitalWrite(FlashCS, HIGH);    // Make both chip selects high (not selected)
  pinMode(SHD_PINA, OUTPUT);      // Make the primary RFID shutdown pin an output.
  digitalWrite(SHD_PINA, HIGH);   // turn the primary RFID circuit off (LOW turns on the EM4095)
  pinMode(SHD_PINB, OUTPUT);      // Make the secondary RFID shutdown pin an output.
  digitalWrite(SHD_PINB, HIGH);   // turn the secondary RFID circuit off (LOW turns on the EM4095)
  pinMode(INT1, INPUT);           // Make the alarm pin an input

  //Motor control pin initializaiton
  pinMode(mStby, OUTPUT);         // pin that can put the motor controller in low-power standby mode
  digitalWrite(mStby, LOW);       // standby pin set low (motor off)
  pinMode(MOTRA, OUTPUT);          // Right motor A control pin
  digitalWrite(MOTRA, LOW);        // turn motor A off
  pinMode(MOTLA, OUTPUT);          // Left motor A control pin
  digitalWrite(MOTLA, LOW);        // turn motor A off
  pinMode(MOTPWMA, OUTPUT);       // pulse widtch (speed control) for motor controller circuit A
  digitalWrite(MOTPWMA, LOW);     // turn pulse off
  pinMode(MOTRB, OUTPUT);          // Right motor B control pin
  digitalWrite(MOTRB, LOW);        // turn motor B off
  pinMode(MOTLB, OUTPUT);          // Left motor B control pin
  digitalWrite(MOTLB, LOW);        // turn motor B off
  pinMode(MOTPWMB, OUTPUT);       // pulse widtch (speed control) for motor controller circuit B
  digitalWrite(MOTPWMB, LOW);     // turn pulse off

  // pinMode(mSwitch, INPUT_PULLUP);  // motor switch enabled as input with internal pullup resistor
  // pinMode(feedInt, INPUT_PULLUP);  // pin that provides a ground reference for the motor switch (optional comment out if not used)
  

  // ********************EXECUTED STARTUP CODE *******************************
  blinkLED(LED_RFID, 4, 400);     // blink LED to provide a delay for serial comms to come online
  serial.begin(9600);
  Serial.println(); Serial.println(); Serial.println();

  //Check flash memory and initialize if needed
  if (readFlashByte(0x00000003) != 0xAA) {          //Read a particular byte from the flash memory; if the byte is 0xFF then the flash memory needs to be initialized
    Serial.println("Initializing Flash Memory..."); //Message
    writeFlashByte(0x00000003, 0xAA);               //Write a different byte to this memory location
  }

  readFlashArray(0x00000004, deviceID, 4); //Get and display the device ID
  if (deviceID[1] == 0xFF) {  //If this byte is empty set a default deviceID
    Serial.println("Setting default device ID - Please update this!!!");
    deviceID[0] = 'R'; deviceID[1] = 'F'; deviceID[2] = '0'; deviceID[3] = '1';
    writeFlashArray(0x00000004, deviceID, 4);  //write to flash memory without updating flash address
  }
  //Serial.print("Device ID: "); Serial.println(String(deviceID));             //Display device ID

  dataLine = getLine(33, 512, 128, 8191, 16);  //provide lines per page, last byte address for all pages, start page, end page, and bytes per line
  Serial.print("Current RFID data line: "); Serial.println(dataLine, DEC);
  logLine = getLine(105, 520, 8, 127, 5);
  Serial.print("Current log data line: "); Serial.println(logLine, DEC);

  logMode = readFlashByte(13);
  if (logMode != 'S' & logMode != 'F') { //If log mode is not established then do so
    Serial.println("Setting log mode to Flash mode");
    writeFlashByte(0x0000000D, 0x46);    //hex 46 is "F"
    logMode = 'F';
  }

  feedMode = readFlashByte(14);
  if (feedMode != 'A' & feedMode != 'T') { //If log mode is not established then do so
    Serial.println("Setting feed mode to A (All)");
    writeFlashByte(14, 'A');
    feedMode = 'A';
  }

  
  readFlashArray(30, cArray1, 8);
  radioNet = cArray1[0];   //radio network number (on flash page 0, byte 30)
  netGrp = cArray1[1];     //group id within network (on flash page 0, byte 31)
  nodeID = cArray1[2];     //radio node number for this feeder (on flash page 0, byte 32)
  targetNode = cArray1[3];  //target node number for transmissions
  numNodes = cArray1[4];
  maxGrp = cArray1[5];
  numTagsStored = (cArray1[6] << 8) + cArray1[7];  //Get number of tags stored in Flash.
  if(numTagsStored==0xFFFF) {   // If number of tags is not initialized, do so and set to zero. 
    numTagsStored = 0;
    cArray1[0] = 0; cArray1[1] = 0;
    writeFlashArray(36, cArray1, 2);
  }

  Serial.print(numTagsStored); Serial.println(" tags in memory.");
  
  Serial.print("Initializing radio for node "); Serial.print(nodeID, HEX); Serial.print(" in group "); Serial.print(netGrp, DEC);
  Serial.print(" on network "); Serial.println(radioNet);
  Serial.print("Total groups: "); Serial.print(maxGrp, DEC); Serial.print("  Total nodes: "); Serial.print(numNodes, DEC);
  Serial.print("  Target node: "); Serial.println(targetNode, DEC);
  
  radio.initialize(FREQUENCY, nodeID, radioNet);
  radio.setHighPower(); // Always use this for RFM69HCW

  if(nodeID != 0xFF & netGrp != 0xFF) { 

     Serial.print("Test transmisstion to target node "); Serial.print(targetNode); Serial.print(" on network "); Serial.println(radioNet);
  
     sprintf(cArray1, "Testing %d %d %d", 1,2,3);
     //Serial.println(cArray1);
     
     if (radio.sendWithRetry(targetNode, cArray1, 15)) {
        Serial.println("ACK received!");
     } else {
        Serial.println("ACK Failed!");
     }
      Serial.println();
  }

   radio.sleep();

  // Initialize SD card
  if (SDwriteString("Message", "DUMMY.TXT")) {     // Write a dummy file to the SD card
    Serial.println("SD card detected.");                // OK message
    SDremoveFile("DUMMY.TXT");                   // delete the dummy file
    updateSD(SDLineLength); //calls large function that evaluates what is on the SD card and updates it to match flash Memory.
    SDstop();
  } else {
    Serial.println("No SD card detected.");       // error message
    if (logMode == 'S') {
      ;
      for (byte i = 0; i < 4; i++) {
        Serial.println("SD card missing or damaged: logging only to flash memory");       // error message
        blinkLED(LED_RFID, 1, 1000);    // long LED flash for warning
        SDOK = 2;                       //indicates SD card not detected
      }
    }
  }

SDOK = 2;

  // start clock functions and check time
  Wire.begin();  //Start I2C bus to communicate with clock.
  if (rtc.begin() == false) {  // Try initiation and report if something is wrong
    Serial.println("Something wrong with clock");
  } else {
    if (rtc.is12Hour() == true) {
      rtc.set24Hour(); //Make sure we are in 24 hour mode??
    }
  }


  //////////////MENU////////////MENU////////////MENU////////////MENU////////
  //Display all of the following each time the main menu is called up.

  byte menu = 1;
  while (menu == 1) {
    Serial.println(); 
    rtc.updateTime();
    Serial.println(showTime());
    Serial.println("Device ID: " + String(deviceID)); //Display device ID
    if (logMode == 'S') {
      Serial.println("Logging mode: S (Data saved to SD card and Flash Mem)");
    }
    if (logMode == 'F') {
      Serial.println("Logging mode: F (Data saved to flash mem only; no SD card logging)");
      SDOK = 0;     //Turns off SD logging
    }
    Serial.print("Feeding mode: "); Serial.println(feedMode);
    Serial.print("Radio node "); Serial.print(nodeID, HEX); Serial.print(" in group "); Serial.print(netGrp, DEC); Serial.print(" on network "); Serial.println(radioNet);
    Serial.print("Target node is "); Serial.println(targetNode);
  
    Serial.println();
    // Ask the user for instruction and display the options
    Serial.println("What to do? (input capital letters)");
    Serial.println("    A, T = Feed modes: All, Target");
    Serial.println("    B = Display flash memory");
    Serial.println("    C = set clock");
    Serial.println("    D = Display logging history");
    Serial.println("    E = Erase (reset) backup memory");
    Serial.println("    I = Set device ID");
    Serial.println("    L = Load info from SD card");
    Serial.println("    M = Change logging mode");
    Serial.println("    W = Write flash data to SD card");
    // Serial.println("    X = Insert elevator");
    // Serial.println("    Y = Eject elevator");
    Serial.println("    Z = Turn off motor (used for testing");
    Serial.println("    anything else = Start logging");

    //Get input from user or wait for timeout after 15 seconds
    char incomingByte = getInputByte(userTimeoutDuration);
    String printThis = String("Value recieved: ") + incomingByte;
    Serial.println(printThis);
    Serial.println(incomingByte, DEC);
    if (incomingByte < 47) {   //Ignore punctuation and line returns and such.
      incomingByte = 'X';
    }
    switch (incomingByte) {                                               // execute whatever option the user selected
      default:
        menu = 0;         //Any non-listed entries - set menu to 0 and break from switch function. Move on to logging data
        break;
      case 'A': case 'T': {
          feedMode = setMode(incomingByte);  //set feeder mode to A: feed all tagged birds; break and show menu again
          //birdFed = 0;
          break;
        }           
      case 'C': {                                                         // option to set clock
          inputTime();                                                    // calls function to get time values from user
          break;                                                        //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'I': {
          inputID(0x00000004);   //  calls function to get a new feeder ID
          writeFlashByte(0x00000003, 0xAA);                           //Write a different byte to this memory location
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'E': {
          //eraseBackup('a');
          eraseBackup('s');
          break;   //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'B': {
          dumpMem(dataLine-1);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'D': {
          displayLog();
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'L': {
          deviceIDstr = String(deviceID);
          String netFile = String(deviceIDstr +  "_NET.TXT");
          transferNetwork(netFile);                                          //Transfer tag data from the SD card to the flash memory
          break;
        }
      case 'M': {
          logMode = readFlashByte(0x0000000D);
          if (logMode != 'S') {
            writeFlashByte(0x0000000D, 0x53);    //hex 53 is "S"
            Serial.println("Logging mode S");
            Serial.println("Data saved immediately SD card and Flash Mem");
            if (SDOK == 0) {
              SDOK = 1;
            }
          } else {
            writeFlashByte(0x0000000D, 0x46);    //hex 46 is "F"
            Serial.println("Logging mode F");
            Serial.println("Data saved to Flash Mem only (no SD card needed)");
            SDOK = 0;
          }
          logMode = readFlashByte(0x0000000D);
          Serial.println("log mode variable: ");
          Serial.println(logMode);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      // case 'X': {
      //     motLoad();
      //     break;       //  break out of this option, menu variable still equals 1 so the menu will display again
      //   }
      // case 'Y': {
      //     motUnload();
      //     break;       //  break out of this option, menu variable still equals 1 so the menu will display again
      //   }
      case 'Z': {
          motOn = 0;
          Serial.println("Motor turned off!! Restart to turn it back on.");
          delay(1500);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }    
//      case 'Q': {
//          motClose();
//          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
//        } 
//      case 'R': {
//          motRecharge();
//          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
//        }    
//      case 'U': {
//          motReturn();
//          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
//        }  
      // case 'K': {
      //     motFeed();
      //     delay(500);
      //     // motClose();
      //     // delay(500);
      //     motRecharge();
      //     delay(1500);
      //     motReturn();
      //     break;       //  break out of this option, menu variable still equals 1 so the menu will display again
      //   }                   
      case 'W': {
          if (SDOK == 1) {
            writeMem(dataFile);
          } else {
            Serial.println("SD card missing");
          }
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
    } //end of switch
  } //end of while(menu = 1){
 
  rtc.updateTime();
  if (SDOK == 1) {
    String logStr = showTime() + " Logging_started";
    SDwriteString(logStr, logFile);
  }
  
  unixTime.unixLong = getUnix();
  Serial.println(unixTime.unixLong, DEC);
  char logInfo[5] = {1, unixTime.b1, unixTime.b2, unixTime.b3, unixTime.b4};
  logFlashAddr = calcFlashAddr(logLine, 105, 5, 8);
  writeFlashArray(logFlashAddr, logInfo, 5);  //write array
  logLine = logLine + 1;

  // if(motOn) {
  //   uint8_t loadUp = 0;
  //   while(loadUp==0) {
  //     loadUp = motRecharge();
  //     motReturn();
  //   }
    
  // } else { 
  //   Serial.println("Motor off--usually feeder loading happens here.");
  //   delay(1000);
  // }
    
  feedState = 1;
  RFcircuit = 1;
  blinkLED(LED_RFID, 3, 100);
}

////////////////////////////////////////////////////////////////////////////////
////////MAIN////////////////MAIN////////////////MAIN////////////////MAIN////////
////////////////////////////////////////////////////////////////////////////////

void loop() { // Main code is here, it loops forever:

  ///////Check Sleep//////////////Check Sleep///////////
  //Check to see if it is time to execute nightime sleep mode.

  rtc.updateTime();                                            // Get an update from the real time clock

  if (Debug) Serial.println(showTime());                       // Show the current time
  int curTimeHHMM = rtc.getHours() * 100 + rtc.getMinutes();   // Combine hours and minutes into one variable
  if (curTimeHHMM == slpTime) {                                // Check to see if it is sleep time
    String SlpStr =  showTime() + " Go_to_sleep";             // if it's time to sleep make a log message
    if (Debug) Serial.println(SlpStr);                        // print log message
    if (SDOK == 1) {
      SDwriteString(SlpStr, logFile); // save log message if SD writes are enabled
    }
    unixTime.unixLong = getUnix();
    Serial.println(unixTime.unixLong, DEC);
    char slpInfo[5] = {2, unixTime.b1, unixTime.b2, unixTime.b3, unixTime.b4};
    logFlashAddr = calcFlashAddr(logLine, 105, 5, 8);
    writeFlashArray(logFlashAddr, slpInfo, 5);  //write array    //write array
    logLine = logLine + 1;
    sleepAlarm();                                             // sleep using clock alarm for wakeup
    rtc.updateTime();                                         // get time from clock
    SlpStr =  "Wake up from sleep mode at " + showTime();     // log message
    if (SDOK == 1) {
      SDwriteString(SlpStr, logFile); // save log message if SD writes are enabled
    }
    unixTime.unixLong = getUnix();
    Serial.println(unixTime.unixLong, DEC);
    char wakeInfo[5] = {3, unixTime.b1, unixTime.b2, unixTime.b3, unixTime.b4};
    logFlashAddr = calcFlashAddr(logLine, 105, 5, 8);
    writeFlashArray(logFlashAddr, wakeInfo, 5);  //write array    //write array
    logLine = logLine + 1;
  }

  //////Read Tags//////////////Read Tags//////////
  //Try to read tags - if a tag is read and it is not a recent repeat, write the data to the SD card and the backup memory.

  uint8_t feedSuccess = 0; // Indicates if bird came to correct feeder
  uint8_t currentGrp = 0;  // Use to convey current feeder group for each bird.
  uint8_t reversals = 0;   // Current number of trial reversals
  
  if (FastRead(RFcircuit, checkTime, pollTime1) == 1) {
    processTag(RFIDtagArray, RFIDstring, RFIDtagUser, &RFIDtagNumber);                  // Parse tag data into string and hexidecimal formats
    rtc.updateTime();                                                                   // Update time from clock - use this for all data logging   
    currRFID = String(RFIDstring) + String(RFcircuit);
    unixTime.unixLong = getUnix();
    if (currRFID != pastRFID  | (unixTime.unixLong - unixPast >= delayTime)) {                   // See if the tag read is a recent repeat -- do away with this?
      
      
      if(feedMode == 'A') {  //feed all birds and log RFID data. No trial logging and no transmission.
        Serial.println("Feed bird mode A");
        if(motOn) {
          motFeed();
        } else {
          Serial.println("Motor turned off");
          feedState = 2;
        }
      }


      if(feedMode == 'T') {  //feed all birds and log RFID data. No trial logging and no transmission.
        uint16_t tagFound = checkTag(0, 10, numTagsStored);  //startLine, lineLength, lastLine, returns lineNumber if tag found, otherwise 0xFFFF, dataline is stored in cArray2
        if(tagFound != 0xFFFF) {    //Check least bits for indication of no match found
          currentGrp = cArray2[5], history.h1=cArray2[6]; history.h2=cArray2[7]; reversals = cArray2[8];

           Serial.print(cArray2[0], HEX); Serial.print(cArray2[1], HEX); Serial.print(cArray2[2], HEX); Serial.print(cArray2[3], HEX); Serial.println(cArray2[4], HEX); 
           Serial.print(cArray2[5], DEC);  Serial.print(" ");  Serial.print(cArray2[6], DEC); Serial.print(" "); Serial.println(cArray2[7], DEC);
          
          
          Serial.print("Bird's current group is "); Serial.println(cArray2[5], DEC);
          Serial.print("Feeder's group is "); Serial.println(netGrp, DEC);

          Serial.print("Old history: "); printBits(history.h16b, 12); Serial.println();

          currentGrp = cArray2[5];
         
          if(cArray2[5] == netGrp) {
            Serial.println("Feed bird mode T");
            if(motOn) {
              motFeed();
            } else {
              Serial.println("Motor turned off");
              feedState = 2;
            }
            
            feedSuccess = 1;
            history.h16b = ((history.h16b << 1) | 0x01) & 0b0000111111111111;      //Update history with success; Shift left one, add 1, clear most significan bits
            uint16_t x = history.h16b;
            uint8_t B_cnt = 0;
            for (; x; x >>= 1) {
              if (x & 1) {B_cnt++;}
            }
            Serial.println("Success bits sum to "); Serial.println(B_cnt);
            
            if(B_cnt >= 10) {
              if(currentGrp==maxGrp) {  //Advance or reset netGrp
                currentGrp = 1;
              } else {
                currentGrp = currentGrp + 1;
              }
              Serial.println("updating current group to "); Serial.println(currentGrp);
              history.h16b = 0;  // Clear hisotry for next trial
              reversals = reversals + 1;
            }
          } else {
            Serial.println("No feed, Wrong group"); 
            feedSuccess = 0;
            history.h16b = (history.h16b << 1) & 0b0000111111111110;      //Update history with failure, shift and clear, low bit=0
          }

          Serial.print("updated history: "); printBits(history.h16b, 12); Serial.println();

          cArray2[0]=currentGrp; cArray2[1]=history.h1; cArray2[2]=history.h2; cArray2[3]=reversals;
          uint32_t wAdr = calcFlashAddr(tagFound, 52, 10, 1); // calcFlashAddr(lineNum, linesPerPage, bytesPerLine, startPage);
          writeFlashArray(wAdr+5, cArray2, 4);  // Write revised group and history to flash. Add 5 to address to avoid overwriting RFID code.
          
          //Generate an array of data to transmit to other feeders.
          transArray[0]=RFIDtagArray[0]; transArray[1]=RFIDtagArray[1]; transArray[2]=RFIDtagArray[2]; transArray[3]=RFIDtagArray[3]; transArray[4]=RFIDtagArray[4];  //RFID tag
          transArray[5]=currentGrp; transArray[6]=history.h1; transArray[7]=history.h2; transArray[8]=reversals;    //Group, history, and reversals (trials)
          transArray[9]=1;   //Indicates transmission type
          transArray[10]=deviceID[0]; transArray[11]=deviceID[1]; transArray[12]=deviceID[2]; transArray[13]=deviceID[3];  //Indicates device of origin
          if (radio.sendWithRetry(targetNode, transArray, 14)) {
            Serial.println("ACK received!");
            //delay to confirm message?
          } else {
            Serial.println("ACK Failed!");
            //Resend??
          }   
        } else {
          Serial.println("no match, no feed");
          currentGrp = 0; history.h16b = 0xFFFE; reversals=0;

           //Generate an array of data to transmit to other feeders.
          transArray[0]=RFIDtagArray[0]; transArray[1]=RFIDtagArray[1]; transArray[2]=RFIDtagArray[2]; transArray[3]=RFIDtagArray[3]; transArray[4]=RFIDtagArray[4];
          transArray[5]=currentGrp; transArray[6]=history.h1; transArray[7]=history.h2; transArray[8]=reversals; transArray[9]=10;  //transmission type 10 indicates non target tag read.
          transArray[10]=deviceID[0]; transArray[11]=deviceID[1]; transArray[12]=deviceID[2]; transArray[13]=deviceID[3];
          if (radio.sendWithRetry(targetNode, transArray, 14)) {
            Serial.println("ACK received!");
            //delay to confirm message?
          } else {
            Serial.println("ACK Failed!");
            //Resend??
          }   
          
        }
      }

      

      if (SDOK == 1) {
        String SDsaveString = String(RFIDstring) + ", " + RFcircuit + ", " + showTime();    // Create a data string for SD write
        SDwriteString(SDsaveString, dataFile); // If it's a new read, save to the SD card
      }

      
      char flashData[16] = {RFIDtagArray[0], RFIDtagArray[1], RFIDtagArray[2],          // Create an array representing an entire line of data to store in flash
                            RFIDtagArray[3], RFIDtagArray[4], unixTime.b1,
                            unixTime.b2, unixTime.b3, unixTime.b4, radioNet, netGrp, nodeID,
                            currentGrp, reversals, history.h1, history.h2};



      
      
                  
      flashAddr = calcFlashAddr(dataLine, 33, 16, 128);     // get the correct flash address based on the current data line

      Serial.print("Old Flash address: page "); Serial.print(flashAddr>>10); Serial.print("  byte "); Serial.println(flashAddr&0x000003FF); 
      
      writeFlashArray(flashAddr, flashData, 16);            // write array
      dataLine = dataLine + 1;                              // advance the data line
      
      flashAddr = calcFlashAddr(dataLine, 33, 16, 128);
      Serial.print("New Flash address: page "); Serial.print(flashAddr>>10); Serial.print("  byte "); Serial.println(flashAddr&0x000003FF); 

      
      pastRFID = currRFID;                                  // Update past RFID to check for repeats
      unixPast = unixTime.unixLong;
      if (Debug) {Serial.print(RFIDstring); Serial.print(" logged to line "); Serial.println(dataLine);}

      //Wait for bird to leave if feeder was opened...Need to do this with the radio on. Can the feeder reload too?
      while(currRFID == pastRFID) {
        
        //radDelay(200, 10);     //radio delay to wait for bird to leave.
        uint32_t delayStop = millis() + 200;
        while (millis() < delayStop) { 
          delay(10);
          radCheck(0);
        }
   
        if (FastRead(RFcircuit, checkTime, pollTime1) == 1) {  //read tags until match fails
           processTag(RFIDtagArray, RFIDstring, RFIDtagUser, &RFIDtagNumber);                  // Parse tag data into string and hexidecimal formats
           currRFID = String(RFIDstring) + String(RFcircuit);
           //Serial.print(currRFID); Serial.print(" "); Serial.println(pastRFID);
        } else { //break if not tag detected.
          break;
        }
      }
      Serial.println("Bird departed, Reload and close feeder if needed");
      Serial.print("feedState is "); Serial.println(feedState);
      // if(feedState == 2){
      //   if(motOn) {
      //     for (uint8_t i=0; i<5; i++) {     //try five times to recharge
      //       uint8_t rechargeOK = motRecharge();
      //       if(rechargeOK == 1) i=5;
      //       motReturn();
      //     }
      //   } else {
      //     Serial.println("Motor turned off - Recharging would happen here");
      //     delay(1000);
      //     feedState = 1;
      //   }  
      // }
    } else {
      if (Debug) Serial.println("Repeat - Data not logged");                            // Message to indicate repeats (no data logged)
    }
    blinkLED(LED_RFID, 2, 5);
  }

  //////////Pause//////////////////Pause//////////
  //After each read attempt execute a pause using either a simple delay or low power sleep mode.

  //if (cycleCount < stopCycleCount) { // Pause between read attempts with delay or a sleep timer

  uint32_t delayStop = millis() + pauseTime;
  while (millis() < delayStop) { 
    delay(10);
    radCheck(0);
  }

//  if (1) { //NEVER SLEEP
//    radDelay(pauseTime, 10);   //uint16_t duration, uint16_t gap
//  } else {
//    sleepTimer(pauseCountDown, pauseRemainder);
//  }

  //Alternate between circuits (comment out to stay on one cicuit).
  //RFcircuit == 1 ? RFcircuit = 2 : RFcircuit = 1; //if-else statement to alternate between RFID circuits

}





/////////////////////////////////////////////////////////////////////////////
/////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////
/////////////////////////////////////////////////////////////////////////////

//repeated LED blinking function
void blinkLED(uint8_t ledPin, uint8_t repeats, uint16_t duration) { //Flash an LED or toggle a pin
  pinMode(ledPin, OUTPUT);             // make pin an output
  for (int i = 0; i < repeats; i++) {  // loop to flash LED x number of times
    digitalWrite(ledPin, LOW);         // turn the LED on (LOW turns it on)
    delay(duration);                   // pause again
    digitalWrite(ledPin, HIGH);        // turn the LED off (HIGH turns it off)
    delay(duration);                   // pause for a while
  }                                    // end loop
}                                      // End function


//Recieve a byte (charaacter) of data from the user- this times out if nothing is entered
char getInputByte(uint32_t timeOut) {                 // Get a single character
  char readChar = '?';                                // Variable for reading in character
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  while (serial.available()) {                        // If there is a response then perform the corresponding operation
    byte R1 = serial.read();                          // read input from the user
    if (R1 > 47) {                                    // Do nothing with character that are not numbers or letter (ignore line returns and such)
      readChar = R1;
    }
  }
  return readChar;                                    //Return the value. It will be "?" if nothing is recieved.
}

//Recieve a character array of data from the user- this times out if nothing is entered
byte getInputString(uint32_t timeOut) {               // Get a character array from the user and put in global array variale. Return the number of bytes read in
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  byte charCnt = 0;
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  if (serial.available()) {                           // If there is a response then read in the data
    delay(40);                                        // long delay to let all the data sink into the buffer
    while (serial.available()) {
      byte R1 = serial.read();                          // read the entry from the user
      if (R1 > 47) {
        Serial.println(R1, DEC);
        cArray1[charCnt] = R1;                          // ignore punctuation, line returns, etc
        charCnt++;
      }
    }
  }
  //  Serial.println("char count is ");
  //  Serial.println(charCnt);
  //  Serial.println("array is ");
  //  for (int i = 0; i <= charCnt; i++) {
  //    Serial.print(cArray1[i]);
  //  }
  //  Serial.println();
  return charCnt;                                     //Return the number of characters recieved. It will be zero if nothing is read in
}


//CLOCK FUNCTIONS///////////

//Get date and time strings and cobine into one string
String showTime() { //Get date and time strings and cobine into one string
  currentDateTime = String(rtc.stringDateUSA()) + " " + String(rtc.stringTime());
  return currentDateTime;
}

//Function to get user data for setting the clock
void inputTime() {                               // Function to set the clock
  Serial.println("Enter mmddyyhhmmss");          // Ask for user input
  byte timeInput = getInputString(20000);        // Get a string of data and supply time out value
  if (timeInput == 12) {                  // If the input string is the right length, then process it
    Serial.println("time read in");                   // Show the string as entered
    byte mo = (cArray1[0] - 48) * 10 + (cArray1[1] - 48); //Convert two ascii characters into a single decimal number
    byte da = (cArray1[2] - 48) * 10 + (cArray1[3] - 48); //Convert two ascii characters into a single decimal number
    byte yr = (cArray1[4] - 48) * 10 + (cArray1[5] - 48); //Convert two ascii characters into a single decimal number
    byte hh = (cArray1[6] - 48) * 10 + (cArray1[7] - 48); //Convert two ascii characters into a single decimal number
    byte mm = (cArray1[8] - 48) * 10 + (cArray1[9] - 48); //Convert two ascii characters into a single decimal number
    byte ss = (cArray1[10] - 48) * 10 + (cArray1[11] - 48); //Convert two ascii characters into a single decimal number
    if (rtc.setTime(ss, mm, hh, da, mo, yr + 2000, 1) == false) {     // attempt to set clock with input values
      Serial.println("Something went wrong setting the time");        // error message
    }
  } else {
    Serial.println("Time entry error");           // error message if string is the wrong lenth
  }
}

// sleep and wake up using the alarm  on the real time clock
void sleepAlarm() {                           // sleep and wake up using the alarm  on the real time clock
  rtc.setAlarm(0, wakM, wakH, 1, 1, 1, 19);  // set alarm: sec, min, hr, date, mo, wkday, yr (only min and hr matter)
  rtc.writeRegister(0x02, 0);                // write a zero to the flags register to clear all flags.
  rtc.enableDisableAlarm(B00000111);         // Enable daily alarm - responds to hour, minute, and second (set last three enable bits)
  rtc.enableAlarmINT(1);                     // Enable the clock interrupt output.
  lpSleep();                                 // sleep funciton - sleep until alarm interrupt
  blinkLED(LED_RFID, 5, 200);                // blink to indicate wakeup
  rtc.writeRegister(0x02, 0);                // clear clock flags to turn off alarm.
}

// Sleep and wake up using a 32-hertz timer on the real time clock
void sleepTimer(uint16_t pCount, byte pRemainder) { // Sleep and wake up using a 32-hertz timer on the real time clock
  rtc.writeRegister(0x02, 0);       // write a zero to the flags register to clear all flags.
  rtc.setTimer(pCount);             // set timer countdown (32-hertz timer)
  rtc.enableTimerINT(1);            // enable the clock interrupt output
  rtc.setCTRL1Register(B10011011);  // set control register to enable a 32 Hertz timer.
  lpSleep();                        // call sleep funciton (you lose USB communicaiton here)
  //blinkLED(LED_RFID, 1, 30);     // blink indicator - processor reawakened
  delay(pRemainder);                // additional delay for accuracy
  rtc.enableTimerINT(0);            // disable the clock interrupt output
  rtc.writeRegister(0x02, 0);       // write a zero to clear all interrupt flags.
}

uint32_t getUnix() {
  int8_t my = (rtc.getMonth() >= 3) ? 1 : 0;
  uint16_t y = (rtc.getYear() + 2000) + my - 1970;
  uint16_t dm = 0;
  for (int i = 0; i < rtc.getMonth() - 1; i++) dm += (i < 7) ? ((i == 1) ? 28 : ((i & 1) ? 30 : 31)) : ((i & 1) ? 31 : 30);
  return (((rtc.getDate() - 1 + dm + ((y + 1) >> 2) - ((y + 69) / 100) + ((y + 369) / 100 / 4) + 365 * (y - my)) * 24ul + rtc.getHours()) * 60ul + rtc.getMinutes()) * 60ul + rtc.getSeconds();
}

void convertUnix(uint32_t t) { //Stores time values to timeIn[12]
  t += 0 * 3600ul;
  timeIn[5] = t % 60ul;   //Seconds stored in timeIn[5]
  t /= 60ul;
  timeIn[4] = t % 60ul;  //Minutes stored to timeIn[4]
  t /= 60ul;
  timeIn[3] = t % 24ul;  //Hours stored to timeIn[3]
  t /= 24ul;
  byte dayOfWeek = (t + 4) % 7;
  if (!dayOfWeek) dayOfWeek = 7;
  uint32_t z = t + 719468;
  uint8_t era = z / 146097ul;
  uint16_t doe = z - era * 146097ul;
  uint16_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  uint16_t y = yoe + era * 400;
  uint16_t doy = doe - (yoe * 365 + yoe / 4 - yoe / 100);
  uint16_t mp = (doy * 5 + 2) / 153;
  timeIn[1] = doy - (mp * 153 + 2) / 5 + 1;  //Day stored to timeIn[1]
  uint8_t month = mp + (mp < 10 ? 3 : -9);
  timeIn[0] = month;  //Month stored to timeIn[0]
  y += (month <= 2);
  timeIn[2] = y; //Year stored to timeIn[2]
}




////FLASH MEMORY FUNCTIONS////////////////////

//Enable the flash chip
void flashOn(void) {              // Enable the flash chip
  pinMode(FlashCS, OUTPUT);       // Chip select pin for Flash memory set to output
  digitalWrite(SDselect, HIGH);   // make sure the SD card select is off
  digitalWrite(FlashCS, LOW);     // activate Flash memory
  SPI.begin();                    // Enable SPI communication for Flash Memory
  SPI.setClockDivider(SPI_CLOCK_DIV16); // slow down the SPI for noise and signal quality reasons.
}

//Disable the flash chip
void flashOff(void) {           // Disable the flash chip
  //SPI.end();                    // turn off SPI
  digitalWrite(FlashCS, HIGH);  // deactivate Flash memory
}

// Read a single byte from flash memory
uint8_t readFlashByte(uint32_t bAddr) {   // read a single byte from flash memory
  flashOn();                                // activate flash chip
  SPI.transfer(0x03);                       // opcode for low freq read
  SPI.transfer((bAddr >> 16) & 0xFF);       // first of three address bytes
  SPI.transfer((bAddr >> 8) & 0xFF);        // second address byte
  SPI.transfer(bAddr & 0xFF);               // third address byte
  byte fByte = SPI.transfer(0);             // finally, read the byte
  flashOff();                               // deactivate flash chip
  return fByte;                             // return the byte that was read
}

void writeFlashByte(uint32_t wAddr, byte wByte) { // write a single byte from flash memory
  flashOn();                                      // activate flash chip
  SPI.transfer(0x58);                             // opcode for read modify write
  SPI.transfer((wAddr >> 16) & 0xFF);             // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);              // second address byte
  SPI.transfer(wAddr & 0xFF);                     // third address byte
  SPI.transfer(wByte);                            // finally, write the byte
  flashOff();                                     // deactivate flash chip
  delay(20);                                      // delay to allow processing
}

//// Get the address counter for the flash memory
//unsigned long getFlashAddr() {      // get the address counter for the flash memory from page 1 address
//  char gAddr[3] = {0, 0, 0};        // create a character array for holding data
//  readFlashArray(0x400, gAddr, 3);  // read from flash
//  unsigned long fAddr1 = (gAddr[0]<<16) + (gAddr[1]<<8) + gAddr[2]; // Combine three bytes into a long
//  return fAddr1;                    // return the address.
//}

//Write a data array to a specified memory address on the flash chip
void writeFlashArray(uint32_t wAddr, char *cArr, byte nchar) {
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  //Serial.println("writing to flash");
  flashOn();                           // activate flash chip
  SPI.transfer(0x58);                  // opcode for read modify write
  SPI.transfer((wAddr >> 16) & 0xFF);  // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);   // second address byte
  SPI.transfer(wAddr & 0xFF);          // third address byte
  for (int n = 0; n < nchar; n++) {    // loop throught the bytes
    //Serial.print(cArr[n], HEX);
    SPI.transfer(cArr[n]);             // finally, write the byte
  }
  Serial.println();
  SPI.endTransaction();
  flashOff();                          // Shut down
  delay(20);                           // This delay allows writing to happen - maybe it can be removed if the chip is not read immediately after
  //Serial.println("done writing to flash");
}

void readFlashArray(unsigned long addr, char *carr, byte nchar) {  // read a single byte from flash memory
  flashOn();                              // activate flash chip
  SPI.transfer(0x03);                     // opcode for low freq read
  SPI.transfer((addr >> 16) & 0xFF);      // first of three address bytes
  SPI.transfer((addr >> 8) & 0xFF);       // second address byte
  SPI.transfer(addr & 0xFF);              // third address byte
  for (int n = 0; n < nchar; n++) {       // loop throught the bytes
    carr[n] = SPI.transfer(0);          // finally, read the byte
  }
  flashOff();                             // deactivate flash chip
}

void displayRFIDLine(uint16_t lineNo) {
  char a1[10];
  uint32_t adr = calcFlashAddr(lineNo, 52, 10, 1);
  readFlashArray(adr, a1, 10);
  for(byte i=0; i<5; i++){
    Serial.print(a1[i], HEX);
  }
}

uint32_t advanceFlashAddr(uint32_t Addr, byte stepsize) {
  uint16_t bAddress = (Addr & 0x03FF) + stepsize;      //and with 00000011 11111111 and add 12 for new byte address
  if (bAddress > 500) {                           //stop writing if beyond byte address 500 (this is kind of wasteful)
    Addr = (Addr & 0xFFFFC00) + 0x0400;      //set byte address to zero and add 1 to the page address
    //Serial.println("Page boundary!!!!!!!!!");
  } else {
    Addr = Addr + stepsize;                        //just add to the byte address
  }
  return Addr;
}


char setMode(char sMode) {
  writeFlashByte(14, sMode);
  serial.print(F("Feeder mode set to: "));
  serial.println(sMode);
  delay(20);
  return sMode;
}

// Input the device ID and write it to flash memory
void inputID(uint32_t writeAddr) {                                         // Function to input and set feeder ID
  Serial.println("Enter four alphanumeric characters");  // Ask for user input
  byte IDin = getInputString(10000);                  // Get input from user (specify timeout)
  if (IDin == 4) {                             // if the string is the right length...
    deviceID[0] = cArray1[0];                      // Parse the bytes in the string into a char array
    deviceID[1] = cArray1[1];
    deviceID[2] = cArray1[2];
    deviceID[3] = cArray1[3];
    writeFlashArray(writeAddr, deviceID, 4);                 // Write the array to flash
    Serial.println(deviceID);
  } else {
    Serial.println("Invalid ID entered");                // error message if the string is the wrong lenth
  }
}

//Display backup memory
void dumpMem(uint16_t endLine) {    // Display backup memory up until endLine
  if(dataLine == 0) {                             // First display RFID data, if any exists.
    Serial.println("Nothing in RFID memory.");
  } else {
    //uint32_t curAddr = 0x00020000;                 // first address for stored data
    //uint32_t fAddressEnd = calcFlashAddr(dataLine-1, 33, 16, 128); // last address for stored data
    //if (doAll) fAddressEnd = ((8191 << 10) + 512);   // If we are dumping EVERYTHING, calculate the last possible flash address
    //unsigned long fAddressEnd = 0xF000;
                                       // byte array for holding data
    Serial.print("Displaying RFID data lines 0 to "); Serial.println(endLine);    // Message

    char BA[32];  // Array for building a data line
    
    for(uint8_t i=0; i <= endLine; i++) {      
               
      if (serial.available()) {                     //  // Escape long memory dump by entering a character...If a charcter is entered....
        Serial.println("User exit");                // ...print a message...
        break;                                      // ...and exit the loop
      }
      
      //Serial.print("Line "); Serial.print(i); Serial.print(" ");
      
      uint32_t curAddr = calcFlashAddr(i, 33, 16, 128);
      readFlashArray(curAddr, BA, 16);              // read a line of data (16 bytes)
      unixTime.b1 = BA[5]; unixTime.b2 = BA[6]; unixTime.b3 = BA[7]; unixTime.b4 = BA[8];

      //Serial.print(BA[9], DEC); Serial.print(" ");  Serial.print(BA[10], DEC); Serial.print(" ");  Serial.print(BA[11], DEC); Serial.print(" ");
     
      convertUnix(unixTime.unixLong);  // covert unix time. Time values get stored in array timeIn, bytes 0 through 5.
     
      history.h1 = BA[14]; history.h2=BA[15]; // parse history bytes.

      uint8_t numBits = 12;
      for (uint8_t j = 0; j < numBits; j++) { 
        BA[j+16] = (history.h16b & (1 << (numBits - 1 - j))) > 0 ? 1 : 0;
        } // Extract bits from integer variable

      static char text[SDLineLength];                       // make an array for writing data lines
      history.h1=BA[14]; history.h2=BA[15];
      sprintf(text, "%02X%02X%02X%02X%02X,%02d/%02d/%04d %02d:%02d:%02d,%c%X%02X,%d,%03d,%d%d%d%d%d%d%d%d%d%d%d%d",
              BA[0], BA[1], BA[2], BA[3], BA[4],                                   //RFID codes
              timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5],    //Date Stamp
              BA[9], BA[10], BA[11], BA[12], BA[13],                              //Feeder ID (3), current group (1), Trial no (1)                                                  
              BA[16], BA[17], BA[18], BA[19], BA[20], BA[21], BA[22], BA[23], BA[24], BA[25], BA[26], BA[27]);  //History
      Serial.println(text); 
    }
    
  }
  //output the trial history data
  if(numTagsStored == 0xFFFF | numTagsStored == 0) {
    Serial.println("No tags or trial data memory.");
  } else {
    Serial.println("Trial Data ");
    for(uint8_t i=0; i < numTagsStored; i++) {
    uint32_t tAddr = calcFlashAddr(i, 52, 10, 1); // last address for stored data
    char BA[10]; 
    readFlashArray(tAddr, BA, 10);
    //BA[5] = asciiToHex(BA[5]);   
    static char text[15];                       // make an array for writing data lines
      sprintf(text, "%02X%02X%02X%02X%02X, %d, ", BA[0], BA[1], BA[2], BA[3], BA[4], BA[5]);
      history.h1 = BA[6]; history.h2 = BA[7];
      Serial.print(i); Serial.print(" "); Serial.print(text); printBits(history.h16b, 12); Serial.print(", "); Serial.println(BA[8], DEC);
    }
  Serial.println();
  }

  
  
}


void printBits(uint16_t n, uint8_t numBits) {
  //byte numBits = 16;  // 2^numBits must be big enough to include the number n
  char b;
  char c = ' ';   // delimiter character
  for (byte i = 0; i < numBits; i++) {
    // shift 1 and mask to identify each bit value
    b = (n & (1 << (numBits - 1 - i))) > 0 ? '1' : '0'; // slightly faster to print chars than ints (saves conversion)
    Serial.print(b);
    //if (i < (numBits - 1) && ((numBits-i - 1) % 4 == 0 )) Serial.print(c); // print a separator at every 4 bits
  }
}


String readFlashLine(uint32_t flashLine) {
  //10 bytes per line, 51 lines per page
  uint16_t pageAddr = 128 + flashLine / 51;
  //Serial.println(pageAddr);
  uint16_t byteAddr = (flashLine % 51 - 1) * 10;
  //Serial.println(byteAddr);
  uint32_t curAddr = (pageAddr << 10) + byteAddr;
  Serial.println(curAddr, HEX);
  char BA[10];
  readFlashArray(curAddr, BA, 10);              // read a line of data (10 bytes)
  unixTime.b1 = BA[6]; unixTime.b2 = BA[7]; unixTime.b3 = BA[8]; unixTime.b4 = BA[9];
  convertUnix(unixTime.unixLong);  // covert unix time. Time values get stored in array timeIn, bytes 0 through 5.
  static char text[34];                       // make an array for writing data lines
  sprintf(text, "%02X%02X%02X%02X%02X, %d, %02d/%02d/%04d %02d:%02d:%02d",
          BA[0], BA[1], BA[2], BA[3], BA[4], BA[5], timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
  return text;
}

void readFlashLine2(uint32_t lineNum, uint16_t startPage, uint8_t bytesPerLine, uint8_t printLine) { //provide which line to read, the start page, length of each line, and whether to print line or not. Line reads into cArray2
  uint16_t linesPerPage = 528 / bytesPerLine;
  uint32_t fAddr = calcFlashAddr(lineNum, linesPerPage, bytesPerLine, startPage);
  char BA[10];
  readFlashArray(fAddr, BA, 10);              // read a line of data
  unixTime.b1 = BA[5]; unixTime.b2 = BA[6]; unixTime.b3 = BA[7]; unixTime.b4 = BA[8];
  convertUnix(unixTime.unixLong);  // covert unix time. Time values get stored in array timeIn, bytes 0 through 5.
  sprintf(cArray2, "%02X%02X%02X%02X%02X,%02d/%02d/%04d %02d:%02d:%02d",
          BA[0], BA[1], BA[2], BA[3], BA[4], timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
  if (printLine) {
    Serial.print("Flash Line number "); Serial.print(lineNum); Serial.print(": ");
    for (uint8_t i = 0; i < 34; i++) { //print line
      Serial.print(cArray2[i]);
    }
    Serial.println();
  }
}


//Display backup memory
void displayLog() {                     // Display logging info
  unsigned long curAddr = 0x00002000;             // first address for stored data
  unsigned long fAddressEnd = logFlashAddr;       // last address for stored data
  char BA[5];                                    // byte array for holding data
  static char text[34];                       // make an array for writing data lines
  Serial.println("Displaying log info.");    // Message
  while (curAddr < fAddressEnd) {                 // Escape long memory dump by entering a character
    //    if (serial.available()) {                     // If a charcter is entered....
    //      Serial.println("User exit");                // ...print a message...
    //      break;                                      // ...and exit the loop
    //    }
    readFlashArray(curAddr, BA, 5);              // read a line of data (5 bytes)
    unixTime.b1 = BA[1];
    unixTime.b2 = BA[2];
    unixTime.b3 = BA[3];
    unixTime.b4 = BA[4];
    convertUnix(unixTime.unixLong);  // covert unix time. Time values get stored in array timeIn, bytes 0 through 5.
    //convert the array to a string
    sprintf(text, "%02d/%02d/%04d %02d:%02d:%02d", timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
    getLogMessage(BA[0]);
    Serial.print(text);
    Serial.print(" ");
    Serial.println(logMessage);                    // print the string
    curAddr = curAddr + 5;                         // add 5 to the byte address
    if ((curAddr & 0x000003FF) > 523) {            // Check to see if we are near the end of a memory page
      curAddr = (curAddr & 0xFFFFC00) + 0x0400;    // if so advance a page set byte address to zero and add 1 to the page address
    }
  }
}

void getLogMessage(uint8_t x1) {
  //Serial.print(x1);
  if (x1 == 1) {
    logMessage = "Logging_started";
  }
  if (x1 == 2) {
    logMessage = "Go_to_sleep";
  }
  if (x1 == 3) {
    logMessage = "Wake_from_sleep";
  }
}



void eraseBackup(char eMode) {  //erase chip and replace stored info
  //first erase pages 2 through 7
  Serial.print("Erasing flash with mode ");
  Serial.println(eMode);
  uint16_t pageAddr = flashAddr >> 10;
  if (eMode == 'a') {
    Serial.println("This will take 80 seconds");
    flashOn();
    SPI.transfer(0xC7);  // opcode for chip erase: C7h, 94h, 80h, and 9Ah
    SPI.transfer(0x94);
    SPI.transfer(0x80);
    SPI.transfer(0x9A);
    digitalWrite(FlashCS, HIGH);    //Deassert cs for process to start
    delay(80000);
    flashOff();                         // Turn off SPI
    Serial.println("DONE!");
    Serial.println("You must now reestablish all parameters");
    return;
  }
  if (eMode == 's') {
    Serial.println("Seek and destroy!!");
    //just erase all pages for log data and any pages with data for rfid data storage, start on page 8
    uint8_t Bt1 = 0;
    uint32_t P1 = 128;
    while(Bt1 != 255) {
      Bt1=readFlashByte(P1<<10);                              //find last page with data on it
      Serial.print("Checking page "); Serial.print(P1); Serial.print(". First byte is "); Serial.println(Bt1, DEC);
      P1++;
    }
    for(uint16_t i=8; i<=P1; i++) {
      Serial.print("Erasing page "); Serial.println(i);
      uint32_t addr = (i << 10);         //Define page address
      flashOn();
      SPI.transfer(0x81);                    // opcode for page erase: 0x81
      SPI.transfer((addr >> 16) & 0xFF);    // first of three address bytes
      SPI.transfer((addr >> 8) & 0xFF);     // second address byte
      SPI.transfer(addr & 0xFF);            // third address byte
      digitalWrite(FlashCS, HIGH);           // Deassert cs for process to start
      delay(35);                             // delay for page erase
    }
    flashOff();
  }
  if (eMode == 'f') {
    Serial.println("Fast erase.");
    pageAddr = (flashAddr >> 10) & 0x1FFF;

    Serial.print("erasing through page ");
    Serial.println(pageAddr, DEC);
  
    for (uint16_t p = 2; p <= pageAddr; p++) {
      Serial.print("Erasing page ");
      Serial.println(p);
      uint32_t ePage = p << 10;
      flashOn();
      SPI.transfer(0x81);                    // opcode for page erase: 0x81
      SPI.transfer((ePage >> 16) & 0xFF);    // first of three address bytes
      SPI.transfer((ePage >> 8) & 0xFF);     // second address byte
      SPI.transfer(ePage & 0xFF);            // third address byte
      digitalWrite(FlashCS, HIGH);           // Deassert cs for process to start
      delay(35);                             // delay for page erase
      flashOff();
    }
  }
  flashAddr = 0x00020000;
  logFlashAddr = 0x00002000;
  dataLine = 0;                   // tracks current data line for writing to Flash memory - starts at 0
  logLine = 0;

}

void writeMem(String fName) {
  //write backup data to SD card
  Serial.print("Writing flash to SD card to ");   // Message to user
  Serial.println(fName);                          // Tell user what the file name is
  uint8_t interval = 10;                          // how many bytes in each data line
  uint16_t pAddr = 128;                           // first page address
  uint16_t bAddr = 0;                             // first byte address (always zero)
  byte BA[528];                                   // define byte array to store a full page
  while (pAddr <= flashAddr >> 10) {              // read in full pages one at a time until last page is reached.
    digitalWrite(LED_RFID, LOW);                 // Flash LED to indicate progress
    Serial.print(" transfering from page ");          // progress message
    Serial.println(pAddr, DEC);
    uint32_t fAddress = pAddr << 10;             // Make the page address a full address with byte address as zero.
    //     Serial.print("current flash Address ");      // progress message
    //     Serial.println(fAddress, DEC);
    //     Serial.print("last flash Address page ");         // progress message
    //     Serial.println(flashAddr>>10, DEC);
    flashOn();
    SPI.transfer(0x03);                         // opcode for low freq read
    SPI.transfer((fAddress >> 16) & 0xFF);      // write most significant byte of Flash address
    SPI.transfer((fAddress >> 8) & 0xFF);       // second address byte
    SPI.transfer(fAddress & 0xFF);              // third address byte

    SPI.transfer(BA, 528);                  // reading in full page.

    digitalWrite(LED_RFID, HIGH);
    flashOff();
    uint16_t lastBAddr = 527;                 //For all but last page we get all the bytes on a page.
    if (pAddr == flashAddr >> 10) {                      //If on the last page, only write the real data (not empty bytes).
      Serial.println("***reached last page***");
      lastBAddr = flashAddr & 0x000003FF;
    }
    //Now write to SD card
    SDstart();
    File dataFile = SD.open(fName, FILE_WRITE);        //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
    if (dataFile) {
      for (int i = 0; i < lastBAddr; i = i + interval) {         // loop to read in an RFID code from byte array and send it out
        
        
        
        
        unixTime.b1 = BA[i + 6]; unixTime.b2 = BA[i + 7]; unixTime.b3 = BA[i + 8]; unixTime.b4 = BA[i + 9]; //move time data into unixTime structure.
        //Convert unix time
        convertUnix(unixTime.unixLong);
        static char text[10];
        sprintf(text, "%02X%02X%02X%02X%02X, %d, %02d/%02d/%04d %02d:%02d:%02d",
                BA[i], BA[i + 1], BA[i + 2], BA[i + 3], BA[i + 4], BA[i + 5], timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
        //
        //             Serial.print("writing from page ");      // progress message
        //             Serial.println(pAddr);         // progress message
        //             Serial.println(text);
        //             Serial.println("Byte counter ");         // progress message
        //             Serial.println(i, DEC);
        //             Serial.print("last byte address ");      // progress message
        //             Serial.println(lastBAddr);         // progress message
        //
        dataFile.println(text);
        //Serial.println(text);
      }
      dataFile.close();      //close the file
    }
    SDstop();
    pAddr++;
  }
  blinkLED(LED_RFID, 6, 70);
}

void writeMemRFID(uint32_t startLine, uint16_t startPage, String fName) {
  //write backup data to SD card
  Serial.print("Writing flash to SD card ");   // Message to user
  Serial.print(fName);                          // Tell user what the file name is
  Serial.print(".  Lines ");                          // Message to user
  Serial.print(startLine);
  Serial.print(" to ");
  Serial.println(dataLine - 1);
  uint8_t interval = 16;                          // how many bytes in each data line on flash
  uint32_t fAddress = calcFlashAddr(startLine, 33, 16, startPage);  //calculate the flash address to begin reading
  uint16_t pAddressStart = fAddress >> 10;
  uint16_t bAddressStart = fAddress & 0x000003FF;
  //  Serial.print("begin writing addr ");            // Message to user
  //  Serial.print(fAddress);
  //  Serial.print("  page: ");
  //  Serial.print(pAddressStart);
  //  Serial.print("  byte: ");
  //  Serial.println(bAddressStart);

  uint32_t fAddressStop = calcFlashAddr(dataLine - 1, 33, 16, startPage); //calculate the flash address to stop reading
  uint16_t pAddressStop = fAddressStop >> 10;
  uint16_t bAddressStop = fAddressStop & 0x000003FF;
  //  Serial.print("stop writing addr ");            // Message to user
  //  Serial.print(fAddressStop);
  //  Serial.print("  page: ");
  //  Serial.print(pAddressStop);
  //  Serial.print("  byte: ");
  //  Serial.println(bAddressStop);

  uint16_t pAddressCurrent = pAddressStart;

  uint8_t BA[528];                                   // define byte array to store a full page
  while (pAddressCurrent <= pAddressStop) {                // read in full pages one at a time until last page is reached.
    digitalWrite(LED_RFID, LOW);                 // Flash LED to indicate progress
    Serial.print(" transfering page ");          // progress message
    Serial.println(pAddressCurrent, DEC);
    uint32_t fAddress = pAddressCurrent << 10;             // Make the page address a full address with byte address as zero.
    flashOn();
    SPI.transfer(0x03);                         // opcode for low freq read
    SPI.transfer((fAddress >> 16) & 0xFF);      // write most significant byte of Flash address
    SPI.transfer((fAddress >> 8) & 0xFF);       // second address byte
    SPI.transfer(fAddress & 0xFF);              // third address byte

    SPI.transfer(BA, 528);                      // read in full page

    digitalWrite(LED_RFID, HIGH);
    flashOff();
    uint16_t lastBAddr = 512;                   //For all but last page we write all the bytes on a page- last address to write to is 512.
    uint16_t firstBAddr = 0;                    //For all but first page we write all the bytes on a page.

    if (pAddressCurrent == pAddressStart) {                     //If on the first page, only start writing at byte bAddress.
      firstBAddr = bAddressStart;
    }

    if (pAddressCurrent == pAddressStop) {                      //If on the last page, only write the real data (not empty bytes).
      Serial.println("***reached last page***");
      lastBAddr = bAddressStop;
    }
    
    //Serial.print("Last Byte Addr "); Serial.println(lastBAddr);
    
    SDstart();
    File dataFile = SD.open(fName, FILE_WRITE);        //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
    if (dataFile) {
      for (int i = firstBAddr; i <= lastBAddr; i = i + interval) {         // loop to read in an RFID code from byte array and send it out
          
          //Serial.print("start Byte Addr "); Serial.println(i);
          //Serial.print("stop Byte Addr "); Serial.println(lastBAddr);
//        
//        for(int bb=0; bb<529; bb++) {
//            if(bb % 16 == 0) {Serial.println();}
//            Serial.print(BA[bb], HEX); Serial.print(" ");
//        }
//        
//        Serial.println(); Serial.print("byte Address "); Serial.println(i, DEC);
//        Serial.print("RFID "); Serial.print(BA[i+1],HEX); Serial.print(BA[i+2],HEX); Serial.print(BA[i+3],HEX); Serial.print(BA[i+4],HEX); Serial.println(BA[i+5],HEX);

        
        unixTime.b1 = BA[i + 5]; unixTime.b2 = BA[i + 6]; unixTime.b3 = BA[i + 7]; unixTime.b4 = BA[i + 8]; //move time data into unixTime structure.
        convertUnix(unixTime.unixLong); //Convert unix time
        
//        //check it
//        Serial.println(unixTime.unixLong);
//        Serial.print(timeIn[0],DEC); Serial.print(timeIn[1],DEC);Serial.print(timeIn[2],DEC);Serial.print(timeIn[3],DEC);Serial.print(timeIn[4],DEC);Serial.println(timeIn[5],DEC); 

        history.h1 = BA[i+14]; history.h2=BA[i+15];

//        //check it
//        Serial.println(history.h16b, BIN); 
//        
        uint8_t numBits = 12;
        uint8_t hist[16];
        for (uint8_t j = 0; j < numBits; j++) { 
          hist[j] = (history.h16b & (1 << (numBits - 1 - j))) > 0 ? 1 : 0;
//          Serial.print(BA[i+j+16]); Serial.print(" ");
        } // Extract bits from integer variable
//        Serial.println(" ");
        
        static char text[52];
        sprintf(text, "%02X%02X%02X%02X%02X,%02d/%02d/%04d %02d:%02d:%02d,%c%X%02X,%d,%03d,%d%d%d%d%d%d%d%d%d%d%d%d",
              BA[0+i], BA[1+i], BA[2+i], BA[3+i], BA[4+i],                                   //RFID codes
              timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5],    //Date Stamp
              BA[9+i], BA[10+i], BA[11+i], BA[12+i], BA[13+i],                              //Feeder ID (3), current group (1), Trial no (1)                                                  
              hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], hist[6], hist[7], hist[8], hist[9], hist[10], hist[11]);  //History
        //Serial.println(text);
        dataFile.println(text);
      } // end for loop
      dataFile.close();
    } // if (dataFile)
    SDstop();
    pAddressCurrent++;
  }
  blinkLED(LED_RFID, 6, 70);
}

void writeMemLog(uint16_t startPage, String fName) { //Overwrite entire log file
  Serial.print("Writing all log data to ");   // Message to user
  Serial.println(fName);                          // Tell user what the file name is
  int32_t fAddress = startPage << 10;  //calculate the flash address to begin writing
  uint16_t pAddressStart = startPage;
  uint16_t bAddressStart = fAddress & 0x000003FF;
  //  Serial.print("begin writing addr ");            // Message to user
  //  Serial.print(fAddress);
  //  Serial.print("  page: ");
  //  Serial.print(pAddressStart);
  //  Serial.print("  byte: ");
  //  Serial.println(bAddressStart);

  uint32_t fAddressStop = calcFlashAddr(logLine - 1, 105, 5, startPage); //calculate the flash address to stop reading
  uint16_t pAddressStop = fAddressStop >> 10;
  uint16_t bAddressStop = fAddressStop & 0x000003FF;
  //  Serial.print("stop writing addr ");            // Message to user
  //  Serial.print(fAddressStop);
  //  Serial.print("  page: ");
  //  Serial.print(pAddressStop);
  //  Serial.print("  byte: ");
  //  Serial.println(bAddressStop);

  uint16_t pAddressCurrent = pAddressStart;

  byte BA[528];                                   // define byte array to store a full page
  while (pAddressCurrent <= pAddressStop) {                // read in full pages one at a time until last page is reached.
    digitalWrite(LED_RFID, LOW);                 // Flash LED to indicate progress
    Serial.print(" transfering log page ");          // progress message
    Serial.println(pAddressCurrent, DEC);
    uint32_t fAddress = pAddressCurrent << 10;             // Make the page address a full address with byte address as zero.
    flashOn();
    SPI.transfer(0x03);                         // opcode for low freq read
    SPI.transfer((fAddress >> 16) & 0xFF);      // write most significant byte of Flash address
    SPI.transfer((fAddress >> 8) & 0xFF);       // second address byte
    SPI.transfer(fAddress & 0xFF);              // third address byte

    SPI.transfer(BA, 528);                  // might work for reading in full page?? needs test.  it works!!

    digitalWrite(LED_RFID, HIGH);
    flashOff();
    uint16_t byteCount = 0;

    SDstart();
    if (SD.exists(fName) & pAddressCurrent == startPage) { //Erase old file if present
      Serial.println("Replacing old log file");
      SD.remove(fName);
    }
    File dataFile = SD.open(fName, FILE_WRITE);        //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
    while (byteCount < 526 ) {
      if (!dataFile) {
        Serial.println("SD card error");
        break;
      }
      if (BA[byteCount] == 255) {
        //Serial.println("end of file");
        break;
      }
      getLogMessage(BA[byteCount]);
      unixTime.b1 = BA[byteCount + 1]; unixTime.b2 = BA[byteCount + 2]; unixTime.b3 = BA[byteCount + 3]; unixTime.b4 = BA[byteCount + 4]; //move time data into unixTime structure.
      convertUnix(unixTime.unixLong); //Convert unix time
      static char text[19];
      sprintf(text, "%02d/%02d/%04d %02d:%02d:%02d", timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
      String logString = String(text) + " " + String(logMessage);
      // Serial.println(logString);
      dataFile.println(logString);
      byteCount = byteCount + 5;
    } // end while loop
    dataFile.close();
    SDstop();
    pAddressCurrent++;
  }
  blinkLED(LED_RFID, 6, 70);
}


void readSDLine(uint32_t lineNum, uint8_t bytesPerLine, String dFile, uint8_t printLine) {   //provide which line to read, length of each line, and file to access. Line is stored in cArray1
  SDstart();
  File readFile = SD.open(dFile, FILE_READ);
  if (readFile) {
    //Serial.print("Seeking line ");
    //Serial.println(lineNum);

    if (!readFile.seek(lineNum * bytesPerLine)) { //two added for carriage return and new line characters.
      Serial.println("Seeking line failed");
    }
    readFile.read(cArray1, 30); //Read 30 bytes (RFID and date)
    if (printLine) {
      Serial.print("SD Line number: "); Serial.print(lineNum); Serial.print(": ");
      for (uint8_t i = 0; i < 30; i++) { //print line
        Serial.print(cArray1[i]);
      }
    Serial.println();
  }

  } else {
    Serial.println("Reading failed");
  }
  SDstop();
}

uint32_t getLine(uint16_t linesPerPage, uint16_t maxByte, uint8_t pStart, uint8_t pEnd, uint8_t interval) { //Seek out the last bit of stored data.
  //uint16_t limit = (527 - interval) + 1;                  // define highest byte address that can be written to on each page.
  //uint16_t lastByteAddr = (limit/interval) * interval;    // define where the last byte address will be
  uint32_t addr=0;                                          // memory address (what we are trying to find) set to first possible value - interval initially subtracted - will be added back in later
  uint16_t pageAddr = pStart;                               // page address: backup memory starts on page 128
  uint16_t byteAddr = 0;                                    // byte address; start where first written byte might be
  uint8_t pByte;
  uint8_t bByte;
  uint8_t done = 0;
  //Serial.println("Finding address ");
  while (done < 1) {                              //This loop provides the last page address
    addr = (pageAddr << 10) + maxByte;
    pByte = readFlashByte(addr);    // read the byte
    if (pByte == 255) {
      done=1;
      break;
    }
    if (pageAddr > pEnd) {
      Serial.println("Problem locating current memory address. Consider erasing memory.");
      done=2;
      break;
    }
    pageAddr = pageAddr + 1;
  }

      
  //Serial.print("page address: "); Serial.println(pageAddr);
    
  while (done < 2) {                            //This loop provides the last byte address
    addr = (pageAddr << 10) + byteAddr;
    bByte = readFlashByte(addr);
    if (bByte == 255) {  //make sure the byte is empty...
      done=2;
      break;
    }
    if (byteAddr > maxByte) {
      Serial.println("Problem locating current memory address. Consider erasing memory.");
      done=2;
      break;
    }
    byteAddr = byteAddr + interval;
    //Serial.print(done); Serial.print(char(32)); Serial.println(byteAddr);
  }

  //Serial.print("page address: "); Serial.println(pageAddr); 
  //Serial.print("byte address: "); Serial.println(byteAddr);


  
  //Serial.print(pageAddr);
  uint32_t line = calcLine(addr, linesPerPage, interval, pStart);

  return line;  //this is the next place to write to.
}

uint32_t calcLine(uint32_t fAddr, uint8_t linesPerPage, uint8_t bytesPerLine, uint8_t pStart) {
  uint16_t pageAddr = (fAddr >> 10) - pStart;     // should be zero for first page
  uint16_t byteAddr = (fAddr & 0x000003FF);       // should be zero if no bytes are detected
  Serial.print(pageAddr); Serial.print(" "); Serial.println(byteAddr); 
  uint32_t line = (pageAddr * linesPerPage) + byteAddr / bytesPerLine;   
  return line;
}

uint32_t calcFlashAddr(uint32_t line, uint8_t linesPerPage, uint8_t bytesPerLine, uint8_t pStart) {
  uint32_t pageAddr = ((line / linesPerPage) + pStart) << 10;
  uint16_t byteAddr = (line % linesPerPage) * bytesPerLine;
  //Serial.print("page "); Serial.print(pageAddr, DEC); Serial.print("  byte "); Serial.println(byteAddr, DEC);
  return pageAddr + byteAddr;
}



uint32_t FlashGetAddr(uint16_t pStart, uint8_t pEnd, uint8_t interval) { //Seek out the last bit of stored data.
  uint16_t limit = (527 - interval) + 1;                  // define highest byte address that can be written to on each page.
  uint16_t lastByteAddr = (limit / interval) * interval;  // define where the last byte address will be
  uint32_t addr;                                          // memory address (what we are trying to find) set to first possible value - interval initially subtracted - will be added back in later
  uint16_t pageAddr = pStart;                             // page address: backup memory starts on page 128
  uint16_t byteAddr = 0;                                  // byte address; start where first written byte might be
  uint8_t pByte;
  uint8_t bByte;
  Serial.println("Finding address ");
  while (1) {
    addr = (pageAddr << 10) + lastByteAddr;
    pByte = readFlashByte(addr);    // read the byte
    if (pByte == 255) {
      break;
    }
    if (pageAddr > pEnd) {
      Serial.println("Problem locating current memory address. Consider erasing memory.");
      break;
    }
    pageAddr = pageAddr + 1;
  }
  while (1) {
    addr = (pageAddr << 10) + byteAddr;
    bByte = readFlashByte(addr);
    if (bByte == 255) {
      break;
    }
    if (bByte > limit) {
      Serial.println("Problem locating current memory address. Consider erasing memory.");
      break;
    }
    byteAddr = byteAddr + interval;
  }
  Serial.print("Address: ");
  Serial.print(addr, HEX);
  Serial.print(".  Page: ");
  Serial.print(pageAddr, DEC);
  Serial.print(".  Byte: ");
  Serial.println(byteAddr, DEC);
  return addr;
}

void pageErase(uint16_t pageAddr) {
  //Serial.print("Erasing page ");
  //Serial.println(pageAddr);
  uint32_t ePage = pageAddr << 10;
  flashOn();
  SPI.transfer(0x81);                    // opcode for page erase: 0x81
  SPI.transfer((ePage >> 16) & 0xFF);    // first of three address bytes
  SPI.transfer((ePage >> 8) & 0xFF);     // second address byte
  SPI.transfer(ePage & 0xFF);            // third address byte
  digitalWrite(FlashCS, HIGH);           // Deassert cs for process to start
  delay(35);                             // delay for page erase
  flashOff();
}



uint16_t checkTag(uint16_t startLine, uint8_t lineLength, uint16_t lastLine) { //Check if a tag matches one in a list in flash memory. return matching line number or 0xFFFF if no match.

  Serial.print("looking to match ");
  Serial.print(RFIDtagArray[0], HEX); Serial.print(RFIDtagArray[1], HEX); Serial.print(RFIDtagArray[2], HEX); Serial.print(RFIDtagArray[3], HEX); Serial.println(RFIDtagArray[4], HEX);
  //Serial.print(startLine); Serial.print(" ");  Serial.print(lineLength); Serial.print(" ");  Serial.println(lastLine);
  uint16_t tagline = 0xFFFF;
  for (uint8_t l = startLine; l <= lastLine; l++) {
    uint32_t fAddr = calcFlashAddr(l, 52, 10, 1);
    //Serial.print("flass addr ");  Serial.println(fAddr, DEC);
    readFlashArray(fAddr, cArray2, 10);              // read a line of data
    if (cArray2[0] == RFIDtagArray[0] & cArray2[1] == RFIDtagArray[1] & cArray2[2] == RFIDtagArray[2] & cArray2[3] == RFIDtagArray[3] & cArray2[4] == RFIDtagArray[4]) {
      tagline = l;
      Serial.print("match found on line "); Serial.println(l);
      break;
    }
  }
  if (tagline == 0xFFFF) {
    Serial.println("match not found.");
  }
  return (tagline); //return line of matching tag ID, note that cArray2 contains all information from the matching line.
}



////////////SD CARD FUNCTIONS////////////////////

void transferNetwork(String fileName) { //long function to load radio network information to flash memory
  uint8_t RFarr[5];
  char netArray[528];   //Byte array
  uint32_t fLen = 0;    //file length
  char c1; char c2;     //single character variables
  for (uint16_t ep = 1; ep <= 7; ep++) {     //First clear tag memory pages.
    pageErase(ep);
  }
  uint16_t posSD = 0;   //SD card position
  uint16_t cCount = 0;

  SDstart();
  fileName[1]='0'; fileName[2]='0'; fileName[3]='0'; 
  File myfile = SD.open(fileName, FILE_READ);  // attempt to open the network file
  if (!myfile) {
    Serial.print(fileName);
    Serial.println(" file not found");
    return;
  }
  if (myfile) {                     // if the file is available, read entire header in one byte at a time
    fLen = myfile.size();           // establish file length so you know when to stop reading from it
    Serial.print(fileName); Serial.print(" file found with "); Serial.print(fLen); Serial.println(" characters."); //note to user
    while (myfile.available()) {
      c1 = myfile.read();
      if ((c1 > 47 & c1 < 58) | (c1 > 64 & c1 < 91)) { //Only respond to numbers and capital letters
        Serial.print(c1);
        netArray[cCount] = c1;
        cCount++;
      }
      if (c1 == 13 | c1 == 10) {
        Serial.println("File header read");
        break;
      }
    }
    posSD = myfile.position();
    myfile.close();
  }
  SDpause();

  uint8_t numNodes = cCount / 4; //how many nodes in list
  uint8_t nameFound = 0;
  Serial.print(numNodes, DEC); Serial.println(" feeder IDs read in.");
   Serial.println(cCount);
  for (uint8_t i = 0; i < cCount; i=i+4) {
    Serial.println(i);
    if (deviceID[0] == netArray[i] & deviceID[1] == netArray[i + 1] & deviceID[2] == netArray[i + 2] & deviceID[3] == netArray[i + 3]) {
      nameFound = 1;
      radioNet = netArray[i];                                       //radio network number (on flash page 0, byte 30)
      netGrp = asciiToHex(netArray[i + 1]);                                     //group id within network (on flash page 0, byte 31)
      nodeID = (asciiToHex(netArray[i + 2]) << 4) + asciiToHex(netArray[i + 3]); //radio node number for this feeder (on flash page 0, byte 32)
      if (i == numNodes * 4 - 4) { //If we are on the last device ID in the list, we need the node ID of the first device in the list.
        targetNode = (asciiToHex(netArray[2]) << 4) + asciiToHex(netArray[3]); //target node for all transmissions (on flash page 0, byte 33)
      } else { //Otherwise we want the node id for the next device in line
        targetNode = (asciiToHex(netArray[i + 6]) << 4) + asciiToHex(netArray[i + 7]); //target node for all transmissions (on flash page 0, byte 33)
      }
      break;
    }
  }

  //Find maximum group
  maxGrp = 0;
  for (uint8_t i = 1; i < cCount; i=i+4) {
    if(netArray[i] > maxGrp) {maxGrp = netArray[i];}
    Serial.print(i); Serial.print(" "); Serial.println(netArray[i], DEC);
  }
  maxGrp = asciiToHex(maxGrp);
  
  Serial.print("name found: "); Serial.println(nameFound, DEC);
  if (nameFound == 0) {
    Serial.println();
    Serial.print("ERROR - No matching feeder name in network file!!");
  }
  if (nameFound == 1) {
    //write to flash addresses 30 and so on
    netArray[0] = radioNet; netArray[1] = netGrp; netArray[2] = nodeID; netArray[3] = targetNode;
    netArray[4] = numNodes; netArray[5] = maxGrp;
    writeFlashArray(30, netArray, 6);  //write array to flash
    Serial.print("radio Network: "); Serial.print(netArray[0]); Serial.print("  Group: ");
    Serial.print(netArray[1]); Serial.print("  Node: "); Serial.println(netArray[2], HEX);
    Serial.print("Target node: "); Serial.print(netArray[3], HEX);  Serial.print("  Total nodes: "); 
    Serial.println(numNodes); Serial.print(" Last group: "); Serial.println(maxGrp, DEC);
  }


//  byte 30 = radio network number
//    byte 31 = network group ID
//    byte 32 = radio node ID
//    byte 33 = target radio ID
//    byte 36/37 = number of tags in storage - 16 bit integer
//    byte 34 = number of nodes in network (must start with 1 and count up)
//    byt3 35 = maximum group number (min is always 1) 

  

  // Add data from line 1 to flash memory

  uint16_t flashLine = 0;
  while (posSD < fLen - 1) {
    SDstart();
    myfile = SD.open(fileName, FILE_READ);  // attempt to open the network file
    //Serial.print("SD position: "); Serial.println(posSD);
    //Serial.print("c1 value "); Serial.println(c1, DEC);
    myfile.seek(posSD);
    uint8_t rfCnt = 0;

    while (c1 != 32 & c1 != 23) {
      c1 = myfile.read();
      if (c1 == 23) {
        break;
      }
      //Serial.println(c1);
      if ((c1 > 47 & c1 < 58) | (c1 > 64 & c1 < 71)) { //Only respond to numbers and capitals A-F
        c2 = myfile.read();
        if ((c2 > 47 & c2 < 58) | (c2 > 64 & c2 < 71)) { //Only respond to numbers and capitals A-F
          //Serial.print(c1); Serial.print(c2), Serial.print(" ");
          RFarr[rfCnt] = (asciiToHex(c1) << 4) + asciiToHex(c2);
          //Serial.print(RFarr[rfCnt], HEX), Serial.print(" ");
          rfCnt++;
        } else {
          Serial.println("File reading error");
          c1 = 23;
        }
      }

      if (c1 == 32) {          // After space character (32) look for start group number
        c2 = myfile.read();    // Should be start group
        c2 = asciiToHex(c2);    // Change from character to number
        //Serial.print("  Start group: "); Serial.print(c2); Serial.println();
        //break;
      }
    }
    posSD = myfile.position();

    myfile.close();
    SDpause();
    //Serial.print("Send RFID to flash "); Serial.print(RFarr[0],HEX); Serial.print(RFarr[2],HEX); Serial.println(RFarr[4],HEX);
    uint32_t flAddr = calcFlashAddr(flashLine, 52, 10, 1);

    //uint32_t calcFlashAddr(uint32_t line, uint8_t linesPerPage, uint8_t bytesPerLine, uint8_t pStart) {

    //Serial.print("flashLine "); Serial.print(flashLine, DEC); Serial.print(" flAddr "); Serial.println(flAddr, HEX);
    Serial.print("Writing to line "); Serial.print(flashLine, DEC); Serial.print(" "); Serial.print(RFarr[0], HEX);
    Serial.print(RFarr[1], HEX); Serial.print(RFarr[2], HEX); Serial.print(RFarr[3], HEX); Serial.print(RFarr[4], HEX);
    Serial.print(" initial group:  "); Serial.print(c2);
    char writeData[10] = {RFarr[0], RFarr[1], RFarr[2],          // Create an array representing an entire line of data
                          RFarr[3], RFarr[4], c2, 0,             // RFID(5 bytes), current grp (c2), history (2 bytes), trial (1 byte, always start with 1), empty(255). 
                          0, 1, 0xFF };
    writeFlashArray(flAddr, writeData, 10);  //write array - entire line of data 
    flashLine++;
    if (c1 != 23) {
      c1 = 50; //keeps loop from quitting
    }
  }
  Serial.print("number of tags:  "); Serial.print(flashLine);
  netArray[0] = (flashLine) >> 8; netArray[1] = (flashLine & 0x00FF);  //flashLine starts at zero, so don't subtract 1 to get number of tags.
  writeFlashArray(36, netArray, 2);  //write number of tags to lines 36 and 37
  numTagsStored = flashLine;

  //  read and test
  
     Serial.println(); Serial.println(); Serial.println();
     char xx[8] = {0, 0, 0, 0, 0, 0,0,0};
     uint32_t fAddr = calcFlashAddr(6, 52, 10, 1);
     readFlashArray(fAddr, xx, 8);
     //readFlashArray(30, xx, 8);
     Serial.print(xx[0], HEX); Serial.print(xx[1], HEX); Serial.print(xx[2], HEX); Serial.print(xx[3], HEX); Serial.println(xx[4], HEX); 
     Serial.print(xx[5], DEC);  Serial.print(" ");  Serial.print(xx[6], DEC); Serial.print(" "); Serial.println(xx[7], DEC);
          
     //readFlashArray((1<<10)+30, xx, 5);
     //Serial.print(xx[0], HEX);  Serial.print(xx[1], HEX);  Serial.print(xx[2], HEX);  Serial.print(xx[3], HEX);  Serial.println(xx[4], HEX);
}


byte asciiToHex(byte x) { //Text conversion for transfering data
  if (x > 0x39) x -= 7; // adjust for hex letters upper or lower case
  x -= 48;
  return x;
}


//Startup routine for the SD card
bool SDstart() {                 // Startup routine for the SD card
  digitalWrite(SDselect, HIGH);  // Deactivate the SD card if necessary
  digitalWrite(FlashCS, HIGH);   // Deactivate flash chip if necessary
  pinMode(SDon, OUTPUT);         // Make sure the SD power pin is an output
  digitalWrite(SDon, LOW);       // Power to the SD card
  delay(20);
  digitalWrite(SDselect, LOW);   // SD card turned on
  if (!SD.begin(SDselect)) {     // Return a 1 if everyting works
    //Serial.println("SD fail");
    return 0;
  } else {
    return 1;
  }
}

void SDpause() {                // Stop the SD card but maintain power to it--sometimes powering down makes it unable to come back on.
  delay(20);                    // delay to prevent write interruption
  SD.end();                     // End SD communication
  digitalWrite(SDselect, HIGH); // SD card turned off
  //digitalWrite(SDon, HIGH);     // power off the SD card
}

//Stop routine for the SD card
void SDstop() {                 // Stop routine for the SD card
  delay(20);                    // delay to prevent write interruption
  SD.end();                     // End SD communication
  digitalWrite(SDselect, HIGH); // SD card turned off
  digitalWrite(SDon, HIGH);     // power off the SD card
}

// Remove a file on the SD card
void SDremoveFile(String killFile) {   // Remove a file on the SD card
  //Serial.println("killing file");    // Message
  SDstart();                           // Enable SD
  SD.remove(killFile);                 // Delete the file.
  SDstop();                            // Disable SD
}

// Write an entire string of data to a file on the SD card
bool SDwriteString(String writeStr, String writeFile) {  // Write an entire string
  SDstart();                                             // Enable SD
  bool success = 0;                                      // valriable to indicate success of operation
  File dFile = SD.open(writeFile, FILE_WRITE);           // Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
  if (dFile) {                                           // If the file is opened successfully...
    dFile.println(writeStr);                             // ...write the string...println appends a carriage return and new line (2 more bytes)
    dFile.close();                                       // ...close the file...
    success = 1;                                         // ...note success of operation...
  }
  SDstop();                                              // Disable SD
  return success;                                        // Indicates success (1) or failure (0)
}




void updateSD(uint8_t lineLen) {
  uint32_t sdLines;
  uint32_t fLength; //length of SD file
  deviceIDstr = String(deviceID);
  char fnum[2];
  uint8_t fcount = 0;
  uint8_t match = 1;
  
  if(dataLine == 0) {
    Serial.println("No data in memory");
    return;
  }

  SDstart();
  do {                    //loop through file names to find the lastest one.
    fcount = fcount + 1;
    if (fcount == 100) {
      fcount = 1;
    }
    sprintf(fnum, "%02d", fcount);
    dataFile = String(deviceIDstr) + "_D" + String(fnum) + ".TXT";
    //Serial.print("lookiing for "); Serial.println(dataFile);
  } while (SD.exists(dataFile));  //end of do/while when file is not found


  if (fcount != 1) {
    //Check whether the last matched file corresponds with the data in Flash memory.
    sprintf(fnum, "%02d", fcount - 1); //Go back one to get to last file
    dataFile = String(deviceIDstr) + "_D" + String(fnum) + ".TXT";
    logFile = String(deviceIDstr) + "_L" + String(fnum) + ".TXT";

    Serial.print("Current data file is: "); Serial.println(dataFile);
   
    File readFile = SD.open(dataFile, FILE_READ);
    fLength = readFile.size();
    //Serial.println("SD file length: "); Serial.println(fLength);
    uint32_t sdLines = (fLength / SDLineLength)-1; // Lines start at zero. This gives the last line number
    //Serial.println("SD file number of lines: "); Serial.println(sdLines);
    //Serial.println("Flash number of lines: "); Serial.println(dataLine);

    //Check for data match first and last line of SD file.
    //Serial.println("Comparing first lines from flash and SD");
    readSDLine(0, lineLen, dataFile, 1);   //provide which line to read, length of each line, and file to access. Line is stored in cArray1
    readFlashLine2(0, 128, 16, 1); //provide which line to read, the start page, length of each line, and whether to print line or not. Line reads into cArray2
    
    for (uint8_t i=0; i < 30; i++) {
      //Serial.print(cArray1[i]); Serial.print(" ? "); Serial.print(cArray2[i]);
      if (cArray1[i] != cArray2[i]) {
        match = 0; 
        //Serial.print(" match "); Serial.println(match);           
        break;
        //Serial.print("match failed on first line");
      }
      //Serial.print(" match "); Serial.println(match); 
    }

    if(match==1){
      //Serial.println("Comparing last SD line to corresponding Flash line");
      //Serial.print("File size is ");
      //Serial.println(fLength);
      //sdLines = fLength / SDLineLength - 1; 
      //Serial.print("last line is ");Serial.println(sdLines);
      readSDLine(sdLines, SDLineLength, dataFile, 1);   //provide which line to read, length of each line, and file to access. Line is stored in cArray1
      readFlashLine2(sdLines, 128, 16, 1); //provide which line to read, the start page, length of each line, and whether to print line or not. Line reads into cArray2
      for (uint8_t i=0; i < 30; i++) {
        if (cArray1[i] != cArray2[i]) {
          match = 0; break;
          //Serial.print("match failed on last line");
        }
      }
    }
    //Serial.print(" match "); Serial.println(match); 
    if (match == 1) {
      Serial.println("Matching data file found on SD card");
      //Serial.print(dataLine-1); Serial.print(" "); Serial.println(sdLines);
      if (dataLine-1 > sdLines && dataLine != 0) {
        Serial.println("Need to transfer some data...");
        writeMemRFID(sdLines + 1, 128, dataFile); //Update SD RFID file with first line to write, start page, file name.
        writeMemLog(8, logFile);  //Just Overwrite all log data, variables = start page and logFile name
      } else {
        Serial.println("SD card already up to date");
      }
    } 
  }

  if (fcount == 1 | match != 1) {
    //if there is no file match, then just establish the first one.
    
    Serial.println("Files don't match up. Making new file called ");
    
    if (fcount == 1) {            //Should be 'if no file exists.'
      sprintf(fnum, "%02d", 1);
      dataFile = String(deviceIDstr) + "_D" + String(fnum) + ".TXT";
      logFile = String(deviceIDstr) + "_L" + String(fnum) + ".TXT";}
    
    if (fcount != 1) {
      uint8_t df6 = asciiToHex(dataFile[6]); uint8_t df7 = asciiToHex(dataFile[7]); 
      df6 = df6*10 + df7 + 1;
      sprintf(fnum, "%02d", df6);
      dataFile = String(deviceIDstr) + "_D" + String(fnum) + ".TXT";
      logFile = String(deviceIDstr) + "_L" + String(fnum) + ".TXT";}

    Serial.println(dataFile); 
    Serial.print("New log file "); Serial.println(logFile); 
    //Serial.println("final file names ");
    //Serial.println(dataFile);
    //Serial.println(logFile);
    
    if (dataLine > 0) {
      Serial.println("Moving flash data to new files on SD card");
      writeMemRFID(0, 128, dataFile);     // Write a new file to the SD card first line to write (0), start page, file name.
      writeMemLog(8, logFile);            // Write all log data, variables = start page and logFile name
    } else {
      Serial.println("New files established on SD Card");
      dataFile = String(deviceIDstr) + "_D" + String(fnum) + ".TXT";
      logFile = String(deviceIDstr) + "_L" + String(fnum) + ".TXT";
    }
  }
}


///////Radio communication Functions /////////////


uint8_t radCheck(uint8_t motState){       //Delay while checking radio every x milliseconds. Returns a status value if a message is received: 1=OK, 2=Transmission error
  if (radio.receiveDone()) { // Got one!
     for (byte i = 0; i < radio.DATALEN; i++) {cArray1[i] = radio.DATA[i];}  // Harvest the data as quickly as possible
     uint8_t radNumBytes = radio.DATALEN;                                      // Store how many bytes were in the message
     radio.sendACK();                                                       // Send Acknowledgement--This seems to clear radio.DATALEM--so extract it first
     
     if(motState != 0) { //Stop motor if it is running (may need to move this up)
        motAllStop(10);
     }

     Serial.print("received data ");
     for (byte i = 0; i < radNumBytes; i++) {Serial.print(cArray1[i], HEX); Serial.print(" ");}
     Serial.println();
     
     if(cArray1[9]==1 | cArray1[9] == 10) {   //message to relay and update trial data
       //Check if message came from this feeder and if so validate it.
       if(cArray1[10]==deviceID[0] & cArray1[11]==deviceID[1] & cArray1[12]==deviceID[2] & cArray1[13]==deviceID[3]) { //see if this device is the origin of the message.
         Serial.println("Recieved message that originated here");
         uint8_t messageOK = 1;
         for(byte i=0; i < 10; i++){ 
            if(cArray1[i]!=transArray[i]) {
              messageOK=0; 
              Serial.println("Transmission error. Need to resend");
              return 2;
            }
         }
         if(messageOK==1) {
            Serial.println("Transmission confirmed");
            return 1;
         }
         
       } else {  //if the current device is not the message origin, then relay to next feeder and store the data
         for(uint8_t i=0; i<14; i++){ cArray1[i] = radio.DATA[i]; }
        //relay first
         if (radio.sendWithRetry(targetNode, cArray1, 14)) {
            Serial.println("Data messages recieved and transmitted: ACK received!");
            Serial.print("received data ");
            for (byte i = 0; i < 10; i++) {Serial.print(cArray1[i], HEX); Serial.print(" ");}
            Serial.println();
         } else {
            Serial.println("Data messages recieved and transmitted: ACK Failed!");
         }
       
         
         for(uint8_t i=0; i<5; i++){ RFIDtagArray[i] = cArray1[i]; } //load tag ID into RFIDtagArray global array variable 
         
         if(cArray1[9] == 1) { //Update trial info if tag is in the target list
           uint16_t whichTag = checkTag(0, 10, numTagsStored);  //uint16_t startLine, uint8_t lineLength, uint16_t lastLine { //Check if a tag matches one in a list in flash memory. return matching line number or 0xFFFF if no match. loads matching line into cArray2
           if(whichTag != 0xFFFF) {
             uint32_t fAd = calcFlashAddr(whichTag, 52, 10, 1);   //calculate the flash address for writing array
             
             Serial.println("Tag match found. Updating trial info on flash: ");
             for(uint8_t i=0; i<9; i++){ Serial.print(cArray1[i], HEX); Serial.print(" "); }
             Serial.println();
             writeFlashArray(fAd, cArray1, 9);  //write 9 bytes to flash
    
             //read flash to show new line.
             char BA[10]={0,0,0,0,0,0,0,0,0,0};
             readFlashArray(fAd, BA, 9);
             static char text[15];                       // make an array for writing data lines
             sprintf(text, "%02X%02X%02X%02X%02X, %d, ", BA[0], BA[1], BA[2], BA[3], BA[4], BA[5]);
             history.h1=BA[6]; history.h2=BA[7];
             Serial.print("New trial data read from flash.");
             for(uint8_t i=0; i<9; i++){ Serial.print(BA[i], HEX); Serial.print(" "); }
             Serial.println();
             Serial.print(text); printBits(history.h16b, 12); Serial.print(", "); Serial.println(BA[8], DEC);
             Serial.println();
           } else { Serial.println("Tag match not found -- problem with trial info?");}
         } else {Serial.println("Not a target tag. Trial info unchanged.");}

//   transArray[0]=RFIDtagArray[0]; transArray[1]=RFIDtagArray[1]; transArray[2]=RFIDtagArray[2]; transArray[3]=RFIDtagArray[3]; transArray[4]=RFIDtagArray[4];
//          transArray[5]=currentGrp; transArray[6]=history.h1; transArray[7]=history.h2; transArray[8]=reversals; transArray[9]=1; 
//          transArray[10]=deviceID[0]; transArray[11]=deviceID[1]; transArray[12]=deviceID[2]; transArray[13]=deviceID[3];


         uint8_t nGrp = asciiToHex(cArray1[11]);
         uint8_t node1 = asciiToHex(cArray1[12]); uint8_t node2 = asciiToHex(cArray1[13]);
         uint8_t node3 = node1*10 + node2;
         
         //Write a new data line to RFID data.
         rtc.updateTime();                   // get time from clock
         unixTime.unixLong = getUnix();      // get unix time
         char fData[16] = {cArray1[0], cArray1[1], cArray1[2], cArray1[3], cArray1[4],     // RFID CODE
                            unixTime.b1, unixTime.b2, unixTime.b3, unixTime.b4,            // unix time
                            cArray1[10], nGrp, node3,                                      // network, group, and node of detecting feeder
                            cArray1[5], cArray1[8], cArray1[6], cArray1[7]};               // current group, trial, and hisotry (2 bytes)
         
         Serial.println("Saving RFID data to flash");
         flashAddr = calcFlashAddr(dataLine, 33, 16, 128);     // get the correct flash address based on the current data line
         writeFlashArray(flashAddr, fData, 16);                // write array
         dataLine = dataLine + 1;                              // advance the data line 
       }
     }

       

     if(radio.DATA[9]==2) {   //message trigger clock sync
          rtc.updateTime();                                         // get time from clock
          syncTime = getUnix();
     }
      
     if(radio.DATA[9]==3) {   //send stored time stamp to node in request message                                         
          uint8_t targ = radio.DATA[0];
          unixTime.unixLong = syncTime;
          cArray1[0]=unixTime.b1; cArray1[1]=unixTime.b2; cArray1[2]=unixTime.b3; cArray1[3]=unixTime.b4;
         if (radio.sendWithRetry(targ, cArray1, 4)) {
            Serial.println("ACK received!");
         } else {
            Serial.println("ACK Failed!");
         }
     }
    if(motState != 0) {
       motGo(motState); //restart motor if it was running.
    }
  } 
  return 1;
}

//void clockCall() //tell all feeders to store their timestamps

//bool RelayDat(message) // process, augment, and relay received message


///////Motor functions/////////////Motor functions/////////

/* Motor for grackle feeder - hook up motor positive (red) to output A1 on motor controller
   digitalWrite(MOTRA, HIGH); digitalWrite(MOTLA, LOW) -- elevator goes down
   digitalWrite(MOTRA, LOW); digitalWrite(MOTLA, HIGH) -- elevator goes up

   Motor switch is open when plunger is extended, and will read high on the motor switch pin.
   Motor swith is closed when the button is depressed and will read low.
*/

void motAllStop(uint8_t del) { //Makes and abrupt stop using the brake.
  motGo(B00001111);
  delay(del);        // adjust as needed
  motGo(0);
}


void motGo(uint8_t motState) {
  //motState --> 0 0 0 0 mStby, MOTPWMA, MOTRA, MOTLA 
  digitalWrite(mStby, bitRead(motState, 3)); 
  digitalWrite(MOTPWMA, bitRead(motState, 2)); 
  digitalWrite(MOTRA, bitRead(motState, 1)); 
  digitalWrite(MOTLA, bitRead(motState, 0));   
}

// void motUnload() {
//   pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
//   uint8_t sw = digitalRead(mSwitch);
//   Serial.print("Ejecting elevator "); Serial.println(sw);
//   motGo(B00001101);              //Motor code to raise elevator.
//   delay(4000);                   //
//   motGo(0);                      //Everything off - slow stop.
// }

// void motLoad() {
//   pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
//   uint8_t sw = digitalRead(mSwitch);
//   Serial.print("Load elevator "); Serial.println(sw);
//   if(sw == 0) {
//     Serial.print("Switch depressed. aborting operation");
//     return;
//   }
//   uint8_t swp = sw; 
//   uint8_t cycle = 120;
//   uint8_t offcyc = 255-cycle;
//   digitalWrite(mStby, HIGH); digitalWrite(MOTRA, HIGH); digitalWrite(MOTLA, LOW);

//   uint8_t tog = 0;
//   while(tog < 4) {
//     digitalWrite(MOTPWMA, LOW); //speed control
//     delayMicroseconds(offcyc);
//     digitalWrite(MOTPWMA, HIGH); //speed control
//     delayMicroseconds(cycle);
//     sw = digitalRead(mSwitch);
//     if(swp != sw) {
//       tog++;
//       swp=sw;
//       for(uint16_t i = 0; i < 100; i++) {  //for debounce
//         digitalWrite(MOTPWMA, HIGH); 
//         delayMicroseconds(cycle);
//         digitalWrite(MOTPWMA, LOW); 
//         delayMicroseconds(offcyc);
//       }
//     }
//   }
//   motAllStop(20);
// }

void motFeed() { //just drop the hatch to release food
  uint8_t cycle = 120;
  uint8_t offcyc = 255-cycle;
  serial.println("open door"); 
  // pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
  // uint8_t sw = digitalRead(mSwitch); //switch should be open --> 1
  uint8_t motState = B00001110; 
  motGo(motState); 
  // while(sw == 1) {
  //     sw = digitalRead(mSwitch);
  //     delay(3);
  //     //radCheck(motState); // check radio every cycle - if there's a message stop the motor, deal with it, and resume.
  // }

  // for(uint8_t i = 0; i < 50; i=i+10) {  //Delay for bounce
  //    delay(10); 
  //    //radCheck(motState);
  // }
  // sw = 0;
  // while(sw == 0) {
  //   digitalWrite(MOTPWMA, HIGH); 
  //   delayMicroseconds(cycle);
  //   digitalWrite(MOTPWMA, LOW); 
  //   delayMicroseconds(offcyc);
  //   sw = digitalRead(mSwitch);
  //   radCheck(motState);
  // }
  delay(1000);
  motAllStop(20);
  // feedState = 2;
}

// void motClose() {   // raise hatch back up
//   uint8_t cycle = 120;
//   uint8_t offcyc = 255-cycle; 
//   serial.println("close door"); 
//   pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
//   uint8_t sw = digitalRead(mSwitch); //switch should be open --> 1
//   uint8_t motState = B00001101; 
//   motGo(motState); 
//   while(sw == 1) {
//       sw = digitalRead(mSwitch);
//       delay(5);
//       radCheck(motState); //check radio every cycle - if there's a message stop the motor, deal with it, and resume.
//   }
//   for(uint8_t i = 0; i < 50; i=i+10) {  //Delay for bounce
//      delay(10); 
//      radCheck(motState);
//   }
//   sw = 0;
//   while(sw == 0) {
//     digitalWrite(MOTPWMA, HIGH); 
//     delayMicroseconds(cycle);
//     digitalWrite(MOTPWMA, LOW); 
//     delayMicroseconds(offcyc);
//     sw = digitalRead(mSwitch);
//     radCheck(motState);
//   }
//   motAllStop(20);
//   feedState = 1;
// }

// uint8_t motRecharge() {  //raise elevator to load food (requires interrupt from food sensor), returns 1 if food fall detected
//   uint8_t cycle = 190;
//   uint8_t offcyc = 255-cycle; 
//   foodOK = 0;
//   serial.print("recharge feeder"); 
//   pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
//   uint8_t sw = digitalRead(mSwitch); //switch should be open --> 1
//   serial.println("recharge feeder"); 
//   uint8_t motState = B00001101; 
//   motGo(motState); 
  
//   for(uint16_t i = 0; i < 1000; i=i+10) {  //Delay for a full second to let elevator move
//      delay(10); 
//      radCheck(motState);
//   }

//   attachInterrupt(feedInt, FoodDet, FALLING);               // Set up interrupt to detect high to low transition on interrupt pin

//   sw = 0;
//   while(sw == 0) {                //Slow down a bit
//     digitalWrite(MOTPWMA, HIGH); 
//     delayMicroseconds(cycle);
//     digitalWrite(MOTPWMA, LOW); 
//     delayMicroseconds(offcyc);
//     sw = digitalRead(mSwitch);
//     radCheck(motState);
//   }
//   motAllStop(20);
//   radCheck(0);
  
//   for(uint16_t timr = 0; timr < 400; timr=timr+10) {  //Delay to let food fall
//      delay(10); 
//      if(foodOK == 1) break;
//      radCheck(0);
//   }

//   feedState=3;
//   delay(300);
//   if(foodOK==0) {
//     serial.println("NO FOOD!!"); 
//     return 0;
//   } else {
//     serial.println("Food detected -- OK");
//     return 1; 
//   }
// }


// void motReturn() {  //lower elevator to resting posiiton
//   uint8_t cycle = 150;
//   uint8_t offcyc = 255-cycle; 
//   serial.println("recharge feeder"); 
//   pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
//   uint8_t sw = digitalRead(mSwitch); //switch should be open --> 1
//   uint8_t motState = B00001110; 
//   motGo(motState); 
  
//   for(uint16_t i = 0; i < 1000; i=i+10) {  //Delay for a full second to let elevator ascend
//      delay(10); 
//      radCheck(motState);
//   }
//   sw=0;
//   while(sw == 0) {                //Slow down a bit
//     digitalWrite(MOTPWMA, HIGH); 
//     delayMicroseconds(cycle);
//     digitalWrite(MOTPWMA, LOW); 
//     delayMicroseconds(offcyc);
//     sw = digitalRead(mSwitch);
//     radCheck(motState);
//   }
//   motAllStop(30);
//   feedState = 1;
// }

void FoodDet() {     
  foodOK = 1;
}

///////Sleep Function/////////////Sleep Function/////////

void lpSleep() {
  radio.sleep();
  digitalWrite(MOTR, HIGH) ;                         // Must be set high to get low power working - don't know why
  shutDownRFID();                                    // Turn off both RFID circuits
  attachInterrupt(INT1, ISR, FALLING);               // Set up interrupt to detect high to low transition on interrupt pin
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |     // Configure EIC to use GCLK1 which uses XOSC32K
                      GCLK_CLKCTRL_GEN_GCLK1 |       // This has to be done after the first call to attachInterrupt()
                      GCLK_CLKCTRL_CLKEN;
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;        // disable USB
  SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;        // disable ms clock

  __WFI();    //Enter sleep mode
  //...Sleep...wait for interrupt
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;         // enable USB
  detachInterrupt(INT1);                             // turn interrupt off
  SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;         // Enable clock
  Debug = 0;
  serial.begin(9600);
}

void ISR() {     // dummy routine - no interrupt activity needed
  byte SLEEP_FLAG = false;
}
