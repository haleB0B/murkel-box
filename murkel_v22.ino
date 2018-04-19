
#define VERSION "MurkelV22 init "

/*V9
  added menu templates
  added power off 60 seconds with counter
  included volume control on KEYPLUS and KEYMINUS
  playing now, but serial query to DFPLayer causes glitch in sound -> go to A7-Busy pin readout


  power off loop - still seems to pull too much power with DFPlayer on, powerbank does not switch off
  Power on switch - additional switch included
  order of files - more investigation required
  no query of player possible - player seems to have problem, works with new player
  done 1. set day, month, year,
  fore and back tilt switches folder if mukrle is on

  V12

  implement learn function

  v13

  implemented play with and without murkel depending on setting

  pause and play function

  sensitivity of IMU set to |5|

  v14

  updates on screen message layout

  v15
  included alarm setting and alarm on/off

  v16
  clean up of code

  v17
  implement save and load of parameters
  implement menu item to trigger save of parameters
  checked new sleep mode - d2 pin set to low, attach interrupt on high... does not work, maybe the AVR gets woken up from sleep mode through int?

  v18
  update checkForMurkel and murkelPlay to enable different RFID
  implement learn mode - new rfid stored in eeprom with maxMurkelFolder ++

  commented out all  serial.print as mem >75% used-> strage behaviour

  implemented alarm mode, song currently static

  removed some inconsistencies, e.g. murkelTitle advances with every remount

  v19
  serial print reactivated

  v20
  reshuffeled menue items
  cleaned up lcd outputs
  included shell case for init rfid store
  working version!! except playing without murkels is not working anymore - no clue why

  V21
  updated noise delay to 1500

  v22
  updates on function names
  cleaned up comments
  startup voice message in folder 99
  implemented init EEPROM 


*************** ToDo *****************************

  done 2. alarm - play alarm song

  3. sleep mode - go to low power when no song plays and nothing happens - inactivity

  4. auto switch off after certain time (einschlafzeit)

  5. play ("advertisements") sounds/recorded voice with every menu item or function of murkel box

  done 6. implement "knocking/tilt" control (next song - tilt left/right, volume - tilt forth and back)

  done 7. Folders are assigned to Murkels
  Murkel on = play this folder from beginning (optional save last played position

  done 8. implement learn mode: structure - 512 EEPROM slots,

  EEPROM
  n-th offet=length of parameter array + 6*n
  00      01      02      03      04      05
  RFID[0] RFID[1] RFID[2] RFID[3] Folder  lastsongplayed

  not set = definition[6n+4]=0xFF

  9. setup -
  RFID:
    display - show assignment
    learn -> assign folder to rfidID
    delete -> deassign folder from RFID
  parameters (stored on EEPROM):
    last volume, max volume,tilt on/off/sensitivity, auto switch off timespan, childs sleep time,set alarm song, "lastplayed or first song", temperature offset
    load/save all/groups=parameter, assignments - upload/offload EEPROm to serial port

  10. detect lifting state - integrate over gz and see if box moves vertically or horizontally - switch off tilt functions

  11. start laughing when being shaken

  cancel -   12. led lights/glow?

  13. temperature display

  14. notouch screen view (e.g. show song markers, play graphics,...) - change as soon as key switched

  15. LCD scroilling with every new message - outopṕut function

  16. insert compiler options to not include standard settings in menu, eg date, clock, ...

*/

// ************** General libraries *****************

#include "Arduino.h"
#include <EEPROM.h>
#include "LowPower.h"
#include <avr/sleep.h>
//General constants, definitions

#define HWSERIALSPEED 9600    // serial speed


//used for EEPROM address store location of the values, all parameters are stored from highest to lower address, RFID info from lower to highest to have most flexibility
#define EEPROMADRESSRANGEMAX 1000 // leave 23 bytes for parameters
#define MURKELVOLUME 1023
#define MAXMURKELFOLDER 1022
#define MURKELTITLE 1021
#define MURKELFOLDER 1020
#define MURKELUSAGE 1019
#define ALARMACTIVE 1018
#define ALARMHOUR 1017
#define ALARMMINUTE 1016



// modes used to call clockdisplay
#define HOURMINUTE 0
#define DAYMONTHYEAR 1
#define ALARMHOURMINUTE 2

#define DELAY1 300 //general loop delay
#define DELAY2 1000 //delay for debounce purpose

int monthDays[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// ************** Menu *****************

#define MAXMENUMODE 14 // 
//#define MAXLEARNSUBLEVEL XX //
#define PAUSE 0 // halt murkel, powerbank switchoff due to missing load trigger
#define SWITCHOFF 14 // display all RFID/folder relations
#define ALARM 2 // assign newly read UID to folder
#define LEARN 13 // edit or delete existing assignment
//#define LEARNLOAD XX // load assignments from serial
//#define LEARNSAVE XX // write assignments to serial
#define LEARNINIT 12 // initialize learning set - should not be used
#define CLKMIN 5 // clock setting min, hrs, year, month, day
#define CLKHRS 6 // clock setting min, hrs, year, month, day
#define CLKDAY  7 // clock setting min, hrs, year, month, day
#define CLKMON 8 // clock setting min, hrs, year, month, day
#define CLKYR 9 // clock setting min, hrs, year, month, day
#define CLKALMIN 3 // alert setting min, hrs, set to nonsense hours/min to switch off
#define CLKALHRS 4 // alert setting min, hrs, set to nonsense hours/min to switch off
//#define PRMVOLMAX XX  // maximum adjutsable volume
//#define PRMTILTSENSITIVITY XX // how sensitive should the tilt control be
//#define PRMINACTIVE XX // time period to switch of due to inactivity
//#define PRMSLEEPTIME XX // time by which box will be switched off automatically
#define PRMMURKELUSAGE 1 // change if murkel is required for playing - save state in eeprom
#define PRMLOAD 10
#define PRMSAVE 11

#define MAXPAUSELOOPS 1000000 // get out of pause loop after N iterations

#define KEYPLUS   4
#define KEYOK     A3
#define KEYMINUS  A2

#define MENUMODE KEYOK
#define MENUUP KEYPLUS
#define MENUDOWN KEYMINUS
#define MENUMODETIMEMILLIS 5000

#define DEBOUNCETIME 250

//******* powerbank *****************

// keep powerbank switched on by short 70mA current peak, transistor base on D5
#define POWERBANKLOADTRIGGER 6 // D6 pin controls base of transistor for load trigger
#define POWERINTERVALLMAX 30000

unsigned long previousMillisPower = 0;


// ************** Unused libraries for later usage *****************



// ************** Software serial libraries *****************

#include "SoftwareSerial.h"

#define SOFTRX 9
#define SOFTTX 5
// Instantiate objects
SoftwareSerial murkelSerial(SOFTRX, SOFTTX); // RX, TX



// ************* IMU section **************

#include "I2Cdev.h"
#include "MPU6050.h"
#include <helper_3dmath.h>                  // calc functions

// I2C library - requied for RTC, IMU
#include <Wire.h>


// Variables, Definitions, constants for RTC
int16_t ax, ay, az;
int16_t gx, gy, gz;

//Tiltrate (a/1000) to switch to next song or folder
#define MAXLEFTTILT 5
#define MAXRIGHTTILT -5
#define MAXBACKTILT 5
#define MAXFORETILT -5

// Instantiate objects
MPU6050 murkelGyro(0x69); // <-- use for AD0 high



// ************* RTC section **************
// real time clock libraries
#include <DS3231.h>

// Variables, Definitions, constants for RTC

bool h12;   // 12 or 24 h setting
bool PM;
bool Century;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year, hourIndex, minuteIndex, logmode = 0, learnMode = 0;

byte alarmMinute = 0, alarmHour = 0;
bool alarmActive = false;

// Instantiate objects
DS3231 murkelClock;



// ************* LCD section **************

#include <LiquidCrystal.h>
#include <SPI.h> // SPI-Bibiothek hinzufügen

// Variables, Definitions, constants for LCD

#define FIRSTLINE 0
#define SECONDLINE 1
#define BOTHLINES 2

#define LCDDATA4 8
#define LCDDATA5 7
#define LCDDATA6 3
#define LCDDATA7 2
#define LCDEN 15
#define LCDRS 14

char lineBuffer [2][17] = { {"                "}, {"                "}} ;

//Instantiate object
LiquidCrystal lcd(LCDRS, LCDEN, LCDDATA4, LCDDATA5, LCDDATA6, LCDDATA7);


// ************* RFID section **************
//near field communications library

#include <MFRC522.h>

// Variables, Definitions, constants for RFID
#define RFIDBYTES 6 // 4* bytes for RFID tag, 1 byte for assigned folder, 1byte for # of songs or readout as function of folder, 1 byte for last played song
#define MAXMURKELS 30 // 30 different folders where murkels can be assigned to - should be increased when more folders are expected
#define SSPIN 10
#define RSTPIN 4
//#define INITMURKEL true
//#define NONINITMURKEL false

byte nuidStore[4];// Init array that will store new NUID
bool rfidTagPresentPrev = false;
bool rfidTagPresent = false;
int rfidErrorCounter = 0;
bool tagFound = false;

bool murkelLanded = false;
bool murkelUsageOn = true;

// Instantiate object

MFRC522 murkelReader(SSPIN, RSTPIN);
MFRC522::MIFARE_Key key;


// ************* DFPlayer Mini section **************

#include "DFRobotDFPlayerMini.h"

// Instantiate object
DFRobotDFPlayerMini murkelMp3Player;

#define MURKELPLAYING 400 // compare value to decide if signal is high or low - A7 did not seem to work as digital pin
#define MURKELIDLE 400  //  compare value to decide if signal is high or low - A7 did not seem to work as digital pin
#define STATSINTERLOOP 10  // queryevery stats Nth loop
#define MURKELBUSYPIN A7 // measure BUSY state of DFPlayer - LOW = Playing
#define AVOIDINITIALNOISE 1500 // delay to cover initial noise humm when switching on DFPLAYER
#define STARTSTOP 0
#define ENDPAUSE 1

uint8_t murkelAlarmFolder = 20;
uint8_t murkelAlarmTitle = 1;

uint8_t statsLoop = 0;

int8_t dfPlayerMaxVolume = 19;

uint8_t murkelVolume = 10;
uint8_t murkelTitle = 1; // current title to play
uint8_t murkelFolder = 1; // current folder to play from
uint8_t maxMurkelFolder = 1; // limit how many folders are assigned - needs to be read from eeprom, number is increased when new RFI is learnt
uint8_t maxMurkelTitle;

uint16_t murkelState, murkelCurrentFile; // intialized in function
bool murkelPlaySetting = true; // false = start from where murkel has been removed.

// ************* function prototypes ****************

void printDetail(uint8_t, int);
//void lcdFormatPrint(
void loadEeprom();
void saveEeprom(bool);


// *******************************************************************
// *************************** setup function ************************
// *******************************************************************

void setup() {

  // ************* General setup ***********

  murkelSerial.begin(9600);
  Serial.begin(HWSERIALSPEED); // intialize the serial interface for messages
  SPI.begin(); // Init SPI bus
  Wire.begin();  // Start the I2C interface

  // ************ LCD setup ************

  lcd.begin(16, 2);  // set up the LCD's number of columns and rows:


  lcd.clear();
  lcd.setCursor (0, 0);
  lcd.print(VERSION);
  //Serial.println("Initializing I2C devices...");       // initialize device


  // ************* Keypad setup *********

  // prepare keypad, pressed keys are setting the pin to LOW
  pinMode( KEYPLUS, INPUT);
  digitalWrite(KEYPLUS, HIGH);
  pinMode( KEYOK, INPUT);
  digitalWrite(KEYOK, HIGH);
  pinMode( KEYMINUS, INPUT);
  digitalWrite(KEYMINUS, HIGH);



  // ************ RFID setup ***********

  lcd.setCursor (0, 1);
  lcd.print("MFRC522         ");
  murkelReader.PCD_Init(); // Init MFRC522

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  lcd.setCursor (10, 1);
  lcd.print("  OK");
  delay(1000);




  // ************ setup IMU ************

  lcd.setCursor (0, 1);
  lcd.print("MPU6050         ");
  delay(1000);
  murkelGyro.initialize();

  //Serial.println("Testing device connections...");    // verify connection
  lcd.setCursor (10, 1);
  lcd.print(murkelGyro.testConnection() ? "  OK" : " failed!");
  delay(1000);

  // ************ setup DFPlayer ************

  //Serial.println(F("Initializing DFPlayer ..."));
  flushScreen(SECONDLINE);
  lcd.setCursor (0, 1);
  lcd.print("DFPlyr          ");
  if (!murkelMp3Player.begin(murkelSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("DFPlayer start failed"));
    //    while (true);   // use this as soon as HW is available.
    while (false);
  }
  //Serial.println(F("DFPlayer Mini online."));

  lcd.setCursor (10, 1);
  lcd.print("  OK");
  delay(1000);

  murkelMp3Player.setTimeOut(500); //Set serial communictaion time out 500ms

  murkelMp3Player.volume(murkelVolume);  //Set volume value (0~30).
  murkelMp3Player.EQ(DFPLAYER_EQ_NORMAL);
  murkelMp3Player.outputDevice(DFPLAYER_DEVICE_SD);

  murkelMp3Player.volume(15);
  murkelMp3Player.playFolder(99, 1);

  // ************** Powerbank load

  pinMode(POWERBANKLOADTRIGGER, OUTPUT);

  // ************ Final status message ************

  lcd.clear();

  loadPowerbank(true); // trigger load peak

  lcd.setCursor(0, 0);
  lcd.print("'OK' for default");

  //rkelMp3Player.volume(murkelVolume);
  //rkelQuiet(STARTSTOP);
  delay(2000);

  if (!digitalRead(MENUMODE)) {
    lcd.setCursor(0, 1);
    lcd.print("Dflt Prm set    ");
    maxMurkelFolder = EEPROM.read(MAXMURKELFOLDER);
    delay(DELAY2);
  }
  else {
    loadEeprom();
    maxMurkelFolder = EEPROM.read(MAXMURKELFOLDER);
  }
  lcd.setCursor(0, 0);
  lcd.print("Murkel Box ready");

  murkelStats(true);

  delay(17000);

  lcd.setCursor(6, 0);
  lcd.print("No Murkel ");

  lcd.setCursor(0, 1);
  lcd.print("                ");


}

// *******************************************************************
// *************************** main loop *****************************
// *******************************************************************


void loop() {

  loadPowerbank(false);  // trigger current spike to avoid powerbank switchoff, false means only trigger after certain time

  murkelControl(); // check for volume and tilt

  murkelDisplay(HOURMINUTE);// display clock or date

  gyroControl(); // readout ax,ay,az

  if (murkelUsageOn) checkForMurkel(true); // check for change of RFID tag, store RFID in array
  else murkelLanded = true;

  if (alarmActive) handleAlarm(); // check if alarm is set and handle

  murkelStats(false); // check busy pin and log if requested

  murkelPlay(); //

}



// *******************************************************************
// *********************** helper functions **************************
// *******************************************************************

void loadPowerbank(uint8_t mode)
{
  if ((millis() - previousMillisPower) > POWERINTERVALLMAX || mode)
  {
    previousMillisPower = millis();
    digitalWrite(POWERBANKLOADTRIGGER, HIGH);
    //     digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Triggering powerbank now...");
    delay(100);
    //     digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(POWERBANKLOADTRIGGER, LOW);
    Serial.println("powerbank load inactive...");
  }

}

// load/save parameters start from both sides ( EEPROM 0 for RFID tags and EEPROM (lenght) for parameters)


void  loadEeprom(/*bool mode*/) //
{
//  if (mode == INITMURKEL) { //Serial.println("EEPROM");
    murkelFolder = EEPROM.read(MURKELFOLDER);
    //Serial.println(murkelFolder);
    murkelTitle = EEPROM.read(RFIDBYTES * (murkelFolder - 1) + 5);
    //Serial.println(murkelTitle);
    murkelUsageOn = EEPROM.read(MURKELUSAGE);
    //Serial.println(murkelUsageOn);
    murkelVolume = EEPROM.read(MURKELVOLUME);
    //Serial.println(murkelVolume);
    alarmActive = EEPROM.read(ALARMACTIVE);
    //Serial.println(alarmActive);
    alarmHour = EEPROM.read(ALARMHOUR);
    //Serial.println(alarmHour);
    alarmMinute = EEPROM.read(ALARMMINUTE);
    //Serial.println(alarmMinute);
//  }
//  if (mode == NONINITMURKEL) maxMurkelFolder = EEPROM.read(MAXMURKELFOLDER); // defines the current available folders
  //Serial.println(maxMurkelFolder);

}


void saveEeprom(bool mode) //mode 0 - save only non volatile parameters, mode 1 save all parameters
{
  if (mode) {
    EEPROM.update( MURKELTITLE , murkelTitle - 1); //  reduced by one as murkelTitle is always increased after song started playing
    EEPROM.update( MURKELFOLDER , murkelFolder);
  }
  EEPROM.update( MURKELUSAGE , murkelUsageOn);
  EEPROM.update( MURKELVOLUME , murkelVolume);
  EEPROM.update( ALARMACTIVE , alarmActive);
  EEPROM.update( ALARMHOUR , alarmHour);
  EEPROM.update( ALARMMINUTE , alarmMinute);
  EEPROM.update( MAXMURKELFOLDER, maxMurkelFolder);
  //Serial.println("Parameters updated in EEPROM");
}


void saveRFID( uint8_t rfidPos, uint8_t rf1, uint8_t rf2 , uint8_t rf3, uint8_t rf4, uint8_t folder, uint8_t title) // very basic  - needs more sanity checks for submitted variables!
{
  EEPROM.write(rfidPos * RFIDBYTES, rf1);
  EEPROM.write(rfidPos * RFIDBYTES + 1, rf2);
  EEPROM.write(rfidPos * RFIDBYTES + 2, rf3);
  EEPROM.write(rfidPos * RFIDBYTES + 3, rf4);
  EEPROM.write(rfidPos * RFIDBYTES + 4, folder);
  EEPROM.write(rfidPos * RFIDBYTES + 5, title);
  //Serial.print(rfidPos * RFIDBYTES);Serial.print(" ");
  //Serial.print(rf1);Serial.print(" ");
  //Serial.print(rf2);Serial.print(" ");
  //Serial.print(rf3);Serial.print(" ");
  //Serial.print(rf4);Serial.print(" ");
  //Serial.print(folder);Serial.print(" ");
  //Serial.print(title);Serial.println();
}


// read gyro, later it needs to return ax, actions for volume or next song

void gyroControl()
{
  murkelGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //  lcd.setCursor(0, 1);
  //  sprintf(lineBuffer[1], "%03i", ax / 1000);
  //  lcd.print(lineBuffer[1]);
}

// play dfplayer murkel assignment

void murkelPlay()
{
  if (!murkelLanded && murkelUsageOn) { // check for logical error here -> need to stop in case no murkel on and murkel usage is switched on
    murkelMp3Player.stop();
    lcd.setCursor(6, 0);
    lcd.print("No Murkel ");
    return;
  }
  if (murkelLanded && murkelState > MURKELIDLE) { // check for logical error here -> need to play next song a murkel is on and no song is playing
    if (++murkelTitle > maxMurkelTitle) murkelTitle = 1;
    murkelFolderTitle(1);

  }
}



// function to play folder, title and update display

void murkelFolderTitle(uint8_t mode) // 0 - just show LCD, 1-  standard start song, 2 - continue after pause - should enable different functions like stop, pause, next, previous,... CHANGE TO SWITCH CASE STRUCTURE
{
  sprintf(lineBuffer[0], "o_o %02i/%02i", murkelFolder, murkelTitle );
  lcd.setCursor(6, 0);
  lcd.print(lineBuffer[0]);
  if (mode == 1) {
    murkelQuiet(STARTSTOP);
    murkelMp3Player.playFolder(murkelFolder, murkelTitle);
    delay(DELAY2);
    murkelStats(true);
  }
  //murkelStats(true);
}

uint8_t checkForMurkel(bool mode) // 0 - just load rfid into array, 1 - check if RFID id known
{
  // rfid tag check and detect if one is available

  rfidTagPresentPrev = rfidTagPresent;

  rfidErrorCounter += 1;
  if (rfidErrorCounter > 2) {
    tagFound = false;
  }

  // Detect Tag without looking for collisions
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);

  // Reset baud rates
  murkelReader.PCD_WriteRegister(murkelReader.TxModeReg, 0x00);
  murkelReader.PCD_WriteRegister(murkelReader.RxModeReg, 0x00);
  // Reset ModWidthReg
  murkelReader.PCD_WriteRegister(murkelReader.ModWidthReg, 0x26);

  MFRC522::StatusCode result = murkelReader.PICC_RequestA(bufferATQA, &bufferSize);

  if (result == murkelReader.STATUS_OK) {
    if ( ! murkelReader.PICC_ReadCardSerial()) { //Since a PICC placed get Serial and continue
      return;
    }
    rfidErrorCounter = 0;
    tagFound = true;
  }

  rfidTagPresent = tagFound;

  // rising edge
  if (rfidTagPresent && !rfidTagPresentPrev) {
    Serial.println("Tag found");
    // Store NUID into nuidStore array
    for (byte i = 0; i < 4; i++) {
      nuidStore[i] = murkelReader.uid.uidByte[i];  //remove and use murkelReader.uid array instead
      Serial.print(nuidStore[i]);
    }
    //Serial.println();
    uint8_t storeFolder = murkelFolder;
    if (mode) {
      for ( int i = 0; i < maxMurkelFolder; i++)
      {
        if (EEPROM.read(i * RFIDBYTES + 0) == nuidStore[0])
        {
          if (EEPROM.read(i * RFIDBYTES + 1) == nuidStore[1])
          {
            if (EEPROM.read(i * RFIDBYTES + 2) == nuidStore[2])
            {
              if (EEPROM.read(i * RFIDBYTES + 3) == nuidStore[3])
              {
                murkelFolder = EEPROM.read(i * RFIDBYTES + 4);
                if (!murkelPlaySetting || storeFolder == murkelFolder) murkelTitle = EEPROM.read(i * RFIDBYTES + 5);
                else murkelTitle = 1;
                maxMurkelTitle = murkelMp3Player.readFileCountsInFolder(murkelFolder);
                murkelLanded = true;
                murkelStats(true);
                //Serial.print("murkelLanded=");Serial.println(murkelLanded);
                return;
              }
              //else Serial.println("RFID not recognized");
            }
            //else Serial.println("RFID not recognized");
          }
          //else Serial.println("RFID not recognized");
        }
        //else Serial.println("RFID not recognized");
      }
    }
    Serial.println();
  }

  // falling edge
  if (!rfidTagPresent && rfidTagPresentPrev) {
    Serial.println("Tag gone");
    murkelLanded = false;
    murkelStats(false);
    Serial.print("murkelLanded="); Serial.println(murkelLanded);
    for (byte i = 0; i < 4; i++) {
      nuidStore[i] = 0xFF;
      Serial.print(nuidStore[i]);
    }
    Serial.println();
    EEPROM.update( RFIDBYTES * (murkelFolder - 1) + 5 , murkelTitle - 1); ////Serial.println(murkelFolder -1);//Serial.println(murkelTitle);  // reduced by one as murkelTitle is always increased right after song starts playing.
  }
  return murkelLanded;
}


void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    lcd.print(buffer[i] < 0x10 ? " 0" : " ");
    lcd.print(buffer[i], HEX);
  }
}

void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    lcd.print(buffer[i] < 0x10 ? " 0" : " ");
    lcd.print(buffer[i], DEC);
  }
}

// ************* Menu and conttrol section **************

void setupmenu()
{
  long timer = millis();
  uint16_t lastMenuMode = 0;
  uint16_t lastUp = 0;
  uint16_t lastDown = 0;
  uint8_t highlightMenuLevel = 0;
  uint8_t subMenuLevel = 0;
  uint8_t subSubMenuLevel = 0;
  uint8_t selectedMenu = 0;


  //Serial.print("Setup mode");Serial.println(highlightMenuLevel);
  lcd.setCursor(6, 0);
  lcd.print("    Setup ");

  long debounceTime = millis();
  long lastDebounceTime = 0;

  while (millis() < timer + MENUMODETIMEMILLIS)
  {
    murkelClockGetDate();//get the time data from tiny RTC

    // active (button pressesed) on LOW.
    uint16_t menuMode = !digitalRead(MENUMODE);
    uint16_t up = !digitalRead(MENUUP);
    uint16_t down = !digitalRead(MENUDOWN);

    loadPowerbank(false);

    if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && !selectedMenu)
    {
      if (++highlightMenuLevel > MAXMENUMODE) highlightMenuLevel = 0;
      flushScreen(1);
      timer = millis(); // reset menu end timer if key is pressed
      debounceTime = millis();
    }
    if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && !selectedMenu)
    {
      if (--highlightMenuLevel > MAXMENUMODE) highlightMenuLevel = MAXMENUMODE;
      flushScreen(1);
      timer = millis(); // reset menu end timer if key is pressed
      debounceTime = millis();
    }
    if (menuMode && menuMode != lastMenuMode && lastDebounceTime > DEBOUNCETIME && !selectedMenu)
    {
      selectedMenu = highlightMenuLevel;
      timer = millis(); // reset menu end timer if key is pressed
      debounceTime = millis();
    }

    switch (highlightMenuLevel)
    {


      case PAUSE: {
          int counter = 0;
          lcd.setCursor(0, 1);
          lcd.print("Pause/Play -> OK");
          if (menuMode && menuMode != lastMenuMode && lastDebounceTime > DEBOUNCETIME && selectedMenu == PAUSE) {
            murkelMp3Player.pause();
            delay(DELAY1);
            while (menuMode = digitalRead(MENUMODE) && counter++ < MAXPAUSELOOPS )
            {
              if (!(counter / DELAY1 % 2)) {
                lcd.setCursor(0, 1);
                lcd.print("Paused...       ");
                murkelDisplay(HOURMINUTE);
              }
              else {
                lcd.setCursor(0, 1);
                lcd.print("                ");
              }
              loadPowerbank(0);

            }
            //murkelMp3Player.start();
            //murkelLanded = false;
            murkelQuiet(ENDPAUSE);
          }
        }
        break;


      case SWITCHOFF: {  // not working yet - sleep does not seem to lower current significantly. Consumption is ~0,07 A - need to check if interrupt of Arduino is triggered by connected signals to int pins?
          saveEeprom(true); // save all relevant data before switching off
          lcd.setCursor(0, 1);
          lcd.print("Power off  -> OK");
          if (menuMode && menuMode != lastMenuMode && lastDebounceTime > DEBOUNCETIME && selectedMenu == SWITCHOFF) {
            lcd.setCursor(0, 1);
            lcd.print("Switching off... ");
            murkelMp3Player.volume(0);
            murkelMp3Player.sleep();
            Serial.print("Power off initiated");
            pinMode(2, INPUT);
            digitalWrite(2, LOW);

            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            sleep_enable();
            attachInterrupt(0, wakeUpNow, HIGH);
            sleep_mode();
            Serial.println("Sleeping...");
            while (true) ;//Serial.print("chrrrrrrrrrrrr             ");
          }
        }
        break;


      case ALARM: {
          lcd.setCursor(0, 1);
          lcd.print("Alarmmode ");
          lcd.print(alarmActive);
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == ALARM) {
            alarmActive = !alarmActive;
            timer = millis();
            debounceTime = millis();
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == ALARM) {
            alarmActive = !alarmActive;
            timer = millis();
            debounceTime = millis();
          }
        }
        break;


      case CLKMIN: {
          lcd.setCursor(0, 1);
          lcd.print("SetMin");
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKMIN) {
            //Serial.print("Minute increased.");
            if (++minute > 59) minute = 0;
            //Serial.println(minute);
            debounceTime = millis();
            timer = millis();
            murkelClockSetDate(hour, minute, murkelClock.getDate(), murkelClock.getMonth(Century), murkelClock.getYear());
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKMIN) {
            //Serial.print("Minute decreased.");
            if (--minute > 60) minute = 59;
            //Serial.println(minute);
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(hour, minute, murkelClock.getDate(), murkelClock.getMonth(Century), murkelClock.getYear());
          }
          murkelDisplay(HOURMINUTE);
        }
        break;


      case CLKHRS: {
          lcd.setCursor(0, 1);
          lcd.print("SetHrs");
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKHRS) {
            //Serial.print("Hour increased.");
            if (++hour > 23) hour = 0;
            //Serial.println(hour);
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(hour, minute, murkelClock.getDate(), murkelClock.getMonth(Century), murkelClock.getYear());
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKHRS) {
            //Serial.print("Hour decreased. ");
            if (--hour < 0) hour = 23;
            //Serial.println(hour);
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(hour, minute, murkelClock.getDate(), murkelClock.getMonth(Century), murkelClock.getYear());
          }
          murkelDisplay(HOURMINUTE);
        }
        break;
      case CLKALMIN: {
          lcd.setCursor(0, 1);
          lcd.print("SetAlMin");
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKALMIN) {
            //Serial.print("Minute increased.");
            if (++alarmMinute > 59) alarmMinute = 0;
            //Serial.println(alarmMinute);
            debounceTime = millis();
            timer = millis();
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKALMIN) {
            //Serial.print("Minute decreased.");
            if (--alarmMinute > 60) alarmMinute = 59;
            //Serial.println(alarmMinute);
            timer = millis();
            debounceTime = millis();
          }
          murkelDisplay(ALARMHOURMINUTE);
        }
        break;


      case CLKALHRS: {
          lcd.setCursor(0, 1);
          lcd.print("SetAlHrs");
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKALHRS) {
            //Serial.print("Hour increased.");
            if (++alarmHour > 23) alarmHour = 0;
            //Serial.println(alarmHour);
            timer = millis();
            debounceTime = millis();
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKALHRS) {
            //Serial.print("alarmHour decreased. ");
            if (--alarmHour < 0) alarmHour = 23;
            //Serial.println(alarmHour);
            timer = millis();
            debounceTime = millis();
          }
          murkelDisplay(ALARMHOURMINUTE);
        }
        break;


      case CLKDAY: {
          lcd.setCursor(0, 1);
          lcd.print("SetDay");
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKDAY) {
            //Serial.print("Day increased.");
            if (++dayOfMonth > monthDays[murkelClock.getMonth(Century)]) dayOfMonth = 1;
            //Serial.println(dayOfMonth);
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(murkelClock.getHour(h12, PM), murkelClock.getMinute(), dayOfMonth, murkelClock.getMonth(Century), murkelClock.getYear());
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKDAY) {
            //Serial.print("Day decreased. ");
            if (--dayOfMonth  < 1 ) dayOfMonth = monthDays[murkelClock.getMonth(Century)];
            //Serial.println(dayOfMonth);
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(murkelClock.getHour(h12, PM), murkelClock.getMinute(), dayOfMonth, murkelClock.getMonth(Century), murkelClock.getYear());
          }
          murkelDisplay(DAYMONTHYEAR);;
        }
        break;


      case CLKMON: {   // set month
          lcd.setCursor(0, 1);
          lcd.print("SetMnth");
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKMON) {
            //Serial.print("Clockmode: ");
            if (++month > 12) month = 1;
            //Serial.println(month);
            //                  EEPROM.write(STATUSCLOCKMODE, learnMode);
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(murkelClock.getHour(h12, PM), murkelClock.getMinute(), murkelClock.getDate(), month, murkelClock.getYear());
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKMON) {
            //Serial.print("ClockMode: ");
            if (--month < 1 ) month = 12;
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(murkelClock.getHour(h12, PM), murkelClock.getMinute(), murkelClock.getDate(), month, murkelClock.getYear());
          }
          murkelDisplay(DAYMONTHYEAR);;
        }
        break;


      case CLKYR: {   // set year
          lcd.setCursor(0, 1);
          lcd.print("SetYear");
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKYR) {
            if (++year > 99) year = 0;
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(murkelClock.getHour(h12, PM), murkelClock.getMinute(), murkelClock.getDate(), murkelClock.getMonth(Century), year);
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == CLKYR) {
            if (--year < 0 ) year = 99;
            timer = millis();
            debounceTime = millis();
            murkelClockSetDate(murkelClock.getHour(h12, PM), murkelClock.getMinute(), murkelClock.getDate(), murkelClock.getMonth(Century), year);
          }
          murkelDisplay(DAYMONTHYEAR);;
        }
        break;


      case LEARN: {
          lcd.setCursor(0, 1);
          lcd.print("Learn new RFID   ");
          if (menuMode && menuMode != lastMenuMode && lastDebounceTime > DEBOUNCETIME && selectedMenu == LEARN)
          {
            flushScreen(0);
            lcd.setCursor(0, 0);
            lcd.print("Mount RFID &    ");
            lcd.setCursor(0, 1);
            lcd.print("Press 'UP'      ");
            while (digitalRead(MENUUP)) loadPowerbank(false); // only triggger when timer is up
            checkForMurkel(false);
            saveRFID( (maxMurkelFolder - 1), nuidStore[0], nuidStore[1], nuidStore[2], nuidStore[3],  maxMurkelFolder, 1);
            lcd.setCursor(0, 0);
            sprintf(lineBuffer[0], "Fldr %02i ->     ", maxMurkelFolder);
            if (++maxMurkelFolder > MAXMURKELS) maxMurkelFolder = MAXMURKELS ;
            drawScreen(0);
            lcd.setCursor(0, 1);
            for (int count = 0; count < 4; count ++) {
              lcd.print( nuidStore[count]);
            }
            lcd.print("     ");
            delay(5 * DELAY1);
            lcd.setCursor(0, 0);
            lcd.print("RFID stored ...");
            saveEeprom(false);
            delay(DELAY2);

          }
        }
        break;


      case PRMMURKELUSAGE: {
          lcd.setCursor(0, 1);
          lcd.print("Use Murkels ");
          lcd.print(murkelUsageOn);
          if (up && up != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == PRMMURKELUSAGE) {
            murkelUsageOn = !murkelUsageOn;
            rfidTagPresent = ! rfidTagPresent;
            timer = millis();
            debounceTime = millis();
          }
          if (down && down != lastDown && lastDebounceTime > DEBOUNCETIME && selectedMenu == PRMMURKELUSAGE) {
            murkelUsageOn = !murkelUsageOn;
            rfidTagPresent = ! rfidTagPresent;
            timer = millis();
            debounceTime = millis();
          }
        }
        break;


      case LEARNINIT: {
          lcd.setCursor(0, 1);
          lcd.print("Init RFID ->'OK'");
          if (menuMode && menuMode != lastUp && lastDebounceTime > DEBOUNCETIME && selectedMenu == LEARNINIT) {
            for (int count = 0; count < MAXMURKELS; count ++) {
              EEPROM.update (RFIDBYTES * count, 255);
              EEPROM.update (RFIDBYTES * count + 1, 255);
              EEPROM.update (RFIDBYTES * count + 2, 255);
              EEPROM.update (RFIDBYTES * count + 3, 255);
              EEPROM.update (RFIDBYTES * count + 4, 255);
              EEPROM.update (RFIDBYTES * count + 5, 255);
            }
            maxMurkelFolder = 1; // maxMurkelFolder represents current position to save new RFID on
            lcd.setCursor(0, 1);
            lcd.print("Initialized ....");
            delay(2000);
            timer = millis();
            debounceTime = millis();
          }
        }
        break;


      case PRMLOAD: {
          lcd.setCursor(0, 1);
          lcd.print("Ld Prm -> 'OK'");
          if (menuMode && menuMode != lastMenuMode && lastDebounceTime > DEBOUNCETIME && selectedMenu == PRMLOAD) {
            loadEeprom();
            lcd.setCursor(0, 1);
            lcd.print("Loaded...       ");
            murkelDisplay(0);
            delay(DELAY2); delay(DELAY2);
          }
        }
        break;


      case PRMSAVE: {
          lcd.setCursor(0, 1);
          lcd.print("Sve Prm -> 'OK'");
          if (menuMode && menuMode != lastMenuMode && lastDebounceTime > DEBOUNCETIME && selectedMenu == PRMSAVE) {
            saveEeprom(true);
            lcd.setCursor(0, 1);
            lcd.print("Saved...       ");
            murkelDisplay(0);
            delay(DELAY2); delay(DELAY2);
          }
        }
        break;


      default: {}


    }
    lastMenuMode = menuMode;
    lastUp = up;
    lastDown = down;
    lastDebounceTime = millis() - debounceTime;
  }
  flushScreen(2);
  murkelFolderTitle(0);
  saveEeprom(true);
}

// ************ Display handling ***********

void flushScreen(int mode)
{
  sprintf(lineBuffer[0], "                ");
  sprintf(lineBuffer[1], "                ");
  drawScreen(mode);
}

void drawScreen(int mode) // mode =0 draw only first line, mode = 1 draw only 2nd line, 2 draw both lines
{
  if (mode == FIRSTLINE || mode == BOTHLINES) {
    lcd.setCursor(0, 0);
    lcd.print(lineBuffer[0]);
  }
  if (mode == SECONDLINE || mode == BOTHLINES) {
    lcd.setCursor(0, 1);
    lcd.print(lineBuffer[1]);
  }
}
void murkelDisplay(uint8_t mode)
{
  lcd.setCursor(0, 0);
  //  sprintf(lineBuffer[0],"%02i:%02i:%02i ",murkelClock.getHour(h12, PM), murkelClock.getMinute(), murkelClock.getSecond());
  switch (mode)
  {
    case HOURMINUTE:
      {
        sprintf(lineBuffer[0], "%02i:%02i ", murkelClock.getHour(h12, PM), murkelClock.getMinute());
      }
      break;
    case DAYMONTHYEAR:
      {
        sprintf(lineBuffer[0], "%02i.%02i.%02i ", murkelClock.getDate(), murkelClock.getMonth(Century), murkelClock.getYear());
      }
      break;
    case ALARMHOURMINUTE:
      {
        sprintf(lineBuffer[0], "%02i:%02i    ", alarmHour, alarmMinute);
      }
      break;
    default: {};
  }

  lcd.print(lineBuffer[0]);
}

// ************* DFPlayer control ***********

void murkelControl()
{
  if (!digitalRead(KEYOK)) setupmenu();

  if (!digitalRead(KEYMINUS)) {
    if (--murkelVolume > dfPlayerMaxVolume) murkelVolume = 0; // somehow sign does not work 0-1 =255 ...
    sprintf(lineBuffer[1], "Volume %02i", murkelVolume);
    murkelMp3Player.volume(murkelVolume);
    drawScreen(SECONDLINE);
  }

  if (!digitalRead(KEYPLUS)) {
    if (++murkelVolume > dfPlayerMaxVolume) murkelVolume = dfPlayerMaxVolume;
    sprintf(lineBuffer[1], "Volume %02i", murkelVolume);
    murkelMp3Player.volume(murkelVolume);
    drawScreen(SECONDLINE);
  }

  if (ax / 1000 < MAXRIGHTTILT && murkelLanded)
  {
    if (++murkelTitle > maxMurkelTitle) murkelTitle = 1;
    murkelFolderTitle(1);
    murkelStats(true);
    delay(DELAY2);
  }

  if (ax / 1000 > MAXLEFTTILT && murkelLanded)
  {
    if (--murkelTitle < 1) murkelTitle = maxMurkelTitle;
    murkelFolderTitle(1);
    murkelStats(true);
    delay(DELAY2);
  }
  if (ay / 1000 > MAXBACKTILT && murkelLanded && !murkelUsageOn)
  {
    if (++murkelFolder >= MAXMURKELS ) murkelFolder = 1;

    murkelTitle = 1;
    murkelFolderTitle(1);
    murkelStats(true);
    delay(DELAY1);
  }

  if (ay / 1000 < MAXFORETILT && murkelLanded && !murkelUsageOn)
  {
    if (--murkelFolder < 1) murkelFolder = MAXMURKELS; 

    murkelTitle = 1;
    murkelFolderTitle(1);
    murkelStats(true);
    delay(DELAY1);
  }
}

// ************* RTC handling **************

// Function to set the current time, change the second&minute&hour to the right time
void murkelClockSetDate(byte hour, byte minute, byte day, byte month, byte year)
{
  murkelClock.setYear(year);
  murkelClock.setMonth(month);
  murkelClock.setDate(day);
  //    murkelClock.setDoW(DoW);
  murkelClock.setHour(hour);
  murkelClock.setMinute(minute);
  //    murkelClock.setSecond(Second);
}
// Function to gets the date and time from the ds1307 and prints result
void murkelClockGetDate()
{
  year = murkelClock.getYear();
  month = murkelClock.getMonth(Century);
  hour = murkelClock.getHour(h12, PM); //24-hr
  minute = murkelClock.getMinute();
  second = murkelClock.getSecond();
  dayOfMonth = murkelClock.getDate();
}

// *************** noise avoidance ********************
void murkelQuiet(bool mode) // mode=0 play arbitary song and stop again after delay, mode=1, continue with song on pause
{
  murkelMp3Player.volume(0);
  if (!mode) murkelMp3Player.playFolder(1, 1);
  else  murkelMp3Player.start();
  delay(AVOIDINITIALNOISE); // no clue why there is a noise when play starts first time...
  if (!mode) murkelMp3Player.stop();
  murkelMp3Player.volume(murkelVolume);
}

// *************** funtion to handle alarm ***************************

void handleAlarm()
{
  murkelClockGetDate();
  long counter = 0;
  if (hour == alarmHour && minute == alarmMinute)
  {
    uint8_t storeFolder = murkelFolder;
    uint8_t storeTitle = murkelTitle;
    murkelFolder = murkelAlarmFolder;
    murkelTitle = murkelAlarmTitle;
    lcd.setCursor(0, 1);
    lcd.print("Alarm. 'OK'");
    murkelQuiet(STARTSTOP);
    murkelFolderTitle(1);
    while (digitalRead(MENUMODE) ) {
      loadPowerbank(false);
      murkelDisplay(HOURMINUTE);
    }
    murkelFolder = storeFolder;
    murkelTitle = storeTitle;
  }
}

// *************** function for powering down arduino *****************

// *************** read and print dfplayer stats *********************

void murkelStats(bool logging)
{
  murkelState = analogRead(MURKELBUSYPIN);
  if (logging) {
    murkelCurrentFile = murkelMp3Player.readCurrentFileNumber();
    Serial.println("Reading MP3 stats");
    Serial.print("Busystate: "); Serial.println(murkelState);
    Serial.print("DFPlayer State:");
    Serial.println( murkelMp3Player.readState()); //read mp3 state
    Serial.print("DFPlayer Current File ");
    Serial.print(murkelTitle); Serial.print(" / total ");
    Serial.println(murkelCurrentFile);
    Serial.println(maxMurkelFolder);
  }
}

// ******************* should never be called as sleep mode should trigger powerbank shutdown due to low current consumption

void wakeUpNow()        // here the interrupt is handled after wakeup
{
  flushScreen(0);
  lcd.print("WOKE UP");
}
