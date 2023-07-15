/*
    Title:    Cryologger - Glacier Velocity Tracker (GVT)
    Version:  2.4.0
    Date:     July 15, 2023
    Author:   Adam Garbo

    Components:
    - SparkFun Artemis Processor
    - SparkFun MicroMod Data Logging Carrier Board
    - SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic)
    - SparkFun Qwiic OLED Display
    - Pololu 5V 600mA Step-Down Voltage Regulator D36V6F5

    Dependencies:
    - Apollo3 Core v1.2.3
    - SparkFun u-blox GNSS v3 v3.0.16
    - SparkFun Qwiic OLED Arduino Library v1.0.5
    - SdFat v2.2.2
*/

// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
#include <RTC.h>
#include <SdFat.h>                    // http://librarymanager/All#SdFat
#include <SparkFun_Qwiic_OLED.h>      // http://librarymanager/All#SparkFun_Qwiic_OLED_Arduino_Library
#include <SparkFun_u-blox_GNSS_v3.h>  // http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include <SPI.h>
#include <WDT.h>
#include <Wire.h>

// ----------------------------------------------------------------------------
// Define hardware and software versions
// ----------------------------------------------------------------------------
#define HARDWARE_VERSION  "2.2.1"
#define SOFTWARE_VERSION  "2.4.0"

// ----------------------------------------------------------------------------
// Define unique identifier
// ----------------------------------------------------------------------------
char UID[5] = "TST";

// -----------------------------------------------------------------------------
// Debugging macros
// -----------------------------------------------------------------------------
#define DEBUG       true  // Output debug messages to Serial Monitor
#define DEBUG_GNSS  true  // Output GNSS information to Serial Monitor
#define OLED        true  // Output messages to OLED display

#if DEBUG
#define DEBUG_PRINT(x)            Serial.print(x)
#define DEBUG_PRINTLN(x)          Serial.println(x)
#define DEBUG_PRINT_DEC(x, y)     Serial.print(x, y)
#define DEBUG_PRINTLN_DEC(x, y)   Serial.println(x, y)
#define DEBUG_WRITE(x)            Serial.write(x)

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_DEC(x, y)
#define DEBUG_PRINTLN_DEC(x, y)
#define DEBUG_WRITE(x)
#endif

// ----------------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------------
#define PIN_MICROSD_POWER   33  // G1
#define PIN_QWIIC_POWER     34  // G2 
#define PIN_SD_CS           41  // CS

// ----------------------------------------------------------------------------
// Object instantiations
// ----------------------------------------------------------------------------
APM3_RTC          rtc;
APM3_WDT          wdt;
SdFs              sd;
FsFile            logFile;
FsFile            debugFile;
QwiicNarrowOLED   oled;       // I2C address: 0x3C
SFE_UBLOX_GNSS    gnss;       // I2C address: 0x42

// ----------------------------------------------------------------------------
// User defined logging/sleeping variables
// ----------------------------------------------------------------------------

// Logging operation modes
// 1: Daily (e.g., 3 hours each day between 16:00-19:00)
// 2: Rolling (e.g., 1 hour awake, 3 hours asleep, repeat)
// 3: Continuous (e.g., constant logging with new log file created every day at 00:00 UTC)
byte          operationMode       = 3;        // 1: daily, 2: rolling, 3: 24-hour/day

// 1: Daily alarm configuration
byte          alarmStartHour      = 16;       // Daily logging start hour (UTC)
byte          alarmStartMinute    = 0;        // Daily logging start minute (UTC)
byte          alarmStopHour       = 19;       // Daily logging end hour (UTC)
byte          alarmStopMinute     = 0;        // Daily logging end minute (UTC)

// 2: Rolling alarm configuration
byte          alarmAwakeHours     = 3;        // Rolling hour alarm
byte          alarmAwakeMinutes   = 0;        // Rolling minute alarm
byte          alarmSleepHours     = 0;        // Rolling hour alarm
byte          alarmSleepMinutes   = 1;        // Rolling minute alarm

// ----------------------------------------------------------------------------
// Global variable declarations
// ----------------------------------------------------------------------------
volatile bool alarmFlag           = false;    // Flag for alarm interrupt service routine
volatile bool wdtFlag             = false;    // Flag for WDT interrupt service routine
volatile int  wdtCounter          = 0;        // Counter for WDT interrupts
volatile int  counterWdtMax       = 0;        // Counter for max WDT interrupts
bool          gnssConfigFlag      = true;     // Flag to indicate whether to configure the u-blox module
bool          flagRtcSync         = false;    // Flag to indicate if RTC has been synced with GNSS
bool          firstTimeFlag       = true;     // Flag to indicate if program running for the first time
byte          alarmModeInitial    = 4;        // Default initial alarm mode (daily)
byte          alarmModeLogging    = 4;        // Default logging alarm mode (daily)
byte          alarmModeSleep      = 4;        // Default sleep alarm mode (daily)
byte          dateCurrent         = 0;        // Variable for tracking when the date changes
byte          dateNew             = 0;        // Variable for tracking when the date changes
char          logFileName[30]     = "";       // Log file name
char          debugFileName[20]   = "";       // Debug log file name
char          dateTimeBuffer[25]  = "";       // Buffer to store datetime information
const int     sdWriteSize         = 512;      // Write data to SD in blocks of 512 bytes
const int     fileBufferSize      = 16384;    // Buffer size to allocate 16 KB RAM for UBX message storage
unsigned int  counterDebug        = 0;        // Counter to track number of recorded debug messages
unsigned int  gnssTimeout         = 300;      // Timeout for GNSS signal acquisition (seconds)
unsigned int  maxBufferBytes      = 0;        // Maximum file buffer size
unsigned int  reading             = 0;        // Battery voltage analog reading
unsigned int  counterFix          = 0;        // GNSS fix counter
unsigned long previousMillis      = 0;        // millis() timer
unsigned long bytesWritten        = 0;        // Counter for tracking bytes written to microSD
unsigned long counterSdSyncFail     = 0;        // microSD logfile synchronize failure counter
unsigned long counterSdWriteFail    = 0;        // microSD logfile write failure counter
unsigned long counterSdCloseFail    = 0;        // microSD logfile close failure counter
unsigned long logStartTime        = 0;        // Counter to track elapsed logging duration
unsigned long counterSfrbx        = 0;        // Counter to track number of received SFRBX message groups
unsigned long counterRawx         = 0;        // Counter to track number of received RAWX message groups
long          rtcDrift            = 0;        // Counter for RTC drift

// ----------------------------------------------------------------------------
// Unions/structures
// ----------------------------------------------------------------------------

// Union to store online/offline states
struct struct_online
{
  bool microSd  = false;
  bool gnss     = false;
  bool oled     = false;
  bool logGnss  = false;
  bool logDebug = false;
} online;

// Union to store loop timers
struct struct_timer
{
  unsigned long wdt;
  unsigned long rtc;
  unsigned long microSd;
  unsigned long voltage;
  unsigned long gnss;
  unsigned long syncRtc;
  unsigned long logDebug;
  unsigned long logGnss;
} timer;

void newSfrbx(UBX_RXM_SFRBX_data_t *ubxDataStruct)
{
  counterSfrbx++; // Increment the count
}

void newRawx(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  counterRawx++; // Increment the count
}

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
  // Pin assignments
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Enable power to Qwiic connector
  qwiicPowerOn();

  // Enable power to peripherials
  peripheralPowerOn();

  Wire.begin();             // Initalize I2C
  Wire.setClock(400000);    // Set I2C clock speed to 400 kHz
  SPI.begin();              // Initialize SPI
  analogReadResolution(14); // Set ADC resolution to 14-bits

#if DEBUG
  Serial.begin(115200);   // Open Serial port
  //while (!Serial);        // Wait for user to open Serial Monitor
  blinkLed(2, 1000);      // Delay to allow user to open Serial Monitor
#endif

  // Configure real-time clock (RTC)
  configureRtc();

  // Configure OLED display
  configureOled();

  printLine();
  DEBUG_PRINTLN("Cryologger - Glacier Velocity Tracker (GVT)");
  printLine();
  DEBUG_PRINT("Unique Identifier:");  printTab(1);  DEBUG_PRINTLN(UID);
  DEBUG_PRINT("Software Version:");   printTab(1);  DEBUG_PRINTLN(SOFTWARE_VERSION);
  DEBUG_PRINT("Hardware Version:");   printTab(1);  DEBUG_PRINTLN(HARDWARE_VERSION);
  DEBUG_PRINT("Datetime:");           printTab(2);  printDateTime();
  DEBUG_PRINT("Battery Voltage:");    printTab(1);  DEBUG_PRINTLN(readVoltage());

  // Display OLED messages(s)
  displayWelcome();

  // Display logging configuration
  printLoggingSettings();
  displayLoggingMode();

  // Configure devices
  DEBUG_PRINTLN("Device Configuration");
  printLine();
  configureWdt();     // Configure and start Watchdog Timer (WDT)
  configureSd();      // Configure microSD
  configureGnss();    // Configure u-blox GNSS receiver
  syncRtc();          // Acquire GNSS fix and sync RTC with GNSS
  checkDate();        // Get current day of month
  createDebugFile();  // Create debug log file
  setInitialAlarm();  // Configure RTC and set initial alarm

  DEBUG_PRINT("Info - Datetime: "); printDateTime();
  DEBUG_PRINT("Info - Initial alarm: "); printAlarm();

  // Indicate completion of setup
  displaySetupComplete();
}

// ----------------------------------------------------------------------------
// Loop
// ----------------------------------------------------------------------------
void loop()
{
  // Check if alarm flag is set
  if (alarmFlag)
  {
    DEBUG_PRINT("Info - Alarm trigger: "); printDateTime();

    // Configure logging
    readRtc();        // Get the RTC's alarm date and time
    setAwakeAlarm();  // Set alarm to stop logging and go to sleep
    getLogFileName(); // Get timestamped log file name

    // Check opeation mode
    if (operationMode != 3) // Skip if in continuous mode
    {
      // Configure devices
      qwiicPowerOn();       // Enable power to Qwiic connector
      peripheralPowerOn();  // Enable power to peripherals
      resetOled();          // Configure OLED display
      configureSd();        // Configure microSD
      configureGnss();      // Configure u-blox GNSS
    }

    // Synchronize RTC only if day rollover has occurred
    checkDate();
    if (dateCurrent != dateNew)
    {
      DEBUG_PRINTLN(F("Info - Daily RTC sync required..."));
      syncRtc();
      dateCurrent = dateNew;
      checkDate();
    }
    else
    {
      DEBUG_PRINTLN(F("Info - RTC sync not required."));
    }

    // Log data
    logGnss();        // Log raw data from u-blox GNSS receiver
    logDebug();       // Log system debug information
    setSleepAlarm();  // Set alarm to wake from sleep and start logging
    printTimers();    // Display global function timers
    clearTimers();    // Clear global function timers
  }

  // Check for watchdog interrupt
  if (wdtFlag)
  {
    petDog(); // Restart WDT
  }

  // Blink LED
  blinkLed(1, 100);

  // Enter deep sleep
  goToSleep();
}

// ----------------------------------------------------------------------------
// Interupt Service Routines (ISR)
// ----------------------------------------------------------------------------

// Interrupt handler for the RTC
extern "C" void am_rtc_isr(void)
{
  // Clear the RTC alarm interrupt
  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

  // Set alarm flag
  alarmFlag = true;
}

// Interrupt handler for the WDT
extern "C" void am_watchdog_isr(void)
{
  // Clear the WDT interrupt
  wdt.clear();

  // Perform system reset after 10 WDT interrupts (should not occur)
  if (wdtCounter < 10)
  {
    wdt.restart(); // Restart the WDT timer
  }

  // Set the WDT flag
  wdtFlag = true;

  // Increment WDT interrupt counter
  wdtCounter++;

  if (wdtCounter > counterWdtMax)
  {
    counterWdtMax = wdtCounter;
  }
}
