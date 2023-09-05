// GNZDA_to_UTC, from the NMEA message "$GNZDA,Time,Date" send the message "$UTC,Date,Time" to simulate this non-standard Applanix "tm" message.
//
// Promoter   : Raphaël Mabit 
// Author     : Bruno Dionne 2023-06-04
// Motivation :	A commercial closer source data logger with 8 serial ports refuses to work until the non-standard GPS message "$UTC,<date>,<time>" is received at regular intervals.
// 				This Arduino Sketch transparently inserts the $UTC message to the other NMEA messages of a generic GPS here the U-BLox ZED-F9R from Spakfun.com
// 				The $UTC message is constructed from the $GNZDA message for the date and $GNRMC for the time.
// 				The program takes into account the time of reception, transmission and processing of NMEA messages and adds this correction to the UTC time of the $UTC message
// 				The UTC time on the $UTC message when it is received by the external data logger is therefore within ± 0.5 mS of the real transmission time of the PPS signal send by the GPS.
// 				It is therefore reasonable to consider the $UTC message as an excellent approximation of the PPS signal when it is not possible to have access to the original PPS signal.
//
// Version  : 2023-05-04, 05h04
// Remarks	: The of millis() counter rollover after ~49 days and unusually longer data delays are not processed. An arbitrary fixed correction is put to UTC until this situation resolve. 
// 
// Suggested connexion *****
//
//  U-Blox ZED-F9R <--uart1--> Artemis Thing Plus   ------------------> DataLogger.
//  U-Blox ZED-F9R <--QWIIC--> Artemis Thing Plus  
//  U-Blox ZED-F9R <- uart2--> Serial to Bluetooh  <------------------> Ordinateur, Tablette ou Mobile
//
//  GPS TX UART1  ----> RX Serial_1 (Artemis thing plus) TX Serial_1 ---> RX DataLogger.   // Yes the same serial port is used for both devices.
//
//  TX/RX UART2    <---> Module Bluetooth 4.0 SPP     <---> SPP Mobile/Tablet.
//  i2c QWIIC      <---> i2c QWIIC Artemis thing plus <---> Librairy "SparkFun_u-blox_GNSS_v3"
//
//  Artemis thing plus notes ***** 
//
//  Serial  is the UART port connected to USB.
//  Serial1 us the UART connected to pins TX and RX.
//  The enbedded Bluetooth BLE is not used for now.
//
// General operation :
// 
// Time stamp the PPS signal detection from the GPS.
// Read the NMEA messages which come in batch following the GPS PPS signal.
// Each character received is immediately forwarded without further processing to the data logger (maximum speed, minimum latency).
// Note the UTC time from the message $GNRMC (usually the first message send after the PPS sinal).
// Note the UTC date from the $GNZDA message (usually the last message send after the PPS signal).
// Upon complete receipt of the $GNZDA message, construct and send the $UTC message with time correction for processing times.
// The program is optimized for NMEA positioning messages at 1 Hz only. If you want to have more than 1 Hz you have to modify the program.
//
// Future development :
//
// Synchronize the RTC real time clock of the Artemis thing plus with the UTC time for validation and verification purposes (GPS time anti spoofing).
// Add a function to the Artemis Thing Plus user button for additional functionality (Ex: Start/Stop or with or without $UTC, etc. ).
// Add an interrupt to capture alarms from the Artemis Thing Plus RTC real time clock for additional functionality.
// Add external probes to support more NMEA message types (Magnetometer, Temperature, Humidity, Salinity, Pressure, etc.).
//
// ***** Include  *****
//
//
#include <Arduino.h>          // Use for Arduino
#include <SPI.h>              // used for SPI communication.
#include <Wire.h>             // Used for serial communication over USB //Needed for I2C to GNSS.
//#include "RTC.h"            // Real time clock. The RTC library included with the Arduino_Apollo3 core (not used for now).
//#include "WDT.h"            // Watchdog library (Not used for now).
//#include <ArduinoBLE.h>     // Integrated Bluetooth BLE of Artemis thing plus.(Not used for now).
//
//
// ***** U-blox Sparkfun librairie en option ***** (Not used for now).
//
//
//#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
//SFE_UBLOX_GNSS myGNSS;
//
//#define myWire Wire1 // Connect using the Wire1 port. Change this if required
//#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required
//
//
// ***** Define *****
//
//
// ***** Constant *****
//
//
// Usualy Arduino SBC define this symbol 'LED_BUILTIN'
// and it's associated with pin pin_size_t for the on board blue LED.
//
const pin_size_t ledPinNumber = LED_BUILTIN;  //LED by pin number
//
// Mbed boards use a PinName called LED1 to indicate the built-in LED
//
const PinName ledPinName = LED1;   // By name
//
const int ppsIntPin        =  5;   // Pin number for PPS interrupt detection.
const int spiChipSelectPin =  9;   // Pin number for SPI chip select.
const int buttonIntPin     = 10;   // Pin number for user botton. 
const int resetPin         =  6;   // Pin number for GPS U-Blox ZED-F9R RESET pin (not used for now).
const int transmitReadyPin =  7;   // Pin number for  GPS U-Blox ZED-F9R  TXR pin, Tansmit Ready / Buffer full (not used for now).
//
//
// ***** Variable  *****
//
//
// ***** Variable for interrupt ISR
//
volatile bool alarmISR_         = false; // Real time clock alarm ISR.
volatile bool buttonISR_        = false; // User press button ISR.
volatile bool ppsISR_           = false; // PPS signal ISR.
volatile bool TransmitReadyISR_ = false; // TXR ISR.
volatile unsigned long horodatePPS = 0;  // Timestamp arrival for PPS sigal.
//
volatile bool watchdogFlag = false;      // Watchdog Timer ISR flag
volatile int watchdogInterrupt = 0;      // Watchdog interrupt counter
//
//
// ***** autres variables 
//
// ***** Object  *****
//
//
///*********************************************************/
//
//
// ***** Routines *****
//
//
// Interript service routines
//
void ppsISR()                                    // Timestam for PPS signal.
{
  horodatePPS = millis();
  ppsISR_ = true;
}
//
void ButtonISR()                                 // User press button ISR.
{
  buttonISR_ = true;
}
//
//
void TransmitReadyISR()                         // Transit Ready TXR (buffer full) ISR.
{
  TransmitReadyISR_ = true; 
}
//
//
//extern "C" void am_rtc_isr(void)              // Real time clock alarm ISR as for datasheet precision is ± 4 microsecondes.
//{ 
//  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);   // Clear RTC alarm interrupt
//  alarmISR_ = true;  
//}
//
//***************************************************************/
// ***** Setup(), Called once on board POWER UP or RESET *****/
//***************************************************************/
//
void setup() {        // At startup all pins pulled low. 
  byte tempoByte = 0;
  Serial.begin( 115200 );                                             // Start Serial USB.
  while ( !Serial );                                                  // Wait until ready (some board ony).
  Serial.print( "Serial_0-USB( OK ), " );                             // Show USB Serial OK 
  Serial1.begin( 115200 );                                            // Start UART serial Serial1(TX-RX)"
  while ( !Serial1 );                                                 // Wait until ready (some board ony).
  Serial.print( "Serial_1-UART( OK ), " );                            // Show UART Serial OK 
  // 
  pinMode(LED_BUILTIN, OUTPUT);                                       // LED oFF  
  pinMode( ppsIntPin, INPUT );                                        // set the IRQ pin as an input pin. do not use INPUT_PULLUP - the ZED-F9R will pull the pin.
  attachInterrupt(digitalPinToInterrupt(ppsIntPin), ppsISR, RISING);  // Le ZED-F9R will pull the interrupt pin HIGH when a PPS event is triggered.
  Serial.print("PPS_Interrupt( OK ), " );                             // Show PPS ISR OK
  pinMode( transmitReadyPin, INPUT );                                 // set the TXR pin as an input pin. do not use INPUT_PULLUP - the ZED-F9R will pull the pin.
  while( Serial1.available()){ tempoByte = Serial1.read();}           // Clear all pending characters from Serial1 for a cleaner start.
  Serial.println("Pre_Flush_RX( OK ).");                              // Show clear pending char OK
}
//
//
//******************************************************************************************
// !!!!! After "setup()", "loop()" will loop forever !!!!!
//******************************************************************************************
//
//
void loop() { // This loop() maybe too slow with Arduino low frequency board < 16 Mhz. Test carefully before use. Sparkfun Artemis thing plus board run at 48 MHz.
  // static = La valeur de la variable est conservée entre chaque itération de loop(). Sinon c'est effacé.
  byte inByte =0;                                 // Read serial data as byte
  static byte indexMsg = 0;                       // NMEA message Input buffer index for next byte.
  const byte numChars = 255;                      // Size of NMEA mesage input buffer. 
  static char receivedChars[numChars];            // NMEA memory buffer.
  static char utcDate[9];                         // Extracted Date from $GNZDA message inluding the comma    AAAAMMJJ,
  static char utcTime[7];                         // Extracted time from $GNRMC message incliding the dot     HHMMSS.
  static bool debutNMEA = false;                  // Flag for start of NMEA message detection '$'
  static bool finNMEA = false;                    // Flag for end of NMEA message detection '\n'.
  static unsigned long DeltaUTC = 0;              // Latency UTC time correction for $UTC message.  Expect a time precision of ± 0.5 mS on the dot "."
  //
  //  
  if (ppsISR_ == true){                           // PPS arrival.
    //horodatePPS = millis();                     // Set in the interrupt ISR routine for best precision.
    Serial.write( "$UTC,");                       // $UTC,  Name of the message (This a on standard message used on Applanix GPS).
    Serial.write( &utcDate[0], 9);                // AAAAMMJJ,
    Serial.write( &utcTime[0], 7);                // HHMMSS.
    Serial.printf( "%u\r\n ", DeltaUTC );         // Show the last time correction for the previous PPS (Lazy move, current UTC time, but with previous UTC time correction value).
    ppsISR_ = false;                              // PPS ISR serviced.
    digitalWrite(LED_BUILTIN, HIGH);              // LED ON to show PPS detection.
  } //endif ppsISR 
  //
  if( Serial1.available() ) {                                 // Data waiting to be read on serial1 port ?.
    inByte = Serial1.read();                                  // Read data from GPS, Serail1 RX pin.
    Serial1.write( inByte );                                  // Write date to data logger, Serial1 TX pin.
    //
    if( inByte == '$' ) {                                     // Is this a start of NMEA mesage '$'
      debutNMEA = true;                                       // Yes set start of message flag
      finNMEA   = false;                                      // Start of NMEA message detected, cancel any pending flag for end of NMEA message detection.
      receivedChars[0] = inByte;                              // Save the data in memory buffer for later processing
      indexMsg = 1;    									      // increment buffer pointer for next data
    } else if ( inByte == 10 /*|| indexMsg > 87*/ ) {         // Is this the end of NMEA message (LF) ?  
        receivedChars[indexMsg] = inByte;                     // Save the data in memory buffer for later processing
        indexMsg++;                                           // increment buffer pointer for next data
        finNMEA = true;                                       // Set the end of NMEA message flag
      }                             
    else {                                                    // Otherwise this is middle NMEA message data
      if( debutNMEA){                                         // If no start of NMEA message detected discard data
        receivedChars[indexMsg] = inByte;                     // Save the data in memory buffer for later processing
        indexMsg++;                                           // increment buffer pointer for next data
      }
    }
    digitalWrite(LED_BUILTIN, LOW);            				  // LED OFF to show the end of current data processing
    //
    // Data received, saved and transittted, you are ether in the middle of the batch of NMEA messge or at the end of the bactch.
    // Depending of your board, Serial internal input buffer and serial speed, you have less than 2.6 mS to do internal housekeeping before losing data for serial buffer overflow.
    // If it's the last NMEA message, you have around 800 mS of internal housekeeping before losing data from serial buffer overflow
    //
    if ( debutNMEA &&  finNMEA ) {                  // Current NMEA message complete.
      if ( receivedChars[0] == '$' && receivedChars[1] == 'G' && receivedChars[2] == 'N' && receivedChars[3] == 'R' && receivedChars[4] == 'M' && receivedChars[5] == 'C' && receivedChars[6] == ',' ) { // Message "$GNRMC," extract time.
        utcTime[0] = receivedChars[7];          //H
        utcTime[1] = receivedChars[8];          //H
        utcTime[2] = receivedChars[9];          //M
        utcTime[3] = receivedChars[10];         //M
        utcTime[4] = receivedChars[11];         //S
        utcTime[5] = receivedChars[12];         //S
        utcTime[6] = receivedChars[13];         //.  Keep also the dot. 
      }//endif $GNRMC,                          //  This in the middle of the batch of NMEA messages take less than 2.4 mS :) :) :)  Were good !
      else if (receivedChars[0] == '$' && receivedChars[1] == 'G' && receivedChars[2] == 'N' && receivedChars[3] == 'Z' && receivedChars[4] == 'D' && receivedChars[5] == 'A' && receivedChars[6] == ',' ) { //Message "$GNZDA," extract date
        // If you configure your GPS to send too many data the last message can be delayed until the next PPS. So extract only the date, as date as very low propabilty of changing over the next PPS. 
        // Raphaël ne fait pas de kayak à 00h00 UTC, car il y a bogue extrême ici. Le changement de date est potentiellement retardé d'une seconde à cause de la dérive du PPS dans les messages NMEA :), mais l'heure est toujours bonne.
        // Autre BOGUE potentiel Raphaël, si tu restes dans ton Kayak plus de 49 jours tu risques d'avoir un bogue dans le facteur de correction de l'heure UTC, car le compteur millis va faire un rollover de 4294967295 à zéro. Désolé ! :)
        // Extract date
        utcDate[0] = receivedChars[23];   //A
        utcDate[1] = receivedChars[24];   //A
        utcDate[2] = receivedChars[25];   //A        
        utcDate[3] = receivedChars[26];   //A
        utcDate[4] = receivedChars[20];   //M
        utcDate[5] = receivedChars[21];   //M
        utcDate[6] = receivedChars[17];   //J
        utcDate[7] = receivedChars[18];   //J
        utcDate[8] = receivedChars[27];   //,  Keep also the comma
        //
        Serial1.write( "$UTC,");                // $UTC,  Applanix message $UTC     
        Serial1.write( &utcDate[0], 9);         // AAAAMMJJ,
        Serial1.write( &receivedChars[7], 7);   // HHMMSS.      Faster if sending data from input buffer
        // The millis() fuction take 0.718 mS to run (real time delay of 0.720160 mS).
        // The next 6 characters take 0.445 mS to send at 115200 bauds (real time delay of 0.446339 mS).
        // The average real time delay between PPS and start of transmitting the $UTC message is about 0.268 mS
        // Total of 1.167 mS for latency correction of UTC time. UTC time correction precision is at most ± 0.5 mS and on average ± 0.3 mS
		// Add '0' left padding if needed
        DeltaUTC = millis() + 1UL - horodatePPS;                            // If DeltaUTC is annormally high, fallback to a fixed average value (manual testing needed for other board).
        if (DeltaUTC >= 700UL) { Serial1.write( "2678\r\n" ); }             // Set an average arbitrary fixed value, hoping the next correction will be OK.   
        else if( DeltaUTC > 99){ Serial1.printf( "%u0\r\n", DeltaUTC ); }   // Between 100 et 799 mS
        else if( DeltaUTC > 9 ){ Serial1.printf( "0%u0\r\n", DeltaUTC ); }  // Between 10 et   99 mS
        else { Serial1.printf( "00%u0\r\n", DeltaUTC ); }                 	// Between  0 et    9 mS
        //
       if( horodatePPS == 0 ) {
         Serial.write( "GPS COLD START ! Waiting for PPS fix.\r\n" ); 		// On GPS cold start and warm start the PPS signal take a long time before showing up is nose.
       }  
      }//endif $GNZDA,
      debutNMEA = false;           // Current NMEA message end of processing, we are now waiting for the next NMEA message.
      finNMEA = false;             // Current NMEA message end of processing, we are now waiting for the next NMEA message.
    }//endif debut et fin,
  } //endif available()
}//end loop
//
//
// ++++++++++ END OF PROGRAM ++++++++++
//
//
// Note for section loop()  *****
//
// Do not use these blocking functions or you risk loosing data
//   Serial.parseInt()
//   Serial.parseFloat()
//   Serial.readBytes()
//   Serial.readBytesUntil()
//
// Do not string class, this is the best way to add memory corruptions, memory leaks to this project.
// Use alternate fast and clean "C" functions here http://www.cplusplus.com/reference/cstring/
//
// char myChar = 'A';
// char myChar = 65; // Both do the same thing.
//
// Logic opertors
//
// and             &&
// and_eq          &=
// bitand          &
// bitor           |
// not             !
// not_eq          !=
// or              ||
// or_eq           |=
// xor             ^
// xor_eq          ^=
//
// The misterious mills() fonction !
//
// The millis() function rely on an integer counter to work and on the CPU frequancy divided by 1024.
// A CPU at 48 MHz,translate to 48 000 000 cycles / second.
// So 48 000 000 cycle divded by 1024 give 46 875 cycles for each second.
// 46 875 cycles divided by 1000 equal 46.875 cycles for each millisecond
// The millis() counter is an integer number, so we must round for 47 cycles by millisecond
// This give a ratio 46.85 on 47 of 0,9968 
// A one millisecond for the CPU is in real time more 0.9973 millisecond.
// A delay of 1 000 millisecond times 0,9968 and rounded give 977 tick for the CPU for a real time equivalent of 1 000 millisecondes.
// This is somewhat 1 mS difference on real time for a delay of 272 mS.
// The Arduino use an algorithm to partially compensate for that on the long run.
//
//
// ***** The End ! - Bonne campagne de mesure Raphaël *****

