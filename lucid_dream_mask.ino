#include <SoftwareSerial.h>
#include <DFMiniMp3.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>

ISR (WDT_vect) {
   // this interrupt disables the watchdog timer
   wdt_disable();
}

class Mp3Notify {
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action) {
    if (source & DfMp3_PlaySources_Sd) Serial.print("SD Card, ");
    if (source & DfMp3_PlaySources_Usb) Serial.print("USB Disk, ");
    if (source & DfMp3_PlaySources_Flash) Serial.print("Flash, ");
    Serial.println(action);
  }
  static void OnError(uint16_t errorCode) {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }
  static void OnPlayFinished(DfMp3_PlaySources source, uint16_t track) {
    Serial.print("Play finished for #");
    Serial.println(track);  
  }
  static void OnPlaySourceOnline(DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted(DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved(DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "removed");
  }
};

// instance a DFMiniMp3 object, defined with the above notification class and a software
// serial port
SoftwareSerial secondarySerial(10, 11); // RX, TX
DFMiniMp3<SoftwareSerial, Mp3Notify> mp3(secondarySerial);

// milliseconds from startup to wait until
uint32_t waitUntil = 0;
// index of current timestamp
int timestampIndex = 0;

// piezoelectric sensor sampling variables
int sensorValue = 0; // for storing sensor sample values
int sensorPeak = 0; // for storing peak sample values
int peakIndex = 0; // for counting samples taken for current peak
const int peakPeriod = 50; // number of samples needed for final peak
const int peakMaximum = 30; // maximum value for final peak
int sensorSum = 0; // sum of final peak value
int sumIndex = 0; // for counting final peak values added
const float sumPeriod = 10.0; // number of final peak values needed for average
float averageArray[] = {
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // array of averages to be used as FIFO stack
int pos = 0; // current position on array
const float threshold = 0.5; // threshold for the difference between any average
                             // and the average before it to be considered movement
bool passingArray[] = {
false, false, false, false, false, false, false, false,
false, false, false, false, false, false, false, false,
false, false, false, false, false, false, false, false,
false, false, false, false, false, false, false, false}; // array of each difference
                                                         // that passes the threshold
int passingSum = 0; // for counting the number of passing differences
const int sumThreshold = 10; // threshold for the sum of passing differences to be
                             // considered Rapid Eye Movement

void setup() {
  // LOW BATTERY FAILSAFE
  // to prevent the EEPROM from being repeatedly overwritten by the arduino
  // repeatedly restarting because of a low battery, if Vin < 4.4v the program doesn't
  // start, staying in an infinite loop
  while(analogRead(A0) < 250) {}
  
  // STARTUP
  // start USB serial
  Serial.begin(115200);
  // red LEDs placed over the eyes to alert the user when they are in REM sleep
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  // builtin LED used to signal that this device is in SENSOR TESTING MODE
  pinMode(LED_BUILTIN, OUTPUT);
  // start DFPlayer
  mp3.begin();
  mp3.setVolume(16);
  // play startup mp3 and flash red LEDs once to signal the completion of STARTUP
  mp3.playMp3FolderTrack(2);
  flashLEDrtl();
  // print all EEPROM timestamps to the serial monitor and reset them until a
  // value less than or equal to the previous value is found, denoting the end
  // of the list, waiting 1 minute first if USB is connected to allow time to
  // turn on the serial monitor before all of this data is printed and lost
  // while the serial monitor is not active
  int ts = 0;
  if (analogRead(A0) < 270) waitMilliseconds(60000);
  Serial.println("Previous timestamps in minutes:");
  for (int i = 0; i < EEPROM.length(); i++) {
    ts = EEPROM.read(i);
    if (ts == 255) break;
    EEPROM.write(i,255);
    Serial.println(ts * 5);
  }

  // SENSOR TESTING MODE
  // turn on builtin LED to signal mode
  digitalWrite(LED_BUILTIN, HIGH);
  // stay in loop as long as Vin < 4.67v, where the 4.5v provided by USB power cannot
  // exceed that unlike the 8~10v provided by the 9v NiMH battery used, measured by A0
  // with a voltage divider of 39.3ohm/139.3ohm to keep the voltage of the ADC below 5v
  Serial.println("Sensor sampling:");
  while(analogRead(A0) < 270) {
    // every millisecond, check DFPlayer and sample piezoelectric sensor
    mp3.loop();
    sampleSensor();
    delay(1);
    // if REM is detected, play ping mp3 and blink lights for 1 second
    if(passingSum > sumThreshold) {
      mp3.playMp3FolderTrack(4);
      displayLEDpattern(1,1,1);
      waitMilliseconds(500);
      displayLEDpattern(0,0,0);
      waitMilliseconds(500);
    }
  }

  // REM DETECTION MODE
  // turn off builting LED to signal mode and save power throughout the night
  digitalWrite(LED_BUILTIN, LOW);
  // run sleep cycle subroutines based on rough estimate of time between the start
  // of each REM cycle which shortens throughout the night as you enter REM sleep
  // more frequently and stay in other stages of sleep less and less
  sleepCycle(150,15,3); // 2:30-2:45
  sleepCycle(60,15,3);  // 3:45-4:00
  sleepCycle(60,15,3);  // 5:00-5:15
}

void loop() {
  sleepCycle(30,10,3);
}

void stampTime() {
  // save the number of 5 minute units since the start of the program to the EEPROM
  // and then increment the index and clear the next byte if it's greater than the
  // current time
  byte ts = millis() / 300000;
  EEPROM.write(timestampIndex,ts);
  timestampIndex++;
  if (EEPROM.read(timestampIndex) > ts) EEPROM.write(timestampIndex,0);
}

void sleepCycle(int minMinutes, int samplingMinutes, int alarmMinutes) {
  // wait the minimum number of minutes before REM sleep should start and stamp time
  waitMinutes(minMinutes);
  stampTime();
  // reset all global variables used for the piezoelectric sensor
  resetSensor();
  // stay in loop for the given number of minutes in which REM sleep is likely to
  // start, checking the DFPlayer and sampling the piezoelectric sensor until
  // REM sleep is detected or the time runs out
  waitUntil = (samplingMinutes * 60000) + millis();
  while (millis() < waitUntil) {
    mp3.loop();
    sampleSensor();
    delay(1);
    if(passingSum > sumThreshold) break;
  }
  // for the given number of minutes, play the alarm mp3 and blink the lights back
  // and forth to alert the user in their dream of the fact that they are dreaming
  waitUntil = (alarmMinutes * 60000) + millis();
  while (millis() < waitUntil) {
    mp3.playMp3FolderTrack(5);
    flashLEDrtl();
    flashLEDltr();
    flashLEDrtl();
    flashLEDltr();
    flashLEDrtl();
    flashLEDltr();
    flashLEDrtl();
    flashLEDltr();
    flashLEDrtl();
    flashLEDltr();
    flashLEDrtl();
    flashLEDltr();
  }
  // stamp time after REM sleep alarm ends
  stampTime();
}

void displayLEDpattern(int A, int B, int C) {
  // set red LEDs
  digitalWrite(7, A);
  digitalWrite(6, B);
  digitalWrite(5, C);
}

void flashLEDrtl() {
  // flash red LEDs in a left to right pattern
  displayLEDpattern(0,0,1);
  waitMilliseconds(100);
  displayLEDpattern(0,1,1);
  waitMilliseconds(100);
  displayLEDpattern(1,1,0);
  waitMilliseconds(100);
  displayLEDpattern(1,0,0);
  waitMilliseconds(100);
  displayLEDpattern(0,0,0);
  waitMilliseconds(100);
}

void flashLEDltr() {
  // flash red LEDs in a right to left pattern
  displayLEDpattern(1,0,0);
  waitMilliseconds(100);
  displayLEDpattern(1,1,0);
  waitMilliseconds(100);
  displayLEDpattern(0,1,1);
  waitMilliseconds(100);
  displayLEDpattern(0,0,1);
  waitMilliseconds(100);
  displayLEDpattern(0,0,0);
  waitMilliseconds(100);
}

void resetSensor() {
  // resets all global variables used to detect Rapid Eye Movement by calculating sampled
  // voltages produced by the piezoelectric film taped to the user's eyelid
  sensorPeak = 0;
  peakIndex = 0;
  sensorSum = 0;
  sumIndex = 0;
  memset(averageArray, 0.0, 30);
  pos = 0;
  memset(passingArray, false, 30);
  passingSum = 0;
}

int sampleSensor() {
  // sample the piezoelectric film taped to the user's eyelid, measuring the voltage
  // generated by any movement of the eye or twitching of the eyelid
  sensorValue = analogRead(A5);
  // if the sampled value is greater than the highest peak value in the current sample
  // set, record it and add 1 to the peak index
  if (sensorValue > sensorPeak) sensorPeak = sensorValue;
  peakIndex++;
  // if enough samples have been taken, add the highest peak recorded, or the maximum value
  // allowed, to the sum of peak values, add 1 to the sum index, and reset the peak index
  if (peakIndex >= peakPeriod) {
    peakIndex = 0;
    sumIndex++;
    if (sensorPeak > peakMaximum) sensorSum += peakMaximum;
    else sensorSum += sensorPeak;
    sensorPeak = 0;
    // if enough peaks have been added together, record the average of all those peaks in
    // the average array and reset the sum index
    if (sumIndex >= sumPeriod) {
      sumIndex = 0;
      averageArray[pos] = (sensorSum/sumPeriod);
      sensorSum = 0;
      // calculate the difference between the average just calculated and the average taken
      // before it
      float difference;
      if (pos == 0) {difference = averageArray[0] - averageArray[31];}
      else {difference = averageArray[pos] - averageArray[pos - 1];}
      difference = abs(difference);
      // print the difference between 0 and 3 for the user to see with the serial plotter
      if (difference > 3.0) difference = 3.0;
      Serial.println(difference);
      // record whether the difference passes the threshold and add up the number of
      // passing differences
      passingArray[pos] = difference > threshold;
      passingSum = 0;
      for (int i = 0; i < 30; i++) {if (passingArray[i]) passingSum++;}
      // increment to the next position on the array
      pos++;
      if (pos > 31) pos = 0;
    }
  }
  return sensorValue;
}

void waitMilliseconds(uint32_t msWait) {
  // calculate the millisecond timestamp to wait until
  waitUntil = msWait + millis();
  // stay in the loop until the timestamp is passed, checking the DFPlayer and sampling the
  // piezoelectric sensor every millisecond
  while (millis() < waitUntil) {
    mp3.loop();
    sampleSensor();
    delay(1);
  }
}

void waitMinutes(int minWait) {
  // calculate the millisecond timestamp to wait until
  waitUntil = (minWait * 60000) + millis();
  // disable the ADC to save power
  ADCSRA &= B01111111;
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set WDIE interrupt mode and an 8 second interval 
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);
  wdt_reset();  // pat the dog
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  noInterrupts (); // timed sequence follows
  sleep_enable();
  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);
  // stay in the loop until the timestamp is passed, sleeping 8 seconds at a time
  while (millis() < waitUntil) {
    // guarantees next instruction executed
    interrupts ();
    // enter sleep
    sleep_cpu ();  
    // cancel sleep as a precaution
    sleep_disable();
  }
  // enable the ADC
  ADCSRA |= B10000000; 
}
