/*
**********************************************************************
* Stereo VU Meter for 1 or 2 LED rings or strips build by ericBcreator
* Designed to be used with an Arduino UNO, Nano or compatible device.
**********************************************************************
* Notice: search for @EB in the Sketch for important variables to set
* for the Sketch to work with your setup.
**********************************************************************
* Last updated 20180202 by ericBcreator
*
* This code is free for personal use, not for commercial purposes.
* Please leave this header intact.
*
* contact: ericBcreator@gmail.com
**********************************************************************
*/
#include <Adafruit_NeoPixel.h>
#define nonLinearLogAudio 

//#define led_strip_60                                    //uncomment this if you are using a single 60 LED strip (Center to sides mode)
#define led_2_strip_60                                  //uncomment this if you are using two 60 LED strips (Left-Right bottom to top mode)

//#define led_strip_30                                    //uncomment this if you are using a single 30 LED strip
//#define led_2_strip_30                                  //uncomment this if you are using two 30 LED strips

//#define led_strip_144                                   //uncomment this if you are using a single 144 LED strip
//#define led_2_strip_144                                 //uncomment this if you are using two 144 LED strips

const int useSensorValues = true;                         // @EB


//
// setup pins
//

int leftPin = A0, rightPin = A1;                          // left audio in on analog 0, right on analog 1
int brightnessPin = A4, sensitivityPin = A5;              // potentiometers for brightness and sensitivity on analog 4 and 5
int leftStripPin = 5;                                     // DIN of left led strip on digital pin 5
int rightStripPin = 6;                                    // DIN of right led strip on digital pin 6
int showPeaksPin = 7;                                     // switch to toggle peaks on or off on digital pin 7 (7, 9 for box version)
int showPeaksMomentarySwitch = false;                     // set false for an on/off toggle switch
int reverseShowPeaks = true;                              // reverses the on/off setting in case you made a wiring mistake ;-) @EB
int selectButton1Pin = 8;                                 // push button for changing settings on digital pin 8
int useSelectButton1 = true;                              // set to false if no push button1 for selecting the color scheme is connected  @EB
int selectButton2Pin = 9;                                 // push button for changing settings on digital pin 9
int useSelectButton2 = true;                              // set to false if no push button2 is connected  @EB


#if defined (led_strip_60)
  //settings for a 60 led strip

  int stripNumOfLeds = 60;                                  // the total number of leds
  int stripsOn2Pins = false;                                // set to true if the LED strips or rings are connected to 2 input pins
  uint32_t stripColor[31];                                  // half of the number of leds + 1
  int displayMiddleLed = true;                              // display the middle led (blue). set to true for one strip, false for two strips or rings
  int splitStrip = false;                                   // set to true when using 2 strips or rings, false for one strip
  int middleOffset = 1;                                     // offset for the middle led when using one strip
  int startupAnimationDelay = 6;                            // delay for the startup animation
  int orangeLimitAmount = 0;                                // limit the amount of green of the middle LEDs to make them more orange
  int swapLeftRight = false;                                // swap the left and right input values or not
  
  int dropDelay = 5;                                        // hold time before dropping the leds
  float dropFactor = .94;                                   // value for dropping the leds
  
  int peakTimeNoDropDelay = 250;                            // peak hold time when not dropping the peaks (when droppingPeak is false)
  int peakTimeFirstDropDelay = 70;                          // peak hold time when dropping the first peak
  int peakTimeDropDelay = 7;                                // peak hold time when dropping the rest of the peaks
  float peakDropFactor = .94;                               // value for dropping the peaks
  int droppingPeakFade = false;                             // display the dropping peak fading to black or not
  
  int bouncingPeaksNumOfLeds = 6;                           // how many leds to bounce up (max)
  int bouncingPeaksNumOfLedsMin = 3;                        // how many leds to bounce up (min) when using dynamicBouncingPeaks
  int bouncingPeakDelay = 4;                                // delay between peak bounce updates
  int bouncingPeakCounterInc = 10;                          // increase counter for each bounce update. note: it uses a 0-180 sin function for the bouncing

#elif defined (led_2_strip_60)
  //settings for 2 60 led strips

  int stripNumOfLeds = 60;
  int stripsOn2Pins = true;
  uint32_t stripColor[61];
  int displayMiddleLed = false;
  int splitStrip = true;
  int middleOffset = 0;
  int startupAnimationDelay = 1;
  int orangeLimitAmount = 0;
  int swapLeftRight = false;
  
  int dropDelay = 5;
  float dropFactor = .94;
  
  int peakTimeNoDropDelay = 250;
  int peakTimeFirstDropDelay = 70;
  int peakTimeDropDelay = 7;
  float peakDropFactor = .94;
  int droppingPeakFade = false;
  
  int bouncingPeaksNumOfLeds = 12;
  int bouncingPeaksNumOfLedsMin = 4;
  int bouncingPeakDelay = 4;
  int bouncingPeakCounterInc = 10;
  
#elif defined (led_strip_30)
  //settings for a 30 led strip

  int stripNumOfLeds = 30;                                  // the total number of leds
  int stripsOn2Pins = false;                                // set to true if the LED strips or rings are connected to 2 input pins
  uint32_t stripColor[16];                                  // half of the number of leds + 1
  int displayMiddleLed = true;                              // display the middle led (blue). set to true for one strip, false for two strips or rings
  int splitStrip = false;                                   // set to true when using 2 strips or rings, false for one strip
  int middleOffset = 1;                                     // offset for the middle led when using one strip
  int startupAnimationDelay = 10;                           // delay for the startup animation
  int orangeLimitAmount = 0;                                // limit the amount of green of the middle LEDs to make them more orange
  int swapLeftRight = false;                                // swap the left and right input values or not
  
  int dropDelay = 10;                                       // hold time before dropping the leds
  float dropFactor = .9;                                    // value for dropping the leds
  
  int peakTimeNoDropDelay = 250;                            // peak hold time when not dropping the peaks (set droppingPeak true or false)
  int peakTimeFirstDropDelay = 150;                         // peak hold time when dropping the first peak
  int peakTimeDropDelay = 15;                               // peak hold time when dropping the rest of the peaks
  float peakDropFactor = .94;                               // value for dropping the peaks
  int droppingPeakFade = false;                             // display the dropping peak fading to black or not
  
  int bouncingPeaksNumOfLeds = 4;                           // how many leds to bounce up (max)
  int bouncingPeaksNumOfLedsMin = 2;                        // how many leds to bounce up (min) when using dynamicBouncingPeaks
  int bouncingPeakDelay = 4;                                // delay between peak bounce updates
  int bouncingPeakCounterInc = 9;                           // increase counter for each bounce update. note: it uses a 0-180 sin function for the bouncing

#elif defined (led_2_strip_30)
  //settings for 2 30 led strips

  int stripNumOfLeds = 30;
  int stripsOn2Pins = true;
  uint32_t stripColor[31];
  int displayMiddleLed = false;
  int splitStrip = true;
  int middleOffset = 0;
  int startupAnimationDelay = 1;
  int orangeLimitAmount = 0;
  int swapLeftRight = false;
  
  int dropDelay = 5;
  float dropFactor = .94;
  
  int peakTimeNoDropDelay = 250;
  int peakTimeFirstDropDelay = 70;
  int peakTimeDropDelay = 7;
  float peakDropFactor = .94;
  int droppingPeakFade = false;
  
  int bouncingPeaksNumOfLeds = 12;
  int bouncingPeaksNumOfLedsMin = 4;
  int bouncingPeakDelay = 4;
  int bouncingPeakCounterInc = 10;
  
#elif defined (led_strip_144)
  //settings for a 144 led strip

  int stripNumOfLeds = 145;
  int stripsOn2Pins = false;
  uint32_t stripColor[73];
  int displayMiddleLed = true;
  int splitStrip = false;
  int middleOffset = 1;
  int startupAnimationDelay = 1;
  int orangeLimitAmount = 0;
  int swapLeftRight = false;
  
  int dropDelay = 4;
  float dropFactor = .92;
  
  int peakTimeNoDropDelay = 250;
  int peakTimeFirstDropDelay = 100;
  int peakTimeDropDelay = 5;
  float peakDropFactor = .94;
  int droppingPeakFade = false;
  
  int bouncingPeaksNumOfLeds = 10;
  int bouncingPeaksNumOfLedsMin = 4;
  int bouncingPeakDelay = 2;
  int bouncingPeakCounterInc = 10;

#elif defined (led_2_strip_144)
  //settings for 2 144 led strips

  int stripNumOfLeds = 144;
  int stripsOn2Pins = true;
  uint32_t stripColor[145];
  int displayMiddleLed = false;
  int splitStrip = true;
  int middleOffset = 0;
  int startupAnimationDelay = 1;
  int orangeLimitAmount = 0;
  int swapLeftRight = false;
  
  int dropDelay = 5;
  float dropFactor = .94;
  
  int peakTimeNoDropDelay = 250;
  int peakTimeFirstDropDelay = 70;
  int peakTimeDropDelay = 7;
  float peakDropFactor = .94;
  int droppingPeakFade = false;
  
  int bouncingPeaksNumOfLeds = 12;
  int bouncingPeaksNumOfLedsMin = 4;
  int bouncingPeakDelay = 4;
  int bouncingPeakCounterInc = 10;
  
#endif



















































//
// setup other user variables
//

// basic settings
int pulsing = false;                                       // pulsing will display from the middle of each strip or ring  @EB

int spinCircle = false;                                    // spin the animation. will not work with stripsOn2Pins  @EB

int animType = 0;                                         // startup animation selection (1 looks nice for 1 ring)  @EB
int colorScheme = 10;                                     // 0: green-red, 1: blue-green, 2: blue-red, 3: red-blue, 4: green-blue, 5: red-green, 6: blue-white-red
                                                          // 7: red-white-blue, 8: green-white-red, 9: green-white-blue, 10: color wheel, 11: spinning color wheel,
                                            // 12: as 11 but spread with factor colorScheme12Factor  @EB
int maxColorScheme = 12;                                  // used for looping through the color schemes with the switch button
int colorScheme11SpinDelay = stripNumOfLeds / 4 ;         // delay for spinning scheme 11
int colorScheme12Factor = 3;                              // wheel spread factor for scheme 12 @EB

int minValue = 10;                                        // min analog input value
int sensitivityValue = 110;                               // 0 - 255, initial value (value read from the potentiometer if useSensorValues = true)

#ifdef highLevelInput
  int maxValue = 700;                                     // max analog input value (0-1023 equals 0-5V). try 300 for low level input, 700 for high
  int maxSensitivity = 2 * 255;                           // set to a higher setting to amplify low input levels. try 4 * 255 for low level input, 2 * 255 for high
#else
  int maxValue = 300;                                     // max analog input value (0-1023 equals 0-5V). try 300 for low level input, 700 for high
  int maxSensitivity = 4 * 255;                           // set to a higher setting to amplify low input levels. try 4 * 255 for low level input, 2 * 255 for high
#endif

int ledBrightness = 30;                                   // 0 - 255, initial value (value read from the potentiometer if useSensorValues = true)
int sensorDeviationBrightness = 3;                        // eliminate fluctuating values
int overflowDelay = 10;                                   // overflow hold time

// peak settings @EB
int displayPeaks = false;                                 // value will be set by the switch if useSensorValues = true
int displayTopAsPeak = true;                              // always display the top LED in peak color
int droppingPeak = true;                                  // display dropping peaks or not. note: displayPeaks has to be true 
int bouncingPeaks = false;                                // display bouncing peaks or not. note: displayPeaks has to be true 
int dynamicBouncingPeaks = false;                         // bounce less with lower peaks. note: bouncingPeaks has to be true 

//
// initialize other variables 
//

int numOfSegments, halfNumOfSegments, stripMiddle, maxDisplaySegments;
float sensitivityFactor;
float nonLinearResponseFactor;

int brightnessValue, prevBrightnessValue;
float ledFactor, ledFactor_div_numOfSegments;

uint32_t stripMiddleColor, stripOverflowColor, stripHoldColor;
uint32_t colorValue;

int leftValue = 0, rightValue = 0, maxReadValue = 0;
int leftValueN = 0, rightValueN = 0;
int leftAnalogValue = 0, rightAnalogValue = 0;
float log10MaxDisplaySegments;

int prevLeftValue = 0, prevRightValue = 0;
int prevLeftAnalogValue = 0, prevRightAnalogValue = 0;

int selectButton1PinState = 0, prevSelectButton1PinState = 0;
int selectButton2PinState = 0, prevSelectButton2PinState = 0;

int selectButton1PinSetting = colorScheme;
int selectButton2PinSetting = 0;

int i, j;
int dropLeft, dropRight;
int leftDropTime, rightDropTime;

int leftPeak = 0, rightPeak = 0;
int leftPeakTime = 0, rightPeakTime = 0;
int leftFirstPeak = true, rightFirstPeak = true;
int showPeaksPinSetting, prevShowPeaksPinSetting;

int stripPulseMiddle = 0;
int halfLeftValue, halfRightValue, halfPrevLeftValue, halfPrevRightValue;

int leftPeakBouncing = false, rightPeakBouncing = false;
int leftPeakBounce = 0, rightPeakBounce = 0;
int prevLeftPeakBounce = 0, prevRightPeakBounce = 0;
int leftPeakBounceCounter = 0, rightPeakBounceCounter = 0;
int leftPeakBounceDelayCounter = 0, rightPeakBounceDelayCounter = 0;
int leftBouncingPeaksNumOfLeds = 0, rightBouncingPeaksNumOfLeds = 0;
float bounceFactor;

int colorScheme11SpinValue = 0, colorScheme11SpinDelayValue = 0;
int colorSchemeFactor = 1;
long selectButton1Timer;

int spinDelayCounter = 0, spinCounter = 0, spinTurnsCounter = 0, spinTurnsMax = 0, spinTurnsDelay = 0, spinTurnsDelayMax = 0;
int spinCounterInc = 1;
int spinDelay = 0;
//
// initialize the strip or rings
//

Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(stripNumOfLeds, leftStripPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel right_strip = Adafruit_NeoPixel(stripNumOfLeds, rightStripPin, NEO_GRB + NEO_KHZ800);

//
// setup
//

void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  randomSeed(analogRead(2));

  if (stripsOn2Pins) {
    numOfSegments = stripNumOfLeds;
    maxDisplaySegments = numOfSegments - 1;
    
    stripMiddle = stripNumOfLeds;
    stripPulseMiddle = stripMiddle / 2;
    spinCircle = false;
  }
  else {
    numOfSegments = stripNumOfLeds / 2;
    stripMiddle = stripNumOfLeds / 2;
    maxDisplaySegments = stripMiddle - 1;

    stripPulseMiddle = stripMiddle / 2;
  }

  halfNumOfSegments = numOfSegments / 2; 
  bounceFactor = (float) bouncingPeaksNumOfLeds / (maxDisplaySegments - bouncingPeaksNumOfLeds);
  nonLinearResponseFactor = 90 / (float) maxDisplaySegments;
  log10MaxDisplaySegments = log10(maxDisplaySegments);

  pinMode(showPeaksPin, INPUT);    
  
  if (useSelectButton1)
    pinMode(selectButton1Pin, INPUT);  

  left_strip.begin();
  if (stripsOn2Pins) 
    right_strip.begin();

  if (useSensorValues) {
    readSensorValues();
    setInitialDisplayPeaks();
  }
  else {
    setStripColors();
    setSensitivityFactor();
  }

  #ifdef DEBUG_TEST_LEDS
    displayTest();
  #endif

  startupAnimation();
}

//
// main loop
//

void loop() {
  #ifdef DEBUG_PRINT_LOOP_TIME
    long time = millis();
  #endif


  if (useSensorValues) 
    readSensorValues();
  
  readValues();

  #if defined (DEBUG_NO_PEAKS)
    displayPeaks = false;
  #endif

  #if defined (DEBUG_PEAKS)
    displayPeaks = true;
  #endif

  if (pulsing) {
    drawPulsingValues();
  }
  else {
    drawValues();
    if (displayPeaks) {
      getPeaks();
      drawPeaks();
    }
  }

  left_strip.show();
  if (stripsOn2Pins)
    right_strip.show();
    
  storePrevValues();

  checkSpinCircle();

  #ifdef DEBUG_PRINT_LOOP_TIME
    time = millis() - time;
    Serial.println(time);
  #endif
}

// 
// functions
//

void setInitialDisplayPeaks() {
  #if !defined (DEBUG_NO_PEAK_SWITCH)
    showPeaksPinSetting = digitalRead(showPeaksPin);
  
    if (showPeaksPinSetting == HIGH)
      displayPeaks = false;
  #endif 
 
  if (reverseShowPeaks) {
    if (!displayPeaks)
      displayPeaks = true;
    else
      displayPeaks = false;
  }
  
  prevShowPeaksPinSetting = showPeaksPinSetting;
}

void readSensorValues() {
  //
  // peaks pin
  //

  #if !defined (DEBUG_NO_PEAK_SWITCH)
    showPeaksPinSetting = digitalRead(showPeaksPin);
  
    if (showPeaksMomentarySwitch) {
      if (showPeaksPinSetting == LOW && prevShowPeaksPinSetting == HIGH) {
        if (displayPeaks == true) {
          displayPeaks = false;
          clearLeftPeak();
          clearRightPeak();        
          if (showPeaksMomentarySwitch)
            while (digitalRead(showPeaksPin) == LOW) {}
        }
        else {
          displayPeaks = true;
        }
      }
    } 
    else {
      if (reverseShowPeaks) {
        if (showPeaksPinSetting == HIGH && prevShowPeaksPinSetting == LOW) 
          displayPeaks = true;
        else if (showPeaksPinSetting == LOW && prevShowPeaksPinSetting == HIGH) {
          displayPeaks = false;
          clearLeftPeak();
          clearRightPeak();        
        }
      }
      else {    
        if (showPeaksPinSetting == LOW && prevShowPeaksPinSetting == HIGH) 
          displayPeaks = true;
        else if (showPeaksPinSetting == HIGH && prevShowPeaksPinSetting == LOW) {
          displayPeaks = false;
          clearLeftPeak();
          clearRightPeak();        
        }
      }
    }
    if (pulsing) {
      if (displayPeaks)
        displayTopAsPeak = true;
      else
        displayTopAsPeak = false;
    }

    prevShowPeaksPinSetting = showPeaksPinSetting;
  #endif
  

  //
  // selectButtonPin 1 and 2
  //
  if (useSelectButton1) {      
    selectButton1PinState = digitalRead(selectButton1Pin);
    
    if (selectButton1PinState == HIGH && prevSelectButton1PinState == LOW)
      selectButton1Timer = millis();
      
    if (selectButton1PinState == HIGH && prevSelectButton1PinState == HIGH) {
      if ((millis() - selectButton1Timer) > 1000) {
        pulsing = !pulsing;
        setStripColors();
        displayNumber(colorScheme, 250);

        while (digitalRead(selectButton1Pin) == HIGH) {}
        selectButton1PinState = LOW;
        clearValues();
      }
    }
    else if (selectButton1PinState == LOW && prevSelectButton1PinState == HIGH) {
      selectButton1PinSetting++;
      if (selectButton1PinSetting > maxColorScheme) {
        selectButton1PinSetting = 0;
      }
      colorScheme = selectButton1PinSetting;

      if (colorScheme == 12)
        colorScheme11SpinValue = (colorScheme11SpinValue * colorScheme12Factor);

      setStripColors();
      displayNumber(colorScheme, 250);
    }
    prevSelectButton1PinState = selectButton1PinState;
  }

  if (useSelectButton2) {
    selectButton2PinState = digitalRead(selectButton2Pin);
    
    if (selectButton2PinState == HIGH && prevSelectButton2PinState == LOW) {
      selectButton2PinSetting++;
      
      switch(selectButton2PinSetting) {
        case 0:
        case 1: {   
          pulsing = true;
          spinCircle = false;
          break;
        }
        case 2: {
          pulsing = true;
          spinCircle = true;
          break;
        }
        case 3: {
          pulsing = false;
          spinCircle = false;
          selectButton2PinSetting = 0;
          break;
        }
      }
      
      setStripColors();
      displayNumber(colorScheme, 250);
    }
    
    prevSelectButton2PinState = selectButton2PinState;
  }
  
  //
  // brightness
  //
  brightnessValue = analogRead(brightnessPin);
  brightnessValue = map(brightnessValue, 0, 1023, 0, 255);
  
  if (abs(brightnessValue - prevBrightnessValue) > sensorDeviationBrightness) {
    ledBrightness = brightnessValue;
    setStripColors();
    prevBrightnessValue = brightnessValue;
  }

  //
  // colorscheme 11 spinning wheel
  //

  if (colorScheme == 11 || colorScheme == 12) {
    colorScheme11SpinDelayValue++;
    if (colorScheme11SpinDelayValue == colorScheme11SpinDelay) {
      colorScheme11SpinDelayValue = 0;
      colorScheme11SpinValue++;
      if (colorScheme11SpinValue > maxDisplaySegments * colorSchemeFactor)
        colorScheme11SpinValue = 0;
      setStripColors();
    }
  }

  //
  // sensitivity
  //
  sensitivityValue = analogRead(sensitivityPin);
  sensitivityValue = map(sensitivityValue, 0, 1023, 0, 255);
  setSensitivityFactor();
}

void setSensitivityFactor() {
  //sensitivityValue_div_numOfSegments = sensitivityValue / numOfSegments;
  sensitivityFactor = ((float) sensitivityValue / 255 * (float) maxSensitivity / 255);
}

void readValues() {
  #ifdef averageReadings
    leftAnalogValue = 0;
    rightAnalogValue = 0;
    
    for (i = 0; i <= averageNumOfReadings; i++) {
      leftAnalogValue += analogRead(leftPin);
      rightAnalogValue += analogRead(rightPin);
    }

    leftAnalogValue /= averageNumOfReadings;
    rightAnalogValue /= averageNumOfReadings;
    
  #else  
    leftAnalogValue = analogRead(leftPin);
    rightAnalogValue = analogRead(rightPin);
  #endif

  if (swapLeftRight) {
    int tempValue = leftAnalogValue;
    leftAnalogValue = rightAnalogValue;
    rightAnalogValue = tempValue;
  }

  if (leftAnalogValue < prevLeftAnalogValue) {
    leftDropTime++;
    if (leftDropTime > dropDelay) {
      leftAnalogValue = prevLeftAnalogValue * dropFactor;
      leftDropTime = 0;
    }
    else
      leftAnalogValue = prevLeftAnalogValue;
  }
   
  if (rightAnalogValue < prevRightAnalogValue) {
    rightDropTime++;
    if (rightDropTime > dropDelay) {
      rightAnalogValue = prevRightAnalogValue * dropFactor;
      rightDropTime = 0;
    }
    else
      rightAnalogValue = prevRightAnalogValue;
  }

  #ifdef DEBUG_PRINT_ANALOGVALUES
    Serial.print(leftAnalogValue);
    Serial.print(" ");
    Serial.println(rightAnalogValue);
  #endif  

  // map values  
  leftValue = map(leftAnalogValue * sensitivityFactor, minValue, maxValue, 0, maxDisplaySegments);
  rightValue = map(rightAnalogValue * sensitivityFactor, minValue, maxValue, 0, maxDisplaySegments);
  
  // if defined, convert to (reverse) non linear response
  boolean flagNonLinear = false;
  
  #if defined (nonLinearSinAudio)
    flagNonLinear = true;
    leftValueN = ((sin(((leftValue * nonLinearResponseFactor) + 270) * 0.0174533) + 1) * maxDisplaySegments);
    rightValueN = ((sin(((rightValue * nonLinearResponseFactor) + 270) * 0.0174533) + 1) * maxDisplaySegments);
    
  #elif defined (nonLinearReverseSinAudio)
    flagNonLinear = true;
    leftValueN = ((sin(((leftValue * nonLinearResponseFactor)) * 0.0174533)) * maxDisplaySegments);
    rightValueN = ((sin(((rightValue * nonLinearResponseFactor)) * 0.0174533)) * maxDisplaySegments);

  #elif defined (nonLinearLogAudio)
    flagNonLinear = true;
    leftValueN = ((log10(leftValue  + 1) / log10MaxDisplaySegments * maxDisplaySegments));
    rightValueN = ((log10(rightValue  + 1) / log10MaxDisplaySegments * maxDisplaySegments));
  
  #endif
  
  if (flagNonLinear == true) {
    #if defined (nonLinearAvr2)
      leftValue = (leftValue + leftValueN) / 2;
      rightValue = (rightValue + rightValueN) / 2;
    #else
      leftValue = leftValueN;
      rightValue = rightValueN;
    #endif
  }
  
// @EB_DEBUG

  #ifdef displayOverflow
    #ifdef compressOverflowPeaks
      for (i = 1; i <= compressOverflowNumOfTimes; i++) {
        if (leftValue > maxDisplaySegments) {
//          Serial.print(i);    
//          Serial.print("  ");
//          Serial.print(leftValue);    
//          Serial.print("  ");
          leftValue = leftValue - leftValue * compressOverflowFactor * i;
//          Serial.print(leftValue);
//          Serial.print("  ");
//          Serial.println(maxDisplaySegments);
        }
      }
    #endif
  #endif
  
  if (leftValue > maxDisplaySegments) {      
    leftValue = maxDisplaySegments;
    #ifdef displayOverflow
      drawOverflow();
    #endif
  }

  #ifdef displayOverflow
    #ifdef compressOverflowPeaks
      if (rightValue > maxDisplaySegments)
        rightValue = rightValue - rightValue * compressOverflowFactor;
    #endif
  #endif
  
  if (rightValue > maxDisplaySegments) {
    rightValue = maxDisplaySegments;
    #ifdef displayOverflow
      drawOverflow();
    #endif
  }
}

void storePrevValues() {
  prevLeftAnalogValue = leftAnalogValue;
  prevRightAnalogValue = rightAnalogValue;

  prevLeftValue = leftValue;
  prevRightValue = rightValue;
}

void getPeaks() {
  if (leftValue > leftPeak) {
    if (dynamicBouncingPeaks || prevLeftPeakBounce > 0)
      clearLeftBouncePeak();
          
    leftPeak = leftValue;
    leftPeakTime = 0;
    leftFirstPeak = true;

    if (bouncingPeaks) {
      leftPeakBouncing = true;
      leftPeakBounceCounter = 0;
      leftPeakBounceDelayCounter = 0;
      
      if (dynamicBouncingPeaks)
        leftBouncingPeaksNumOfLeds = max(bouncingPeaksNumOfLedsMin, (leftPeak * bounceFactor));
      else
        leftBouncingPeaksNumOfLeds = bouncingPeaksNumOfLeds;
    }
  }
  else {
    leftPeakTime++;
    if (droppingPeak) {
      if (leftFirstPeak) {
        if (leftPeakTime > peakTimeFirstDropDelay) {
          clearLeftPeak();
          leftFirstPeak = false;
        }
      }
      else {
        if (leftPeakTime > peakTimeDropDelay) {
          clearLeftPeak();
        }
      }
    }
    else {
      if (leftPeakTime > peakTimeNoDropDelay) {
        clearLeftPeak();
      }
    }
  }

  if (leftPeakBouncing) {
    if (leftFirstPeak) {
      leftPeakBounceDelayCounter++;
      if (leftPeakBounceDelayCounter >= bouncingPeakDelay) {
        leftPeakBounceDelayCounter = 0;
        leftPeakBounceCounter += bouncingPeakCounterInc;
        if (leftPeakBounceCounter >= 180) {
          clearLeftBouncePeak();
          clearLeftBounce();
        }
        else {        
          leftPeakBounce = min((sin(leftPeakBounceCounter * 0.0174533) * leftBouncingPeaksNumOfLeds), (maxDisplaySegments - leftPeak));
          if (leftPeakBounce != prevLeftPeakBounce) {
            clearLeftBouncePeak();
          }
          prevLeftPeakBounce = leftPeakBounce;
        }
      }
    }
  }

  if (rightValue > rightPeak) {
    if (dynamicBouncingPeaks || prevRightPeakBounce > 0)
      clearRightBouncePeak();

    rightPeak = rightValue;
    rightPeakTime = 0;
    rightFirstPeak = true;

    if (bouncingPeaks) {
      rightPeakBouncing = true;
      rightPeakBounceCounter = 0;
      rightPeakBounceDelayCounter = 0;

      if (dynamicBouncingPeaks)
        rightBouncingPeaksNumOfLeds = max(bouncingPeaksNumOfLedsMin, (rightPeak * bounceFactor));
      else
        rightBouncingPeaksNumOfLeds = bouncingPeaksNumOfLeds;
    }
  }
  else {
    rightPeakTime++;
    if (droppingPeak) {
      if (rightFirstPeak) {
        if (rightPeakTime > peakTimeFirstDropDelay) {
          clearRightPeak();
          rightFirstPeak = false;
        }
      }
      else {
        if (rightPeakTime > peakTimeDropDelay)
          clearRightPeak();
      }
    }
    else {
      if (rightPeakTime > peakTimeNoDropDelay)
        clearRightPeak();
    }
  }

  if (rightPeakBouncing) {
    if (rightFirstPeak) {
      rightPeakBounceDelayCounter++;
      if (rightPeakBounceDelayCounter >= bouncingPeakDelay) {
        rightPeakBounceDelayCounter = 0;
        rightPeakBounceCounter += bouncingPeakCounterInc;
  
        if (rightPeakBounceCounter >= 180) {
          clearRightBouncePeak();
          clearRightBounce();
        }
        else {        
          rightPeakBounce = min((sin(rightPeakBounceCounter * 0.0174533) * rightBouncingPeaksNumOfLeds), (maxDisplaySegments - rightPeak));
          if (rightPeakBounce != prevRightPeakBounce) {
            clearRightBouncePeak();
          }
          prevRightPeakBounce = rightPeakBounce;
        }
      }
    }
  }
}

void checkSpinCircle () {
  if (spinCircle) {
    if (spinTurnsMax == 0) {
      spinTurnsMax = random(stripNumOfLeds / 4, stripNumOfLeds * 3);  // spin at least a quarter turn, max 3 turns
      
      if (random(10) > 4)
        spinCounterInc = -spinCounterInc;
      
      spinTurnsDelayMax = random(100, 1000); // @EB_DEBUG

      spinDelay = random(20, 75); // @EB_DEBUG
    }

    if (spinTurnsCounter == spinTurnsMax) {
      spinTurnsDelay++;
      if (spinTurnsDelay == spinTurnsDelayMax) {
        spinTurnsDelay = 0;
        spinTurnsCounter = 0;
        spinTurnsMax = 0;
      }
    }
    else {
      spinDelayCounter++;
  
      if (spinDelayCounter > spinDelay) {
        clearZeroAndPeaks();

        spinCounter += spinCounterInc;
        if (spinCounter > stripNumOfLeds)
          spinCounter = 0;
        else if (spinCounter < 0)
          spinCounter = stripNumOfLeds;

        spinTurnsCounter++;
        spinDelayCounter = 0;
      }
    }
  }
}

int getSpinCircleValue(int value) {
  if (!spinCircle)
   return value;
  else {
    int calcValue = value + spinCounter;
    if (calcValue >= stripNumOfLeds)
      calcValue -= stripNumOfLeds;
    return calcValue;
  }
}

void drawValues() {
  if (splitStrip) {
    for (i = middleOffset; i < leftValue; i++)
      left_strip.setPixelColor(getSpinCircleValue(i), stripColor[i]);

    if (!displayPeaks && displayTopAsPeak)
      left_strip.setPixelColor(getSpinCircleValue(leftValue), stripHoldColor);

    for (i = prevLeftValue; i >= leftValue; i--)
      left_strip.setPixelColor(getSpinCircleValue(i), 0);

    if (stripsOn2Pins) {
      for (i = middleOffset; i < rightValue; i++) 
        right_strip.setPixelColor(i, stripColor[i]);

      if (!displayPeaks && displayTopAsPeak)
        right_strip.setPixelColor(rightValue, stripHoldColor);

      for (i = prevRightValue; i >= rightValue; i--)
        right_strip.setPixelColor(i, 0);
    }
    else {
      for (i = middleOffset; i < rightValue; i++)
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + i), stripColor[i]);

      if (!displayPeaks && displayTopAsPeak)
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + rightValue), stripHoldColor);
        
      for (i = prevRightValue; i >= rightValue; i--)
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + i), 0);
    }
  }
  else {
    for (i = middleOffset; i < leftValue; i++)
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + i), stripColor[i]);

    if (!displayPeaks && displayTopAsPeak)
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + leftValue), stripHoldColor);

    for (i = prevLeftValue; i >= leftValue; i--)
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + i), 0);
  
    for (i = middleOffset; i < rightValue; i++)
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - i), stripColor[i]);

    if (!displayPeaks && displayTopAsPeak)
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - rightValue), stripHoldColor);

    for (i = prevRightValue; i >= rightValue; i--)
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - i), 0);
  }

  if (displayMiddleLed) 
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle), stripMiddleColor);
}

void drawPulsingValues() {
  halfLeftValue = (leftValue + 1) / 2;
  halfRightValue = (rightValue + 1) / 2;
  halfPrevLeftValue = (prevLeftValue + 1)/ 2;
  halfPrevRightValue = (prevRightValue + 1) / 2;
  
  if (splitStrip) {
    for (i = 0; i < halfLeftValue; i++) {
      colorValue = stripColor[i * 2];
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle + i), colorValue);
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle - i), colorValue);
    }

    if (displayTopAsPeak) {
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle + halfLeftValue), stripHoldColor);
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle - halfLeftValue), stripHoldColor);
    }

    for (i = halfPrevLeftValue; i >= halfLeftValue; i--) {
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle + i), 0);
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle - i), 0);
    }

    if (stripsOn2Pins) {
      for (i = 0; i < halfRightValue; i++) {
        colorValue = stripColor[i * 2];
        right_strip.setPixelColor((stripPulseMiddle + i), colorValue);
        right_strip.setPixelColor((stripPulseMiddle - i), colorValue);
      }

      if (displayTopAsPeak) {
        right_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle + halfRightValue), stripHoldColor);
        right_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle - halfRightValue), stripHoldColor);
      }
      
      for (i = halfPrevRightValue; i >= halfRightValue; i--) {
        right_strip.setPixelColor((stripPulseMiddle + i), 0);
        right_strip.setPixelColor((stripPulseMiddle - i), 0);
      }
    }
    else {
      for (i = 0; i < halfRightValue; i++) {
        colorValue = colorValue = stripColor[i * 2];
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + i), colorValue);
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - i), colorValue);
      }

      if (displayTopAsPeak) {
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + halfRightValue), stripHoldColor);
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - halfRightValue), stripHoldColor);
      }

      for (i = halfPrevRightValue; i >= halfRightValue; i--) {
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + i), 0);
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - i), 0);
      }
    }
  }
  else {
    for (i = 0; i < halfLeftValue; i++) {
      colorValue = colorValue = stripColor[i * 2];
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + i), colorValue);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - i), colorValue);
    }

    if (displayTopAsPeak) {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + halfLeftValue), stripHoldColor);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - halfLeftValue), stripHoldColor);
    }
    
    for (i = halfPrevLeftValue; i >= halfLeftValue; i--) {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + i), 0);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - i), 0);
    }
  
    for (i = 0; i < halfRightValue; i++) {
      colorValue = colorValue = stripColor[i * 2];
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle + i)), colorValue);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle - i)), colorValue);
    }

    if (displayTopAsPeak) {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle + halfRightValue)), stripHoldColor);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle - halfRightValue)), stripHoldColor);
    }
    
    for (i = halfPrevRightValue; i >= halfRightValue; i--) {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle + i)), 0);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle - i)), 0);
    }
  }

  if (displayMiddleLed) {
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle - stripPulseMiddle), stripMiddleColor);
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle), stripMiddleColor);
  }
}

void clearZeroAndPeaks() {
  left_strip.setPixelColor(getSpinCircleValue(middleOffset), 0);
  left_strip.setPixelColor(getSpinCircleValue(stripMiddle), 0);

  if (displayTopAsPeak) {
    if (splitStrip) {
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle + halfLeftValue), 0);
      left_strip.setPixelColor(getSpinCircleValue(stripPulseMiddle - halfLeftValue), 0);
      
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + halfRightValue), 0);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - halfRightValue), 0);
    }
    else {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle + halfLeftValue), 0);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + stripPulseMiddle - halfLeftValue), 0);

      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle + halfRightValue)), 0);
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (stripPulseMiddle - halfRightValue)), 0);
    }
  }
}

void drawPeaks() {  
  if (leftPeak > 0) {
    if (droppingPeakFade && leftPeakBouncing == false)
      stripHoldColor = left_strip.Color(max(1, (255 * leftPeak * ledFactor_div_numOfSegments)), 0, 0);
    else
      stripHoldColor = stripColor[numOfSegments];

    if (splitStrip)
      left_strip.setPixelColor(getSpinCircleValue(leftPeak + leftPeakBounce), stripHoldColor);
    else
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + (leftPeak + leftPeakBounce)), stripHoldColor);
  } 
  
  if (rightPeak > 0) {
    if (droppingPeakFade && rightPeakBouncing == false)
      stripHoldColor = left_strip.Color(max(1, (255 * rightPeak * ledFactor_div_numOfSegments)), 0, 0);    
    else
      stripHoldColor = stripColor[numOfSegments];

    if (splitStrip) {
      if (stripsOn2Pins) {
        right_strip.setPixelColor(getSpinCircleValue(rightPeak + rightPeakBounce), stripHoldColor);
      }
      else {
        left_strip.setPixelColor(getSpinCircleValue(stripMiddle + rightPeak + rightPeakBounce), stripHoldColor);
      }
    }
    else {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (rightPeak + rightPeakBounce)), stripHoldColor);
    }
  }

  //if (leftPeak > 0 || rightPeak > 0)
  //  left_strip.show();    
}

void clearLeftPeak() {  
  if (splitStrip)
    left_strip.setPixelColor(getSpinCircleValue(leftPeak + prevLeftPeakBounce), 0);
  else
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle + (leftPeak + prevLeftPeakBounce)), 0);

  if (droppingPeak)
    leftPeak = leftPeak * peakDropFactor;
  else
    leftPeak = 0;
  leftPeakTime = 0;
}

void clearRightPeak() {
  if (splitStrip) {
    if( stripsOn2Pins) {
      right_strip.setPixelColor(getSpinCircleValue(rightPeak + prevRightPeakBounce), 0);
    }
    else {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + rightPeak + prevRightPeakBounce), 0);
    }
  }
  else {
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (rightPeak + prevRightPeakBounce)), 0);
  }
  
  if (droppingPeak) 
    rightPeak = rightPeak * peakDropFactor;
  else
    rightPeak = 0;
  rightPeakTime = 0;
}

void clearLeftBouncePeak() {
  if (splitStrip)
    left_strip.setPixelColor(getSpinCircleValue(leftPeak + prevLeftPeakBounce), 0);
  else
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle + (leftPeak + prevLeftPeakBounce)), 0);
}

void clearRightBouncePeak() {
  if (splitStrip) {
    if (stripsOn2Pins) {
      right_strip.setPixelColor(getSpinCircleValue(rightPeak + prevRightPeakBounce), 0);
    }
    else {
      left_strip.setPixelColor(getSpinCircleValue((stripMiddle + rightPeak + prevRightPeakBounce)), 0);
    }
  }
  else {
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle - (rightPeak + prevRightPeakBounce)), 0);
  }
}

void clearLeftBounce() {
  leftPeakBouncing = false;
  leftPeakBounceCounter = 0;
  leftPeakBounce = 0;
  prevLeftPeakBounce = 0;
  leftBouncingPeaksNumOfLeds = 0;
}

void clearRightBounce() {
  rightPeakBouncing = false;
  rightPeakBounceCounter = 0;
  rightPeakBounce = 0;
  prevRightPeakBounce = 0;
  leftBouncingPeaksNumOfLeds = 0;
}

void clearValues() {
  leftAnalogValue = 0;
  rightAnalogValue = 0;
  prevLeftAnalogValue = 0;
  prevRightAnalogValue = 0;
  leftPeak = 0;
  rightPeak = 0;
}


void drawOverflow() {
  for (i = 0; i <= numOfSegments; i++) {
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle - i), stripOverflowColor);
    if (stripsOn2Pins) {
      right_strip.setPixelColor(i, stripOverflowColor);
    }
    else {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + i), stripOverflowColor);
    }
  }
  left_strip.show();
  if (stripsOn2Pins)
    right_strip.show();
  
  delay(overflowDelay);

  for (i = 0; i <= numOfSegments; i++) {
    left_strip.setPixelColor(getSpinCircleValue(stripMiddle - i), 0);
    if (stripsOn2Pins) {
      right_strip.setPixelColor(i, 0);
    }
    else {
      left_strip.setPixelColor(getSpinCircleValue(stripMiddle + i), 0);
    }
  }
  left_strip.show();
  if (stripsOn2Pins)
    right_strip.show();
}

void setStripColors() {
  int r, g, b;
  int p1, p2;

  ledFactor = (float) ledBrightness / 255;
  ledFactor_div_numOfSegments = (float) ledFactor / (float) numOfSegments;
  stripMiddleColor = left_strip.Color(0, 0, 255 * ledFactor);
  
  switch (colorScheme) {
    case 0: {      
      int orangeLimit;
      float orangeFactor = orangeLimitAmount / halfNumOfSegments;
    
      for (i = 0; i <= numOfSegments; i++) {
        if (i <= halfNumOfSegments)
          orangeLimit = (i * orangeFactor);
        else
          orangeLimit = ((numOfSegments - i) * orangeFactor);
    
        stripColor[i] = left_strip.Color((255 * i * ledFactor_div_numOfSegments), ((255 - orangeLimit) * (numOfSegments - i) * ledFactor_div_numOfSegments), 0); 
      }
      break;
    }

    case 1: {
      for (i = 0; i <= numOfSegments; i++) {
        stripColor[i] = left_strip.Color(0, (255 * i * ledFactor_div_numOfSegments), (255 * (numOfSegments - i) * ledFactor_div_numOfSegments)); 
      }
      break;
    }

    case 2: {
      for (i = 0; i <= numOfSegments; i++) {
        stripColor[i] = left_strip.Color((255 * i * ledFactor_div_numOfSegments), 0, (255 * (numOfSegments - i) * ledFactor_div_numOfSegments)); 
      }
      break;
    }

    case 3: {
      for (i = 0; i <= numOfSegments; i++) {
        stripColor[i] = left_strip.Color((255 * (numOfSegments - i) * ledFactor_div_numOfSegments), 0, (255 * i * ledFactor_div_numOfSegments)); 
      }
      break;
    }

    case 4: {
      for (i = 0; i <= numOfSegments; i++) {
        stripColor[i] = left_strip.Color(0, (255 * (numOfSegments - i) * ledFactor_div_numOfSegments), (255 * i * ledFactor_div_numOfSegments)); 
      }
      break;
    }

    case 5: {
      for (i = 0; i <= numOfSegments; i++) {
        stripColor[i] = left_strip.Color((255 * (numOfSegments - i) * ledFactor_div_numOfSegments), (255 * i * ledFactor_div_numOfSegments), 0); 
      }
      break;
    }

    case 6: {
      for (i = 0; i <= numOfSegments; i++) {
        r = (255 * i * ledFactor_div_numOfSegments);
        g = (255 * min(i, numOfSegments - i) * ledFactor_div_numOfSegments);
        b = (200 * (numOfSegments - i) * ledFactor_div_numOfSegments); 
        stripColor[i] = left_strip.Color(r, g, b); 
      }
      break;
    }

    case 7: {
      for (i = 0; i <= numOfSegments; i++) {
        b = (255 * i * ledFactor_div_numOfSegments);
        g = (255 * min(i, numOfSegments - i) * ledFactor_div_numOfSegments);
        r = (255 * (numOfSegments - i) * ledFactor_div_numOfSegments); 
        stripColor[i] = left_strip.Color(r, g, b); 
      }
      break;
    }

    case 8: {
      for (i = 0; i <= numOfSegments; i++) {
        r = (255 * i * ledFactor_div_numOfSegments);
        b = (255 * min(i, numOfSegments - i) * ledFactor_div_numOfSegments);
        g = (255 * (numOfSegments - i) * ledFactor_div_numOfSegments); 
        stripColor[i] = left_strip.Color(r, g, b); 
      }
      break;
    }

    case 9: {
      for (i = 0; i <= numOfSegments; i++) {
        b = (255 * i * ledFactor_div_numOfSegments);
        r = (255 * min(i, numOfSegments - i) * ledFactor_div_numOfSegments);
        g = (255 * (numOfSegments - i) * ledFactor_div_numOfSegments); 
        stripColor[i] = left_strip.Color(r, g, b); 
      }
      break;
    }

    case 10: 
      colorScheme11SpinValue = 0;
      
    case 11:     
    
    case 12: {
      p1 = (85 * numOfSegments / 255);
      p2 = (170 * numOfSegments / 255);
      int wheel;

      if (colorScheme == 12)
        colorSchemeFactor = colorScheme12Factor;        
      else
        colorSchemeFactor = 1;
            
      for (i = 0; i <= numOfSegments; i++) {  
        //wheel = int(float(i + colorScheme11SpinValue) / colorSchemeFactor) % numOfSegments;  // reverse wheel
        
        wheel = int(float(i - colorScheme11SpinValue) / colorSchemeFactor + numOfSegments) % numOfSegments;

        if (wheel < p1) {          
          wheel = map(wheel, 0, p1, 0, 255);
          r = (wheel * ledFactor);
          g = ((255 - wheel) * ledFactor);
          b = 0;
        } 
        else if (wheel < p2) {
          wheel = map(wheel, p1, p2, 0, 255);
          r = ((255 - wheel) * ledFactor);
          g = 0;
          b = (wheel * ledFactor);
        } 
        else {
          wheel = map(wheel, p2, numOfSegments, 0, 255);
          r = 0;
          g = (wheel * ledFactor);
          b = ((255 - wheel) * ledFactor);
        }        

        stripColor[i] = left_strip.Color(r, g, b); 
      }
      break;
    }
  }

  if (colorScheme >= 10)
    stripHoldColor = left_strip.Color(255 * ledFactor, 0, 0); // set to red for the color wheels
  else
    stripHoldColor = stripColor[numOfSegments];
    
  stripOverflowColor = stripHoldColor;   // left_strip.Color(min(255, 255 * ledFactor * 1.5), 0, 0);
}

void startupAnimation() {  
  for (j = 0; j < 2; j++) {
    for (i = 0; i <= numOfSegments; i++) {
      if (animType == 1)
        left_strip.setPixelColor(stripMiddle - (numOfSegments - i), stripColor[i]);
      else
        left_strip.setPixelColor(stripMiddle - i, stripColor[i]);
      
      if (stripsOn2Pins)
        right_strip.setPixelColor(i, stripColor[i]);
      else
        left_strip.setPixelColor(stripMiddle + i, stripColor[i]);

      left_strip.show();
      if (stripsOn2Pins)
        right_strip.show();  
      
      delay(startupAnimationDelay);
    }
    
    for (i = 0; i <= numOfSegments; i++) {
      if (animType == 1)
        left_strip.setPixelColor(stripMiddle - (numOfSegments - i), 0);
      else
        left_strip.setPixelColor(stripMiddle - i, 0);
        
      if (stripsOn2Pins)
        right_strip.setPixelColor(i, 0);
      else
        left_strip.setPixelColor(stripMiddle + i, 0);

      left_strip.show();
      if (stripsOn2Pins)
        right_strip.show();  
      
      delay(startupAnimationDelay);
    }
  }
}

void displayNumber (int number, int displayDelay) {
  left_strip.clear();
  if (stripsOn2Pins)
    right_strip.clear();

  number++; // @EB_DEBUG : display value 0 at led 1
  
  for (i = 0; i <= number; i++) {
    if (i % 5 == 0) 
      colorValue = stripMiddleColor;
    else
      colorValue = stripColor[0];

    left_strip.setPixelColor(middleOffset + i, colorValue);

    if (stripsOn2Pins)
      right_strip.setPixelColor(middleOffset + i, colorValue);
    else
      left_strip.setPixelColor(stripMiddle + middleOffset + i, colorValue);

    delay(45 - number * 3); // @EB_DEBUG
    
    left_strip.show();
    if (stripsOn2Pins)
      right_strip.show();  
  }

  if (pulsing) {
    left_strip.setPixelColor(middleOffset + maxDisplaySegments, stripMiddleColor);

    if (stripsOn2Pins)
      right_strip.setPixelColor(maxDisplaySegments, stripMiddleColor);
    else
      left_strip.setPixelColor(stripMiddle + maxDisplaySegments, stripMiddleColor);

    left_strip.show();
    if (stripsOn2Pins)
      right_strip.show();  
  }

  delay(displayDelay);

  left_strip.clear();
  if (stripsOn2Pins)
    right_strip.clear();
}


//
// for debugging
//

#ifdef DEBUG_TEST_LEDS
  void displayTest() {
    for (i = 0; i <= numOfSegments; i++) {
      left_strip.setPixelColor(stripMiddle - i, stripColor[i]);

      if (stripsOn2Pins)
        right_strip.setPixelColor(i, stripColor[i]);
      else
        left_strip.setPixelColor(stripMiddle + i, stripColor[i]);

      left_strip.show();
      if (stripsOn2Pins)
        right_strip.show();  

      delay(50);
    }
    delay(5000);
  
    for (i = 0; i <= numOfSegments; i++) {
      left_strip.setPixelColor(stripMiddle - i, 0);

      if (stripsOn2Pins)
        right_strip.setPixelColor(i, 0);
      else
        left_strip.setPixelColor(stripMiddle + i, 0);

      left_strip.show();
      if (stripsOn2Pins)
        right_strip.show();  
    }
  }
  
  void serialDisplayRGB(int r, int g, int b) {
    Serial.print(i);
    Serial.print(" ");
    Serial.print(r);
    Serial.print(" ");
    Serial.print(g);
    Serial.print(" ");
    Serial.println(b);
  }
#endif
