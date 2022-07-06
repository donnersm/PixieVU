/********************************************************************************************************************************************************
*                                                                                                                                                       *
   Project:         Stereo VU meter with Webserver, made for an Element14 project
   This is open source GNu licenced. In short: Do whatever you want with it but give credit to the orignal author Mark Donners
   Use at your own risk, the author will not take any responsibility for your actions with or without the use of this software.
   Target Platform: ESP32
*                                                                                                                                                       *
   Version: 1.0
   Hardware setup: See github
   
   Libraries/ Versions I used:
   FastLED_NeoMatrix at version 1.1         --> install with the library manager
   Framebuffer_GFX at version 1.0           --> install with the library manager
   FastLED at version 3.4.0                 --> install with the library manager
   Adafruit_GFX_Library at version 1.10.4   --> install with the library manager
   EasyButton at version 2.0.1              --> install with the library manager
   Adafruit_BusIO at version 1.7.1          --> install with the library manager
   Wire at version 1.0.1                    --> part of the ESP32 Framework, no need to install
   SPI at version 1.0                       --> part of the ESP32 Framework, no need to install

   You also need to install the ESP32 framework
   Go to File->Preferences and add the following line to the additional board manager:
   https://dl.espressif.com/dl/package_esp32_index.json
   Then go to Tools-> Board Mananger and find and install ESP32 board library

*                                                                                                                                                       *
   Mark Donners
   The Electronic Engineer
   Website:   www.theelectronicengineer.nl
   facebook:  https://www.facebook.com/TheelectronicEngineer
   youtube:   https://www.youtube.com/channel/UCm5wy-2RoXGjG2F9wpDFF3w
   github:    https://github.com/donnersm
*                                                                                                                                                       *
*********************************************************************************************************************************************************
  Version History
  1.0 release
*/

#define VERSION     "Version 1.0"
#include <FastLED_NeoMatrix.h>
#include <Adafruit_GFX.h>
#include <pixeltypes.h>
#include <math.h>
#include <EasyButton.h>


/*
Before we start with setup and main function, let's define the variables and all the settings.
Not all are intresting enough for you to change but the most important are mentioned first
Also, the color paterns are defined at the last section of this file
*/

// Tell the software how many leds you are using per channel.
// Change it to match your setup
 const uint8_t kMatrixHeight = 18;                  // Matrix height --> number of leds per column

// Now what kind of leds are we using?
#define CHIPSET         WS2812B                     // LED strip type
#define COLOR_ORDER     GRB                         // If colours look wrong, play with this
#define LED_PIN         27                          // LED strip data
#define LED_VOLTS       5                           // Usually 5 or 12
#define MAX_MILLIAMPS   2000                        // Careful with the amount of power here if running off USB port


//Other setting you can play around with
//Options Change to your likings
#define BottomRowAlwaysOn   0                       // if set to 1, bottom row is always on. Setting only applies to LEDstrip not HUB75
#define Fallingspeed        30                      // Falling down factor that effects the speed of falling tiles
int PEAKDELAY =             60;                     // Delay before peak falls down to stack. Overruled by PEAKDEALY Potmeter
int SENSE =                 60;                     // Sensitivity 
#define GAIN_DAMPEN         2                       // Higher values cause auto gain to react more slowly
#define SecToChangePattern  10                      // number of seconds that pattern changes when auto change mode is enabled
#define InputBoost          1000                    // this is the SEnse potmeter amplification. Higher number for low level input
#define Speedfilter         20                      // slowdown factor for columns to look less 'nervous' The higher the quicker
int buttonPushCounter     = 0;                      // This number defines what pattern to start after boot (0 to 12)
bool autoChangePatterns   = false;                  // After boot, the pattern will not change automatically. 
int noise                 = 100;                    // this will effect the upper bands most.
#define DemoAfterSec        6000                    // if there is no input signal during this number of milli seconds, the unit will go to demo mode
#define DemoTreshold        200                     // this defines the treshold that will get the unit out of demo mode
#define BRIGHTNESSMAX       100                     // Max brightness of the leds...carefull...to bright might draw to much amps!
int BRIGHTNESS            = 50;                     // Default brightnetss, however, overruled by the Brightness potmeter
int BRIGHTNESSMIN         = 1;                      // Min brightness
#define NumberOfModes       12

// Until here...not any further....do not change anything below this line unless you know what you are doing */
//Controls  //don't change unless you are using your own hardware design

// Arduino pin number where the button is connected.
#define MODE_BUTTON_PIN     25
#define SELECT_BUTTON_PIN   26
#define LONG_PRESS_MS       3000                    // Number of ms to count as a long press on the switch

#define BRIGHTNESSPOT       39
#define PEAKDELAYPOT        34
#define SENSEPOT            35
#define LeftChannelInput    32
#define RightChannelInput   33


// Other stuff don't change
#define numBands 2                                  // we only need left and right channel
#define offsetBar   0                               //if you need the whole display to start on a different column, you can give an offset. 
#define BAR_WIDTH  1                                // If width >= 8 light 1 LED width per bar, >= 16 light 2 LEDs width bar etc
#define TOP            (kMatrixHeight - 0)          // Don't allow the bars to go offscreen
#define NUM_LEDS   (kMatrixWidth * kMatrixHeight)   // Total number of LEDs
const uint8_t kMatrixWidth = numBands;              // Matrix width --> number of columns in your led matrix
char PeakFlag[numBands];                            // the top peak delay needs to have a flag because it has different timing while floating compared to falling to the stack
int PeakTimer[numBands];                            // counter how many loops to stay floating before falling to stack
uint8_t colorTimer = 0;
byte peak[2]            = {0,0};                    // The length of these arrays must be >= NUM_BANDS
int oldBarHeights[2]    = {0,0};                    // so they are set to 11
int PeakToPeak[2]       = {0,0};
int OldPeakValues[2]    = {0,0};
int PeakBins[2]         = {0,0};


/****************************************************************************
 * Colors of bars and peaks in different modes, changeable to your likings  *
 ****************************************************************************/
// Colors mode 0
#define ChangingBar_Color   y * (255 / kMatrixHeight) + colorTimer, 255, 255
// no peaks

// Colors mode 1 These are the colors from the TRIBAR when using Ledstrip
#define TriBar_Color_Top      0 , 255, 255          // Red CHSV
#define TriBar_Color_Bottom   95 , 255, 255         // Green CHSV
#define TriBar_Color_Middle   45, 255, 255          // Yellow CHSV

#define TriBar_Color_Top_Peak      0 , 255, 255     // Red CHSV
#define TriBar_Color_Bottom_Peak   95 , 255, 255    // Green CHSV
#define TriBar_Color_Middle_Peak   45, 255, 255     // Yellow CHSV

// Colors mode 1 These are the colors from the TRIBAR when using HUB75
#define TriBar_RGB_Top      255 , 0, 0              // Red CRGB
#define TriBar_RGB_Bottom   0 , 255, 0              // Green CRGB
#define TriBar_RGB_Middle   255, 255, 0             // Yellow CRGB

#define TriBar_RGB_Top_Peak      255 , 0, 0         // Red CRGB
#define TriBar_RGB_Bottom_Peak   0 , 255, 0         // Green CRGB
#define TriBar_RGB_Middle_Peak   255, 255, 0        // Yellow CRGB

// hub 75 center bars
#define Center_RGB_Edge      255 , 0, 0             // Red CRGB
#define Center_RGB_Middle   255, 255, 0             // Yellow CRGB
// hub 75 center bars 2
#define Center_RGB_Edge2      255 , 0, 0            // Red CRGB
#define Center_RGB_Middle2   255, 255, 255          // Yellow CRGB

// Colors mode 2
#define RainbowBar_Color  (x / BAR_WIDTH) * (255 / numBands), 255, 255
#define PeakColor1  0, 0, 255                       // white CHSV

// Colors mode 3
#define PeakColor2  0, 0, 255                       // white CHSV
DEFINE_GRADIENT_PALETTE( purple_gp ) {
  0,   0, 212, 255,                                 //blue
255, 179,   0, 255 };                               //purple
CRGBPalette16 purplePal = purple_gp;


// Colors mode 4
#define SameBar_Color1      0 , 255, 255            //red  CHSV
#define PeakColor3  160, 255, 255                   // blue CHSV

// Colors mode 5
#define SameBar_Color2      160 , 255, 255          //blue  CHSV
#define PeakColor4  0, 255, 255                     // red CHSV

// Colors mode 6
DEFINE_GRADIENT_PALETTE( redyellow_gp ) {  
  0,   200, 200,  200,                              //white
 64,   255, 218,    0,                              //yellow
128,   231,   0,    0,                              //red
192,   255, 218,    0,                              //yellow
255,   200, 200,  200 };                            //white
CRGBPalette16 heatPal = redyellow_gp;
// no peaks

// Colors mode 7
DEFINE_GRADIENT_PALETTE( outrun_gp ) {
  0, 141,   0, 100,                                 //purple
127, 255, 192,   0,                                 //yellow
255,   0,   5, 255 };                               //blue
CRGBPalette16 outrunPal = outrun_gp;
// no peaks

// Colors mode 8
DEFINE_GRADIENT_PALETTE( mark_gp2 ) {
  0,   255,   218,    0,                            //Yellow
 64,   200, 200,    200,                            //white
128,   141,   0, 100,                               //pur
192,   200, 200,    200,                            //white
255,   255,   218,    0,};                          //Yellow
CRGBPalette16 markPal2 = mark_gp2;

// Colors mode 9
// no bars only peaks
DEFINE_GRADIENT_PALETTE( mark_gp ) {
  0,   231,   0,    0,                              //red
 64,   200, 200,    200,                            //white
128,   200, 200,    200,                            //white
192,   200, 200,    200,                            //white
255,   231, 0,  0,};                                //red
CRGBPalette16 markPal = mark_gp;

// Colors mode 10
// no bars only peaks
#define PeakColor5  160, 255, 255                   //blue CHSV

// Colors mode 11
#define SameBar_Orange      32 , 83, 96            //red  CHSV

// These are the colors from the TRIPEAK mode 11
// no bars
#define PeakColor11  0 , 255, 255                   //red CHSV
#define TriBar_Color_Top_Peak2      0 , 255, 255    //Red CHSV
#define TriBar_Color_Bottom_Peak2   95 , 255, 255   //Green CHSV
#define TriBar_Color_Middle_Peak2   45, 255, 255    //Yellow CHSV
/****************************************************/

#define up  1
#define down 0
int PeakDirection = 0;
long LastDoNothingTime = 0;                          // only needed for screensaver
int DemoModeMem = 0;                                 // to remember what mode we are in when going to demo, in order to restore it after wake up
bool AutoModeMem = false;                            // same story
bool DemoFlag = false;                               // we need to know if demo mode was manually selected or auto engadged.
bool webtoken = false;
const int sampleWindow = 10;                         // Sample window width in mS (50 mS = 20Hz)
const int sampleWindow1 = 10;                        // Sample window width in mS (50 mS = 20Hz)
int val;
unsigned int sample;
unsigned int sample1;




/**********************************************************
 * 
 *        Now let's do some programming!
 *        
 * ********************************************************/




// Button.
EasyButton ModeBut(MODE_BUTTON_PIN);
EasyButton SelBut(SELECT_BUTTON_PIN);

// Callback.
void onPressed() {
  Serial.println("Mode Button has been pressed!");
  buttonPushCounter = (buttonPushCounter + 1) % NumberOfModes; //%6
  Serial.printf("New mode: %d\n",buttonPushCounter);
  FastLED.clear();
  //SetInput(MIC);
}

void ChangePeakDirection() {
  PeakDirection = !PeakDirection;
}
// define the pixelleds
CRGB leds[NUM_LEDS];

 // this one is used if you are using a ledstrip setup simular to the one from the acryllic spectrum analyzer
  FastLED_NeoMatrix *matrix = new FastLED_NeoMatrix(leds, kMatrixWidth, kMatrixHeight,
                                  NEO_MATRIX_TOP        + NEO_MATRIX_LEFT +
                                  NEO_MATRIX_COLUMNS       + NEO_MATRIX_PROGRESSIVE +
                                  NEO_TILE_TOP + NEO_TILE_LEFT + NEO_TILE_ROWS);


void setup() {
  Serial.begin(115200);
  Serial.print("Main program is  running on core ");
  Serial.println(xPortGetCoreID());
  // Initialize the button.
  ModeBut.begin();
  SelBut.begin();
  // Attach callback.
  ModeBut.onPressed(onPressed);
  SelBut.onPressedFor(LONG_PRESS_MS, startAutoMode);
  // ModeBut.onSequence(3, 600, VUMeterToggle);
  SelBut.onSequence(2, 500, ChangePeakDirection);
  Serial.println("Original program written by Mark Donners, aka The Electronic Engineer.");
  Serial.println(VERSION);
  Serial.println("www.theelectronicengineer.nl");
  Serial.println("email: Mark.Donners@judoles.n");
  SetupLEDSTRIP();
}

void loop() {
  // wm.process();
  size_t bytesRead = 0;
  int TempADC = 0;

  //############ Step 1: Handle user interface ##################

  // read potmeters and process
  PEAKDELAY = map(analogRead(PEAKDELAYPOT), 0, 4095, Fallingspeed,100);
  

 // BRIGHTNESS = map(analogRead(BRIGHTNESSPOT), 0, 4095, BRIGHTNESSMAX, BRIGHTNESSMIN);
  SENSE = map(analogRead(SENSEPOT), 0, 4095,  InputBoost,1);

  FastLED.setBrightness(BRIGHTNESS);

  // Continuously update the button state.
  ModeBut.read();
  SelBut.read();

  //############ Step 2: get the audio amplitude and transform it to a VU value  ##################
  Analyser_ALL(); // call function to read all channels to fill FreqBinsNew[]

  //############ Step 3:Process the VU value we just aquired ##################

  // compare to previous values. If gone up then oldvalue is overwritten with new value
  // if gone down then newvalue is oldvalue-50 ( this will reduce noise glitches and smooths out the display)
  float averageSum = 0;

  for (int cnt = 0; cnt < numBands; cnt++) {
    if (PeakToPeak[cnt] < OldPeakValues[cnt]) {
      PeakBins[cnt] = max(OldPeakValues[cnt] - Speedfilter, PeakToPeak[cnt]);
    }
    else if (PeakToPeak[cnt] > OldPeakValues[cnt]) {
      PeakBins[cnt] = PeakToPeak[cnt];
    }

    OldPeakValues[cnt] = PeakBins[cnt];
    // using this same loop to do some processing
    // like scaling the freqbins to matrix height
    PeakBins[cnt] = map(PeakBins[cnt], 0, SENSE, 0, kMatrixHeight);
  }

  //############ Step 3:Process the bin data( left n right VU) into bar height ##################

  for (int band = 0; band < numBands; band++) {
    int barHeight = PeakBins[band];

    // Small amount of averaging between frames
    barHeight = ((oldBarHeights[band] * 1) + barHeight) / 2;

    // Move peak up
    if (barHeight > peak[band]) {
      peak[band] = min(TOP, barHeight);
      PeakFlag[band] = 1;
    }

#if BottomRowAlwaysOn
    if (barHeight == 0)barHeight = 1; // make sure there is always one bar that lights up
#endif
    // Now visualize those bar heights
    switch (buttonPushCounter) {
      case 0:
        changingBarsLS(band, barHeight);
        break;

      case 1:
        TriBarLS(band, barHeight);
        TriPeakLS(band);
        break;
        
      case 2:
        rainbowBarsLS(band, barHeight);
        NormalPeakLS(band, PeakColor1);
        break;
        
      case 3:
        purpleBarsLS(band, barHeight);
        NormalPeakLS(band, PeakColor2);
        break;
        
      case 4:
        SameBarLS(band, barHeight);
        NormalPeakLS(band, PeakColor3);
        break;
        
      case 5:
        SameBar2LS(band, barHeight);
        NormalPeakLS(band, PeakColor3);
        break;
        
      case 6:
        centerBarsLS(band, barHeight);
        break;
        
      case 7:
        centerBars2LS(band, barHeight);
        break;
        
      case 8:
        //PeakDirection=up;
        centerBars3LS(band, barHeight);
        break;
        
      case 9:
        BlackBarLS(band, barHeight);
        outrunPeakLS(band);
        break;
        
      case 10:
        BlackBarLS(band, barHeight);
        NormalPeakLS(band, PeakColor5);
        break;
        
      case 11:
        OrangeBarLS(band, barHeight);
        NormalPeakLS(band, PeakColor11);
        break;
    }
    // Save oldBarHeights for averaging later
    oldBarHeights[band] = barHeight;
  }
  //############ Step 4:Other Stuff ##################
  // Decay peak
  EVERY_N_MILLISECONDS(Fallingspeed) {
    for (byte band = 0; band < numBands; band++) {
      if (PeakFlag[band] == 1) {
        PeakTimer[band]++;
        if (PeakTimer[band] > PEAKDELAY) {
          PeakTimer[band] = 0;
          PeakFlag[band] = 0;
        }
      }
      else if ((peak[band] > 0) && (PeakDirection == up)) {
        peak[band] += 1;
        if (peak[band] > (kMatrixHeight + 10))peak[band] = 0;
      } // when to far off screen then reset peak height
      else if ((peak[band] > 0) && (PeakDirection == down)) {
        peak[band] -= 1;
      }
    }
    colorTimer++;
  }

  EVERY_N_MILLISECONDS(10)colorTimer++; // Used in some of the patterns

  EVERY_N_SECONDS(SecToChangePattern) {
    if (autoChangePatterns) {
      buttonPushCounter = (buttonPushCounter + 1) % 12;
    }
  }
  delay(1); // needed to give fastled a minimum recovery time
  FastLED.show();

} // loop end

/****************end of main loop ************************/

void SetupLEDSTRIP(void)
{
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setMaxPowerInVoltsAndMilliamps(LED_VOLTS, MAX_MILLIAMPS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();  
}


void Analyser_ALL() {
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int signalMax = 100;
  unsigned int signalMin = 2000;
  unsigned int signalMax1 = 100;
  unsigned int signalMin1 = 2000;

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(LeftChannelInput); //links
    sample1 = analogRead(RightChannelInput); //links
    if (sample > signalMax)signalMax = sample;  // save just the max levels
    if (sample < signalMin)signalMin = sample;  // save just the min levels
    if (sample1 > signalMax1)signalMax1 = sample1;  // save just the max levels
    if (sample1 < signalMin1)signalMin1 = sample1;  // save just the min levels
  }
  PeakToPeak[0] = signalMax - signalMin;  // max - min = peak-peak amplitude
  PeakToPeak[1] = signalMax1 - signalMin1;  // max - min = peak-peak amplitude
}


void startAutoMode() {
  autoChangePatterns = true;
  // Matrix_Flag();                  //this is to show user that automode was engaged. It will show dutch flag for 2 seconds
  delay(5000);
  Serial.println(" Patterns will change after few seconds ");
  Serial.println(" You can reset by pressing the mode button again");
}



/********************************************************************************************************************************
 * ** SUB Rountines related to Paterns and Peaks                                                                               **
 ********************************************************************************************************************************/


//************ Mode 0 ***********
 void changingBarsLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        matrix -> drawPixel(x, y, CHSV(ChangingBar_Color));
     }
     else {
      matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
    } 
  }
}

//************ Mode 1 ***********
 void TriBarLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        if (y < (kMatrixHeight/3)) matrix -> drawPixel(x, y, CHSV(TriBar_Color_Top));     //Top red
      else if (y > (1 *kMatrixHeight/2)) matrix -> drawPixel(x, y, CHSV(TriBar_Color_Bottom)); //green
      else matrix -> drawPixel(x, y, CHSV(TriBar_Color_Middle));      //yellow
     }
     else {
      
      matrix->drawPixel(x, y, CRGB(0, 0, 0).fadeToBlackBy(200)); // make unused pixel in a band black
     }
    } 
  }
}

void TriPeakLS(int band) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  int peakHeight = TOP - peak[band] - 1;
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    if (peakHeight < 4) matrix -> drawPixel(x, peakHeight, CHSV(TriBar_Color_Top_Peak)); //Top red
    else if (peakHeight > 8) matrix -> drawPixel(x, peakHeight, CHSV(TriBar_Color_Bottom_Peak)); //green
    else matrix -> drawPixel(x, peakHeight, CHSV(TriBar_Color_Middle_Peak)); //yellow
  }
}
//************ Mode 2 ***********
 void rainbowBarsLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        matrix -> drawPixel(x, y, CHSV(RainbowBar_Color));
     }
     else {
      matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
    } 
  }
}

void NormalPeakLS(int band, int H, int S, int V) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  int peakHeight = TOP - peak[band] - 1;
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    matrix -> drawPixel(x, peakHeight, CHSV(H, S, V));
  }
}

//************ Mode 3 ***********

void purpleBarsLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        matrix -> drawPixel(x, y, ColorFromPalette(purplePal, y * (255 / (barHeight + 1))));
     }
     else {
      matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
    } 
  }
}

// for peaks see mode 2

//************ Mode 4 ***********

void SameBarLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        matrix -> drawPixel(x, y, CHSV(SameBar_Color1)); //blue
     }
     else {
      matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
    } 
  }
}
// for peaks see mode 2

//************ Mode 5 ***********
void SameBar2LS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        matrix -> drawPixel(x, y, CHSV(SameBar_Color2)); //blue
     }
     else {
      matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
    } 
  }
}
// for peaks see mode 2

//************ Mode 6 ***********
void centerBarsLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  int center= TOP/2;
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    if (barHeight % 2 == 0) barHeight--;
    if (barHeight < 0) barHeight = 1; // at least a white line in the middle is what we want
    int yStart = ((kMatrixHeight - barHeight) / 2);
    for (int y = yStart; y <= (yStart + barHeight); y++) {
      int colorIndex = constrain((y - yStart) * (255 / barHeight), 0, 255);
      matrix -> drawPixel(x, y, ColorFromPalette(heatPal, colorIndex));
    }
    for (int y= barHeight/2;y<TOP;y++){
      matrix->drawPixel(x, center+y+1, CRGB(0, 0, 0)); // make unused pixel bottom black
      matrix->drawPixel(x, center-y-2, CRGB(0, 0, 0)); // make unused pixel bottom black
    }
    
  }
}

//************ Mode 7 ***********
void centerBars2LS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  int center= TOP/2;
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    if (barHeight % 2 == 0) barHeight--;
    if (barHeight < 0) barHeight = 1; // at least a white line in the middle is what we want
    int yStart = ((kMatrixHeight - barHeight) / 2);
    for (int y = yStart; y <= (yStart + barHeight); y++) {
      int colorIndex = constrain((y - yStart) * (255 / barHeight), 0, 255);
      matrix -> drawPixel(x, y, ColorFromPalette(markPal, colorIndex));
    }
    for (int y= barHeight/2;y<TOP;y++){
      matrix->drawPixel(x, center+y+1, CRGB(0, 0, 0)); // make unused pixel bottom black
      matrix->drawPixel(x, center-y-2, CRGB(0, 0, 0)); // make unused pixel bottom black
    }
    
  }
}

//************ Mode 8 ***********
void centerBars3LS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  int center= TOP/2;
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    if (barHeight % 2 == 0) barHeight--;
    if (barHeight < 0) barHeight = 1; // at least a white line in the middle is what we want
    int yStart = ((kMatrixHeight - barHeight) / 2);
    for (int y = yStart; y <= (yStart + barHeight); y++) {
      int colorIndex = constrain((y - yStart) * (255 / barHeight), 0, 255);
      matrix -> drawPixel(x, y, ColorFromPalette(markPal2, colorIndex));
    }
    for (int y= barHeight/2;y<TOP;y++){
      matrix->drawPixel(x, center+y+1, CRGB(0, 0, 0)); // make unused pixel bottom black
      matrix->drawPixel(x, center-y-2, CRGB(0, 0, 0)); // make unused pixel bottom black
    }
    
  }
}
//************ Mode 9 ***********
void BlackBarLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
     else {
      matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
    } 
  }
}

void outrunPeakLS(int band) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  int peakHeight = TOP - peak[band] - 1;
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    matrix -> drawPixel(x, peakHeight, ColorFromPalette(outrunPal, peakHeight * (255 / kMatrixHeight)));
  }
}

//************ Mode 10 ***********
void TriPeak2LS(int band) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  int peakHeight = TOP - peak[band] - 1;
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    if (peakHeight < 4) matrix -> drawPixel(x, peakHeight, CHSV(TriBar_Color_Top_Peak2)); //Top red
    else if (peakHeight > 8) matrix -> drawPixel(x, peakHeight, CHSV(TriBar_Color_Bottom_Peak2)); //green
    else matrix -> drawPixel(x, peakHeight, CHSV(TriBar_Color_Middle_Peak2)); //yellow
  }
}

//************ Mode 11 ***********
// for peaks see mode 1
//************ Mode 4 ***********

void OrangeBarLS(int band, int barHeight) {
  int xStart = offsetBar+(BAR_WIDTH * band);
  for (int x = xStart; x < xStart + BAR_WIDTH; x++) {
    for (int y = TOP; y >= 0; y--) {
     if(y >= TOP - barHeight){
        matrix -> drawPixel(x, y, CRGB(CRGB::Red)); //blue
     }
     else {
      matrix->drawPixel(x, y, CRGB(0, 0, 0)); // make unused pixel in a band black
     }
    } 
  }
}


/********************************************************************************************************************************
 * ** END SUB Rountines related to Paterns and Peaks                                                                           **
 ********************************************************************************************************************************/


 
