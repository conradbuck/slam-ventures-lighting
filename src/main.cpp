#include <Arduino.h>
#include <FastLED.h>
#include <NRFLite.h>
#include <SPI.h>
#include <HardwareSerial.h>

//leds
#define NUM_LEDS 140
#define NUM_STRIPS 14
//rotary
#define CLK 37 
#define DAT 39
#define SWT 40
//rocker
#define ROCK_UP 41
#define ROCK_DN 42
//lidar
//#define LIDAR_TX 2
//#define LIDAR_RX 1

//43 and 44 unused but are serial pins, maybe switch to them if UART necessary?
//is 14 unused???

#define BRIGHTNESS 100 //brightness ceiling
unsigned long seed = 93;
uint8_t FFACTOR = 2;
uint8_t COLORCLASS = 1;
uint8_t SOFTWARE_REVISION = 2;
uint8_t BOOT_PATTERN = 0;    //0 for chasing, 1 for slow pulse, 2 for instant on default white
DEFINE_GRADIENT_PALETTE( Green_3D_1_gp ) { //leaves
    0, 120,220,120,
   42,  45,147, 48,
   84,  13,114, 14,
  127,   2, 95,  2,
  170,   0, 80,  0,
  212,   0, 40,  0,
  255,   0, 50, 0};
DEFINE_GRADIENT_PALETTE( ColorTemp ) {
    0, 255,  61,   2,
  135, 255, 170,  62,
  180, 236, 197, 121,
  255, 134, 169, 255};
DEFINE_GRADIENT_PALETTE( Caribbean_Blues_gp ) {
    0,   0,  2, 66,
  102,   1, 51, 88,
  144,   3,169,114,
  163,  67,207,142,
  173, 255,248,174,
  176, 190,217,132,
  178, 159,199,111,
  181, 135,186, 95,
  183, 117,175, 82,
  186, 100,164, 71,
  188,  87,156, 62,
  191,  75,147, 53,
  194,  65,139, 47,
  196,  56,133, 41,
  199,  48,125, 35,
  201,  41,119, 30,
  204,  35,114, 26,
  206,  30,108, 22,
  209,  26,103, 19,
  211,  21, 97, 16,
  214,  18, 92, 13,
  216,  15, 88, 11,
  219,  12, 84,  9,
  221,   9, 80,  7,
  224,   7, 75,  6,
  227,   6, 72,  4,
  229,   4, 69,  3,
  232,   3, 65,  2,
  234,   2, 61,  2,
  237,   1, 58,  1,
  239,   1, 55,  1,
  242,   1, 53,  1,
  244,   1, 50,  1,
  247,   1, 47,  1,
  249,   1, 45,  1,
  252,   1, 42,  1,
  255,   0, 40,  0};

//0 80 120 160 190 220
DEFINE_GRADIENT_PALETTE( Special ) {
    0, 255,   0,   0, //red
   42,  86, 213,   0, //lime green
   84,   0, 191,  64, //cyan
  126,   0,   0, 255, //blue
  168,  80,   0, 175, //purple
  210, 160,   0,  96, //magenta
  255, 255,   0,   0}; //red

// Gradient palette "bhw1_sunset3_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/bhw/bhw1/tn/bhw1_sunset3.png.index.html
DEFINE_GRADIENT_PALETTE( bhw1_sunset3_gp ) {
    0, 227,237, 56,
   33, 186, 67,  1,
   71, 163, 21,  1,
   81, 157, 13,  1,
  188,  39, 21, 18,
  234,  12,  7,  4,
  255,  12,  7,  4};
  
  // Gradient palette "es_ocean_breeze_009_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/ocean_breeze/tn/es_ocean_breeze_009.png.index.html
DEFINE_GRADIENT_PALETTE( es_ocean_breeze_009_gp ) {
    0, 190,176,123,
   63, 184,231,250,
  255,   1, 50, 62};

// Gradient palette "bhw3_07_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/bhw/bhw3/tn/bhw3_07.png.index.html
DEFINE_GRADIENT_PALETTE( bhw3_07_gp ) {
    0, 132,  5,  1,
   30,  12, 13,147,
   53,  54, 32,166,
   73, 103, 39, 98,
   84, 239, 52, 27,
   96,  71,  2,  1,
  117,  46,  8, 33,
  130,  54, 32,166,
  163,  10,  7, 78,
  196, 239, 52, 27,
  255,   1,  1, 14};
//end of hard coded palettes

//palette declaration
//programatic palette right below
CRGBPalette16 colorScroll;
//absolute RGB declaration. See checkNewData()
CRGBPalette16 fullFive;
//hardcoded palettes
CRGBPalette16 leaves = Green_3D_1_gp;
CRGBPalette16 cbPal = Caribbean_Blues_gp;
CRGBPalette16 cTemp = ColorTemp;
CRGBPalette16 anchor = Special;

CRGBPalette16 warmDesertSunset = bhw1_sunset3_gp;
CRGBPalette16 coolDesertSunset = es_ocean_breeze_009_gp;
CRGBPalette16 purpleRedPinkMulti = bhw3_07_gp;

const static uint8_t RADIO_ID = 97;
const static uint8_t PIN_RADIO_CE = 35;
const static uint8_t PIN_RADIO_CSN = 36;
unsigned long lastPacketSent = 0;
struct __attribute__((packed)) RadioPacket {
    uint8_t brightKey;
    uint16_t zone;
    uint16_t qcomm; //quick command or pattern 
    uint16_t color;
    uint16_t speed;
    uint16_t extra;
    uint16_t rh;
    uint16_t gs;
    uint16_t bv;
    uint8_t rh2;
    uint8_t gs2;
    uint8_t bv2;
    uint8_t ext1;
    uint8_t ext2;
    uint8_t ext3;
    uint8_t ext4;
    uint8_t sw1;
    uint8_t sw2;
    uint16_t randfactor;
    uint8_t formfactor; //Specify whether a command is meant for a strip or pod or shadow projector, etc.
    uint8_t cclass;
    uint8_t rev;
};

bool engineActive = 0;
unsigned long lastKnAct = 0;
int rdelta = 0;
CRGB offMode[NUM_LEDS];

//array that holds values 0-255 to represent places within a palette. randomized upon startup. see _.qcomm = 15
uint8_t colorIndex[NUM_LEDS];
//used in delayTime() to allow for constant checking of new data
unsigned long difference;
unsigned long difference2;
//used to delay the number of times the radio can be checked
unsigned long lastTimeChecked;
int checkInterval = 2;
uint8_t paletteIndex;
int delta = 0;
uint8_t numAnchorPoints = 4;
bool circularPalette = 0;

//basic color manipulation
uint8_t ghue = 0;
int currBrightness = BRIGHTNESS;
//addressing
int addr = 0;
long UNIV_NUMBER = 65535;
//agnum //for PCBDIP only
int agnum = 0;
//for advanced stripwise functions
int stripIndex = 0;
//sine curves
uint8_t sinA = 0;
uint8_t sinB = 0;

//runtime convenience
bool powerStatus = 0;
bool firstRun = 1;
bool protectedRedo = 0;

NRFLite _satellite;
RadioPacket _baseData;
RadioPacket _baseDataTemp;
CRGB leds[NUM_LEDS];
CRGB defaultColor = CRGB::SeaGreen;

//rotate knob to change this during normal operation, sets dominant color for motion patterns and solid color for person tracking
CRGB activeColor = CRGB::Purple;

//change the indicies in this array to fill each LED strip with a different solid color, use ledsStripesProject(); to display it
CRGB stripeColors[NUM_STRIPS]; //use this to hold specific colors for each strip to be fill_solid with
uint8_t stripeIndex[NUM_STRIPS]; //use this to hold the index of a palette to then fill_solid(ColorFromPalette()) with

CRGB ledsMatrix[NUM_STRIPS][NUM_LEDS];

void ledsProject() {
  for(int i = 0; i < NUM_STRIPS; i++) {
    for(int j = 0; j < NUM_LEDS; j++) {
      ledsMatrix[i][j] = leds[j];
    }
  }
  FastLED.show();
}
void ledsStripesProject() {
  for(int i = 0; i < NUM_STRIPS; i++) {
    fill_solid(ledsMatrix[i], NUM_LEDS, stripeColors[i]);
  }
  FastLED.show();
}

CRGB hsvToRgb(const CHSV& hsv) {
  CRGB color = CHSV(hsv.h, hsv.s, hsv.v);
  return color;
}
int queryNextBreak(uint8_t palIndex, uint8_t ancPoints, int direc) {
  int* palEval = nullptr;
  int twoBp[2] = {0, 240};
  int threeBp[3] = {0, 127, 240};
  int fourBp[4] = {0, 80, 168, 240};  
  int fiveBp[5] = {0, 63, 127, 191, 240};
  int sixBp[6] = {0, 50, 101, 152, 203, 240};
  int sevenBp[7] = {0, 42, 84, 126, 168, 210, 240};
  int eightBp[8] = {0, 35, 70, 106, 142, 177, 213, 240};
  switch(ancPoints) {
    case 2:
      palEval = twoBp;
      break;
    case 3:
      palEval = threeBp;
      break;
    case 4:
      palEval = fourBp;
      break;
    case 5:
      palEval = fiveBp;
      break;
    case 6:
      palEval = sixBp;
      break;
    case 7:
      palEval = sevenBp;
      break;
    case 8:
      palEval = eightBp;
      break;
    default:
      palEval = threeBp;
      break;
  }
  if(direc == 2) {
    return palEval[random(0, ancPoints)];
  } else if(direc == 1) { //up
    //if (paletteIndex == palEval[sizeof(palEval)/sizeof(int) - 1]) {
    if (paletteIndex == 240) {
      if(circularPalette) {
        //circular
        paletteIndex = 0;
        return palEval[1];
      } else {
        //non-circular
        //return palEval[sizeof(palEval)/sizeof(int) - 1];
        return 240;
      }
    } else {
      for(int i = 0; i < ancPoints - 1; i++) {
        if(palEval[i] == palIndex) {
          return palEval[i+1];
        }
      }
    }
  } else { //down
    if(paletteIndex == 0) {
      if(circularPalette) {
        //circular
        //paletteIndex = palEval[sizeof(palEval)/sizeof(int) - 1];
        paletteIndex = 240;
        return palEval[ancPoints - 2];
      } else {
        //non-circular
        return 0;
      }
    } else {
      for(int i = ancPoints - 1; i >= 0; i--) {
        if(palEval[i] == palIndex) {
          return palEval[i-1];
        }
      }
    }
  }
  return 0;
}

//programatic palette assembly
void fillNewPalette(uint8_t hue, uint8_t magnitude, bool hsVariance) {
  const CHSV tempCHSVhueVar[16];
  const CHSV tempCHSVsatVar[16];
  uint8_t sat = 255;
  uint8_t val = 255;
  if(hsVariance == 0) {
    CHSVPalette16 h = CHSVPalette16(
      CHSV(hue-(magnitude*6), sat, val),
      CHSV(hue-(magnitude*5), sat, val),
      CHSV(hue-(magnitude*4), sat, val),
      CHSV(hue-(magnitude*3), sat, val),
      CHSV(hue-(magnitude*2), sat, val),
      CHSV(hue-(magnitude*1), sat, val),
      CHSV(hue, sat, val),
      CHSV(hue, sat, val),
      CHSV(hue, sat, val),
      CHSV(hue, sat, val),
      CHSV(hue+(magnitude*1), sat, val),
      CHSV(hue+(magnitude*2), sat, val),
      CHSV(hue+(magnitude*3), sat, val),
      CHSV(hue+(magnitude*4), sat, val),
      CHSV(hue+(magnitude*5), sat, val),
      CHSV(hue+(magnitude*6), sat, val)
    );
    colorScroll = h;
  } else if(hsVariance == 1) {
    CHSVPalette16 h = CHSVPalette16(
      CHSV(hue, sat, val),
      CHSV(hue, sat, val),
      CHSV(hue, sat-(magnitude), val),
      CHSV(hue, sat-(magnitude), val),
      CHSV(hue, sat-(magnitude*3), val),
      CHSV(hue, sat-(magnitude*5), val),
      CHSV(hue, sat-(magnitude*8), val),
      CHSV(hue, sat-(magnitude*10), val),
      CHSV(hue, sat-(magnitude*15), val),
      CHSV(hue, sat-(magnitude*10), val),
      CHSV(hue, sat-(magnitude*8), val),
      CHSV(hue, sat-(magnitude*5), val),
      CHSV(hue, sat-(magnitude*3), val),
      CHSV(hue, sat-(magnitude), val),
      CHSV(hue, sat, val),
      CHSV(hue, sat, val)
    );
    colorScroll = h;
  }
}
void dipoleFadePaletteCHSV(uint8_t a, uint8_t b, uint8_t c, uint8_t a2, uint8_t b2, uint8_t c2) {
  colorScroll = CHSVPalette16(CHSV(a,b,c),CHSV(a2,b2,c2));
}
void dipoleFadePaletteCRGB(uint8_t a, uint8_t b, uint8_t c, uint8_t a2, uint8_t b2, uint8_t c2) {
  colorScroll = CRGBPalette16(CRGB(a,b,c), CRGB(a2,b2,c2));
}

//double ended
void ovalFadePalette(uint8_t a, uint8_t b) {
  colorScroll = CHSVPalette16(CHSV(a,255,255),CHSV(b,255,255),CHSV(a,255,255));
}
void ovalFadePaletteCRGB(uint8_t a, uint8_t b, uint8_t c, uint8_t a2, uint8_t b2, uint8_t c2) {
  colorScroll = CRGBPalette16(CRGB(a,b,c),CRGB(a2,b2,c2),CRGB(a,b,c));
}
void triangleFadePalette(uint8_t a,uint8_t b, uint8_t c) {
  colorScroll = CHSVPalette16(CHSV(a,255,255),CHSV(b,255,255),CHSV(c,255,255),CHSV(a,255,255));
}

void twoColorPalette(uint8_t a, uint8_t b) {
  colorScroll = CHSVPalette16(CHSV(a,255,255),CHSV(b,255,255));
}
void threeColorPalette(uint8_t a,uint8_t b, uint8_t c) {
  colorScroll = CHSVPalette16(CHSV(a,255,255),CHSV(b,255,255),CHSV(c,255,255));
}
void fourColorPalette(uint8_t a,uint8_t b, uint8_t c, uint8_t d) {
  colorScroll = CHSVPalette16(CHSV(a,255,255),CHSV(b,255,255),CHSV(c,255,255),CHSV(d,255,255));
}
void blackIsolationPalette(uint8_t a,uint8_t b, uint8_t c) {
  colorScroll = CHSVPalette16(CHSV(0,0,0),CHSV(a,b,c),CHSV(0,0,0));
}
void hueSatOnlyPalette(uint8_t a, uint8_t a2, uint8_t b, uint8_t b2, uint8_t c, uint8_t c2) {
  colorScroll = CHSVPalette16(CHSV(a,a2,255), CHSV(b,b2,255), CHSV(c,c2,255));
}
void hueSatOnlyPaletteCircular(uint8_t a, uint8_t a2, uint8_t b, uint8_t b2, uint8_t c, uint8_t c2) {
  colorScroll = CHSVPalette16(CHSV(a,a2,255), CHSV(b,b2,255), CHSV(c,c2,255), CHSV(a,a2,255));
}


int rockerState = 0;
int prevRockerState = 0;
CRGB oboard[1];
void initRocker() {
  pinMode(ROCK_UP, INPUT_PULLUP);
  pinMode(ROCK_DN, INPUT_PULLUP);

}
void pollRocker() {
  if(!digitalRead(ROCK_UP)) {
    rockerState = -1;
  } else if(!digitalRead(ROCK_DN)) {
    rockerState = 1;
  } else {
    rockerState = 0;
  }
  Serial.println(rockerState);
  delay(20);
}

int rangeNumber(int num, int lower, int upper) { //range a number between lower (inclusive) and upper (exclusive)
  if(num < lower) {
    return lower;
  } else if(num >= upper) {
    return upper - 1;
  } else {
    return num;
  }
  
}
int rangeNumberCircular(int num, int lower, int upper) { //circulate a number between lower (inclusive) and upper (exclusive)
  if(num < lower) {
    return upper - 1;
  } else if(num >= upper) {
    return lower;
  } else {
    return num;
  }
  
}

void initRadio() {
  SPI.begin(48,47,21);  // SCK, MISO, MOSI
  if(!_satellite.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE2MBPS)) {
    while (1) {
      leds[0] = CRGB::Red;
      ledsProject();
      delay(500);
      leds[0] = CRGB::Black;
      ledsProject();
      delay(500);
    }
  }
}
uint8_t paletteSelect(uint8_t n) {
  //return statement sets ancPoints
  int a = 3;
  switch(n) {
    case 0:
      circularPalette = 0;
      dipoleFadePaletteCHSV(_baseData.rh, _baseData.gs, _baseData.bv, _baseData.rh2, _baseData.gs2, _baseData.bv2);
      a = 2;
      break;
    case 1:
      circularPalette = 0;
      dipoleFadePaletteCRGB(_baseData.rh, _baseData.gs, _baseData.bv, _baseData.rh2, _baseData.gs2, _baseData.bv2);
      a = 2;
      break;
    case 2:
      circularPalette = 1;
      ovalFadePalette(_baseData.rh, _baseData.gs);
      a = 3;
      break;
    case 3:
      circularPalette = 1;
      ovalFadePaletteCRGB(_baseData.rh, _baseData.gs, _baseData.bv, _baseData.rh2, _baseData.gs2, _baseData.bv2);
      a = 3;
      break;
    case 4:
      circularPalette = 1;
      triangleFadePalette(_baseData.rh, _baseData.gs, _baseData.bv);
      a = 4;
      break;
    case 5:
      circularPalette = 0;
      twoColorPalette(_baseData.rh, _baseData.gs);
      a = 2;
      break;
    case 6:
      circularPalette = 0;
      threeColorPalette(_baseData.rh, _baseData.gs, _baseData.bv);
      a = 3;
      break;
    case 7:
      circularPalette = 0;
      fourColorPalette(_baseData.rh, _baseData.gs, _baseData.bv, _baseData.rh2);
      a = 4;
      break;
    case 8:
      circularPalette = 1;
      blackIsolationPalette(_baseData.rh, _baseData.gs, _baseData.bv);
      a = 3;
      break; 
    case 9:
      circularPalette = 1;
      fillNewPalette(_baseData.rh, _baseData.gs, _baseData.bv);
      a = 8;
      break;
    case 10:
      circularPalette = 1;
      colorScroll = anchor;
      a = 7;
      break;
    case 11: //sunsetFade palette
      circularPalette = 1;
      triangleFadePalette(13, 165, 190);
      a = 4;
      break;
    case 12: //caribbean blues
      circularPalette = 1;
      colorScroll = cbPal;
      a = 8;
      break;
    case 13:
      circularPalette = 1;
      colorScroll = leaves;
      a = 8;
      break;
    case 14:
      circularPalette = 0;
      hueSatOnlyPalette(_baseData.rh, _baseData.rh2, _baseData.gs, _baseData.gs2, _baseData.bv, _baseData.bv2);
      a = 3;
      break;
    case 15:
      circularPalette = 1;
      hueSatOnlyPaletteCircular(_baseData.rh, _baseData.rh2, _baseData.gs, _baseData.gs2, _baseData.bv, _baseData.bv2);
      a = 4;
      break;
    case 16:
      circularPalette = 0;
      colorScroll = warmDesertSunset;
      a = 8;
      break;
    case 17:
      circularPalette = 0;
      colorScroll = coolDesertSunset;
      a = 2;
      break;
    case 18:
      circularPalette = 0;
      colorScroll = purpleRedPinkMulti;
      a = 8;
      break;
    case 19:
      circularPalette = 0;
      colorScroll = fullFive;
      a = 4;
      break;
    case 20: {
        //saturationStepdown
        //leaves approximation for any color
        CHSV hc = rgb2hsv_approximate(CRGB(_baseData.rh, _baseData.gs, _baseData.bv));
        uint8_t index = 255;
        for(int i = 0; i < 8; i++) {
          colorScroll[i] = CHSV(hc.hue, index, _baseData.rh2);
          index -= 50/8;
        }
        for(int i = 8; i < 16; i++) {
          colorScroll[i] = CHSV(hc.hue, index, _baseData.rh2);
          index -= 200/8;
        }
        circularPalette = 0;
        a = 8;
        break;
      }
    case 21:
      circularPalette = 1;
      //make rainbow palette
      for(int i = 0; i < 256; i++) {
        colorScroll[i] = CHSV(i, 255,255);
      }
      a = 8;
      break;
  }
  activeColor = ColorFromPalette(colorScroll, random8(), 255, LINEARBLEND);
  return a;
}
boolean verifyAddress() {
  if((addr == _baseDataTemp.zone || UNIV_NUMBER == _baseDataTemp.zone) && _baseDataTemp.rev <= SOFTWARE_REVISION 
          && (_baseDataTemp.formfactor == 0 || _baseDataTemp.formfactor == FFACTOR) && (_baseDataTemp.cclass == COLORCLASS 
          || _baseDataTemp.cclass == 0 || _baseDataTemp.cclass == 2)) {
    return true;
  }
  if(_baseDataTemp.cclass == 4) {
    for(int i = 1; i <= currBrightness; i++) {
      FastLED.setBrightness(currBrightness - i);
      ledsProject();
      delay(3);
    }
    powerStatus = 0;
    currBrightness = 0;
  }
  return false;
}
boolean checkNewData() {
  if(millis() - lastTimeChecked > checkInterval) {
    lastTimeChecked = millis();
    if(_satellite.hasData() != 0) {
      //_satellite.readData(&_baseData); // Note how '&' must be placed in front of the variable name.
      //Serial.println("packet recerived");
      _satellite.readData(&_baseDataTemp);
      if(verifyAddress()) {
        if(_baseDataTemp.qcomm == 65535) {
          fullFive = CRGBPalette16(CRGB(_baseDataTemp.rh ,_baseDataTemp.gs , _baseDataTemp.bv), 
          CRGB(_baseDataTemp.rh2 ,_baseDataTemp.gs2 , _baseDataTemp.bv2), 
          CRGB(_baseDataTemp.ext1 ,_baseDataTemp.ext2 , _baseDataTemp.ext3),
          CRGB(_baseDataTemp.ext4 ,_baseDataTemp.sw1 , _baseDataTemp.sw2));
        } else {
          _baseData = _baseDataTemp;
          paletteIndex = 0;
          //brightnessDataVerified = 1;
          firstRun = 1;
          return true;
        } 
      }
    }
  }
  return false;
}

int rcount = 0;
void iterateColorTheme(int x) {
  x = rangeNumberCircular(x, 0, 24);
  if(x != rcount) { //enforce ranging on rcount as well
    rcount = x;
  }
  _baseData.brightKey = 255;
  if(x < 16) {
    activeColor = CHSV(x * 16 , 255, 255);
    _baseData.qcomm = 11;
    _baseData.color = x * 16;
  } else {
    switch(x) {
      case 16: //sunset static color stripes (orange at top, purple, then blue at floor)
        activeColor = CHSV(195,255,255);
        _baseData.qcomm = 41;
        _baseData.ext3 = 0;
        _baseData.ext4 = 0;
        _baseData.extra = 6;
        _baseData.rh = 13;
        _baseData.gs = 165;
        _baseData.bv = 190;
        break;
      case 17: //neon sunset static stripes (blue at ceiling, then magenta, then teal)
        activeColor = CHSV(160,255,255);
        _baseData.qcomm = 41;
        _baseData.ext3 = 0;
        _baseData.ext4 = 0;
        _baseData.extra = 6;
        _baseData.rh = 160;
        _baseData.gs = 220;
        _baseData.bv = 120;
        break;
      case 18: //multi stripe solid fade (qcomm == 41), sunsetFade palette
        activeColor = CHSV(13,255,255);
        _baseData.qcomm = 41;
        _baseData.ext3 = 0;
        _baseData.ext4 = 1;
        _baseData.extra = 6;
        _baseData.rh = 13;
        _baseData.gs = 165;
        _baseData.bv = 190;
        _baseData.speed = 20;
        break;
      case 19: //multi stripe solid fade, ocean (cbPal) palette
        activeColor = CHSV(150, 255,255);
        _baseData.qcomm = 41;
        _baseData.ext3 = 0;
        _baseData.ext4 = 1;
        _baseData.extra = 12;
        _baseData.speed = 20;
        break;
      case 20:  //2700K
        activeColor = CRGB(255,78,8);
        _baseData.qcomm = 10;
        _baseData.extra = 0;
        _baseData.color = 270;
        break;
      case 21:  //salmon
        activeColor = CRGB(255,73,21);
        _baseData.qcomm = 10;
        _baseData.extra = 0;
        _baseData.color = 5;
        break;
      case 22:  //4000K
        activeColor = CRGB(255,106,44);
        _baseData.qcomm = 10;
        _baseData.extra = 0;
        _baseData.color = 400;
        break;
      case 23:  //6500K
        activeColor =  CRGB(239,164,173);
        _baseData.qcomm = 10;
        _baseData.extra = 0;
        _baseData.color = 650;
        break;
      
    }
  }
  firstRun = 1; //flag to trigger light engine update
}

void initRotary() {
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DAT, INPUT_PULLUP);
  pinMode(SWT, INPUT_PULLUP);
}
uint8_t lrmemR = 3;
int lrsumR = 0;
uint8_t lrmemG = 3;
int lrsumG = 0;
uint8_t lrmemB = 3;
int lrsumB = 0;
int rotary() {
   static int8_t TRANS[] = {0,-1,1,14,1,0,14,-1,-1,14,0,1,14,1,-1,0};
   int8_t lef, rig;

   lef = digitalRead(CLK);
   rig = digitalRead(DAT);

   lrmemR = ((lrmemR & 0x03) << 2) + 2*lef + rig;
   lrsumR = lrsumR + TRANS[lrmemR];
   /* encoder not in the neutral state */
   if(lrsumR % 2 != 0) return(0);
   /* encoder in the neutral state */
   if (lrsumR == 2) {
      lrsumR = 0;
      return(1);
   }
   if (lrsumR == -2) {
      lrsumR = 0;
      return(-1);
   }
   /* lrsum > 0 if the impossible transition */
   lrsumR=0;
   return(0);
}
boolean checkRotary() {
  rdelta = rotary();
  if(rdelta != 0) {
    lastKnAct = millis();
    rcount += rdelta;
    if(rockerState == 1 || rockerState == 0) {
      iterateColorTheme(rcount);
    }
    Serial.println(rcount);
    return true;
  } else {
    return false;
  }
}
void fadeUpBasic() {
  _baseData.qcomm = 10;
  _baseData.speed = 0;
  _baseData.extra = 5;
  _baseData.brightKey = 255;
  paletteIndex = 0;
  firstRun = 1;
}
//summons the shadow around a position in the LED strip
void summonPeak(uint16_t ledIndex) { // 0 <= x < NUM_LEDS
  int tailLength = 10;
  if(ledIndex > NUM_LEDS) {
    ledIndex = NUM_LEDS;
  }
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  leds[ledIndex] = activeColor;

  for(int i = ledIndex + 1; i < NUM_LEDS; i++) {
    if(abs(ledIndex - i) > tailLength) { break; }
    CRGB t = activeColor;
    t.r = t.r * ((float) (tailLength - abs(ledIndex - i))) / tailLength;
    t.g = t.g * ((float) (tailLength - abs(ledIndex - i))) / tailLength;
    t.b = t.b * ((float) (tailLength - abs(ledIndex - i))) / tailLength;
    leds[i] = t;
  }
  for(int i = ledIndex - 1; i > 0; i--) {
    if(abs(ledIndex - i) > tailLength) { break; }
    CRGB t = activeColor;
    t.r = t.r * ((float) (tailLength - abs(ledIndex - i))) / tailLength;
    t.g = t.g * ((float) (tailLength - abs(ledIndex - i))) / tailLength;
    t.b = t.b * ((float) (tailLength - abs(ledIndex - i))) / tailLength;
    leds[i] = t;
  }

  ledsProject();
}

//LIDAR
HardwareSerial mySerial(2);
int lidar[9];
uint16_t livePeakPosition;
uint16_t targetPeakPosition;
uint16_t lastLidarDist;
uint16_t maxLidarDistance = 695;
unsigned long lastLidarPing = 0;
uint16_t strength;
uint16_t lidarTemp = 0;
unsigned long noMovementSince = 0;
unsigned long lastPositionUpdate = 0;
unsigned long followingShadowLedsUpdate = 0;
unsigned long movementDetectionTimer = 0;
bool noMovement = 0;
void initLidar() {
    mySerial.begin(115200, SERIAL_8N1, 2, 1);
}
void pollLidar() {
  // Look for the 0x59 0x59 frame header
  while (mySerial.available() >= 9) { // Peek ahead without removing bytes
    if (mySerial.peek() == 0x59) {
      mySerial.read();  // consume first 0x59
      if (mySerial.peek() == 0x59) {
        mySerial.read();  // consume second 0x59
        // Store header
        lidar[0] = 0x59;
        lidar[1] = 0x59;

        // Read the remaining 7 bytes
        for (int i = 2; i < 9; i++) {
          while (!mySerial.available()); // wait for byte
          lidar[i] = mySerial.read();
        }
        return; // got a valid frame
      }
    } else {
      // Drop misaligned byte and keep searching
      mySerial.read();
    }
  }
}
uint16_t getLidarDistance() {
    strength = (static_cast<uint16_t>(lidar[5]) << 8) | lidar[4];
    lidarTemp = (static_cast<uint16_t>(lidar[7]) << 8) | lidar[6];
    return (static_cast<uint16_t>(lidar[3]) << 8) | lidar[2]; //current distance measurement
}
void printLidarData() {
    for(int i = 0; i < 9; i++) {
        Serial.print(lidar[i]);
        Serial.print(" ");
    }
    Serial.println();
}
int liveBrightness = 0;
boolean updateLidar() {
  boolean movementDetected = false;
  if(millis() - lastLidarPing > 5) { //ping the lidar sensor every 5 ms
    lastLidarPing = millis();
    pollLidar();
  }
  if(millis() - lastPositionUpdate > 250) { //update the target position of the LEDs every 250ms
    lastPositionUpdate = millis();
    targetPeakPosition = (int)( ((float) getLidarDistance()) / ((float) maxLidarDistance) * NUM_LEDS);
    if(livePeakPosition != targetPeakPosition) {
      movementDetected = true;
    }
  }
  if(millis() - followingShadowLedsUpdate > 1) { //have a function that converges the LED strip to the target position every 5ms
    followingShadowLedsUpdate = millis();
    if(engineActive == 0) {
      if(livePeakPosition != targetPeakPosition) {
        if(liveBrightness < BRIGHTNESS) { //bring up brightness when transitioning into peak mode
          liveBrightness += 1;
          FastLED.setBrightness(liveBrightness);
        }
        if(livePeakPosition < targetPeakPosition) {
          livePeakPosition++;
          summonPeak(livePeakPosition);
        } else {
          livePeakPosition--;
          summonPeak(livePeakPosition);
        }
      }
    } else {
      if(abs(targetPeakPosition - NUM_LEDS) > 5) {
        livePeakPosition = 0; //reset to zero so peak starts at box and comes to meet you
        liveBrightness = 0;
        engineActive = 0; //SWAP POINT, going to peak mode
      }
    }
  }
  if(millis() - movementDetectionTimer > 2700 && engineActive == 0) {  //check if the livePeakPosition is sitting at the end of the hall every 3000ms
    movementDetectionTimer = millis();
    if(abs(livePeakPosition - NUM_LEDS) < 5) {
      engineActive = 1;   //SWAP POINT, going back to idle mode
      for(int i = currBrightness; i >= 0; i -= 5) {
        FastLED.setBrightness(i);
        ledsProject();
        delay(1);
      }
      FastLED.setBrightness(0);
      powerStatus = 0;
      currBrightness = 0;
      
      iterateColorTheme(rcount);
    }
  }
  return movementDetected;
}
//LIDAR

boolean delayTime(uint16_t delayamnt) {
  difference = millis();
  while( millis() - difference < delayamnt) {
    if((checkNewData() && verifyAddress()) || checkRotary()) { // || updateLidar()) {
      return true;  //exit the delay early
    }
  }
  return false; //the delay finished completely
}
void off(uint16_t dspeed) {
  for(int i = 1; i <= currBrightness; i++) {
    FastLED.setBrightness(currBrightness - i);
    ledsProject();
    delayTime(dspeed+1);
  }
  powerStatus = 0;
  currBrightness = 0;
}
void on(uint8_t goal, uint16_t dspeed) {
  for(int i = goal; i >= 0; i--) {
    FastLED.setBrightness(goal - i);
    ledsProject();
    delayTime(dspeed+1);
  }
  powerStatus = 1;
  currBrightness = BRIGHTNESS;
}

void setup() {
  //const int ledPins[14] = {4,5,6,7,15,16,17,18,8,9,10,11,12,13};
  FastLED.addLeds<WS2811, 4, BRG>(ledsMatrix[0], NUM_LEDS);
  FastLED.addLeds<WS2811, 5, BRG>(ledsMatrix[1], NUM_LEDS);
  FastLED.addLeds<WS2811, 6, BRG>(ledsMatrix[2], NUM_LEDS);
  FastLED.addLeds<WS2811, 7, BRG>(ledsMatrix[3], NUM_LEDS);
  FastLED.addLeds<WS2811, 15, BRG>(ledsMatrix[4], NUM_LEDS);
  FastLED.addLeds<WS2811, 16, BRG>(ledsMatrix[5], NUM_LEDS);
  FastLED.addLeds<WS2811, 17, BRG>(ledsMatrix[6], NUM_LEDS);
  FastLED.addLeds<WS2811, 18, BRG>(ledsMatrix[7], NUM_LEDS);
  FastLED.addLeds<WS2811, 8, BRG>(ledsMatrix[8], NUM_LEDS);
  FastLED.addLeds<WS2811, 9, BRG>(ledsMatrix[9], NUM_LEDS);
  //these four strips below are unused, more strips can be connected to the esp with the unused ports on the esp
  FastLED.addLeds<WS2811, 10, BRG>(ledsMatrix[10], NUM_LEDS);
  FastLED.addLeds<WS2811, 11, BRG>(ledsMatrix[11], NUM_LEDS);
  FastLED.addLeds<WS2811, 12, BRG>(ledsMatrix[12], NUM_LEDS);
  FastLED.addLeds<WS2811, 13, BRG>(ledsMatrix[13], NUM_LEDS);
  //onbard status LED
  FastLED.addLeds<WS2811, 38, GRB>(oboard, 1);
  delay(100);

  Serial.begin(9600);

  initRadio();
  initRotary();
  initLidar();
  initRocker();

  fullFive = leaves;
  if(BOOT_PATTERN == 1) {  //0 for chasing green, 1 for slow pulse
    leds[0] = CHSV(88,188,255);
    FastLED.setBrightness(0);
    ledsProject();
    for(int i = 0; i <= BRIGHTNESS; i++) {
      FastLED.setBrightness(i);
      ledsProject();
      delay(2);
    }
    for(int i = BRIGHTNESS; i >= 0; i--) {
      FastLED.setBrightness(i);
      ledsProject();
      delay(2);
    }
    //4000K
    fill_solid(leds, NUM_LEDS, CRGB(255,106,44));
    //
    FastLED.setBrightness(0);
    ledsProject();
  } else if(BOOT_PATTERN == 0) { //chasing
    FastLED.setBrightness(BRIGHTNESS);
    for(int i = 0; i < NUM_LEDS; i++){
      leds[i] = CHSV(88,188,255);
      ledsProject();
      fadeToBlackBy(leds, NUM_LEDS, 35);
      delay(25);
    }
    //4000K
    fill_solid(leds, NUM_LEDS, CRGB(255,106,44));
    //
    FastLED.setBrightness(0);
    ledsProject();

    fadeUpBasic();
  } else if(BOOT_PATTERN == 2) { //init to default white
    fadeUpBasic();
  }
  
  //ZONE NUMBER
  addr = 1;
  srand(seed);
  for(int i = 0; i < NUM_LEDS; i++) { //make it so that pallettes arent solid colored
    colorIndex[i] = random8();
  }
  rockerState = -1;
  prevRockerState = -1;
}

void loop() {
  checkRotary();
  pollRocker();
  if(rockerState != prevRockerState) {
    if(prevRockerState == 1 && rockerState == 0) {
      FastLED.setBrightness(BRIGHTNESS); //once following is exited, set brightness to max
    }
    prevRockerState = rockerState;
    if(rockerState == 1) {  //switch is up
      //on, lidar following
      engineActive = 1;
      oboard[0] = CHSV(13,255,255); //orange
      iterateColorTheme(rcount);
    } else if(rockerState == 0) { //switch is in middle
      //on, no lidar following
      oboard[0] = CRGB::Teal;
      engineActive = 1;
      iterateColorTheme(rcount);
    } else if(rockerState == -1) {  //swith is down
      //off
      off(3);
      engineActive = 0;
    }
    oboard[0] = CRGB((float) oboard[0].r * 0.1, (float) oboard[0].g * 0.1, (float) oboard[0].b * 0.1);  //dimming it a lot bit
    FastLED.show();
  }

  if(rockerState == 1) {
    updateLidar();
  }
  
  if(engineActive) {
    if(!protectedRedo) {
      checkNewData();
    }
    protectedRedo = 0;
    if(firstRun) {
      paletteIndex = 0;
    }
    if(_baseData.qcomm == 0) { //turn off
      if(powerStatus == 1) {
        if(_baseData.extra == 0) { //fade off, anything else will jolt off
          off(_baseData.speed);
        }
        FastLED.setBrightness(0);
        ledsProject();
      }
      currBrightness = 0;
    } else if(_baseData.qcomm == 1) { //turn on
      if(powerStatus == 0) {
        if(_baseData.extra == 0) { //fade on, anything else will jolt on
          on(BRIGHTNESS, _baseData.speed);
        }
        FastLED.setBrightness(BRIGHTNESS);
        ledsProject();
      }
    } else if(_baseData.qcomm == 2) { //brightness ramp up / down
      if(firstRun) { //this loop ensures we only let the for loops run once
        if(_baseData.extra == 0) { // ramp down to a floor in .color
          if(_baseData.color < currBrightness) {
            for(int i = currBrightness; i >= _baseData.color; i--) { //ramp down to a floor in .color
              FastLED.setBrightness(i);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                break;
              }
            }
            currBrightness = _baseData.color;
          }
          FastLED.setBrightness(_baseData.color);
          ledsProject();
        } else if(_baseData.extra == 1) { //ramp up to a ceiling in .color
          if(_baseData.color > BRIGHTNESS) {
            _baseData.color = BRIGHTNESS;
          }
          if(currBrightness < _baseData.color) {
            for(int i = currBrightness; i <= _baseData.color; i++) {
              FastLED.setBrightness(i);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                break;
              }
            }
            currBrightness = _baseData.color;
          }
          FastLED.setBrightness(_baseData.color);
          ledsProject();
        } else if(_baseData.extra == 2) { //move down by in .color
          currBrightness -= _baseData.color;
          if(currBrightness < 0) {
            currBrightness = 0;
          }
          for(int i = (currBrightness + _baseData.color); i >= currBrightness; i--) {
            FastLED.setBrightness(i);
            ledsProject();
            if(delayTime(_baseData.speed)) {
            break;
            }
          }
        } else if(_baseData.extra == 3) { //move up by in .color
          currBrightness +=_baseData.color;
          if(currBrightness > BRIGHTNESS) {
            currBrightness = BRIGHTNESS;
          }
          for(int i = (currBrightness - _baseData.color); i <= currBrightness; i++) {
            FastLED.setBrightness(i);
            ledsProject();
            if(delayTime(_baseData.speed)) {
            break;
            }
          }
        } else if(_baseData.extra == 4) { //absolute brightness tranversal, will correct brightness to whatever is in .color, up or down, won't violate BRIGHTNESS
          //ideally more even lighting
          if(_baseData.color > BRIGHTNESS) {
            _baseData.color = BRIGHTNESS;
          }
          if(_baseData.color > currBrightness) {
            //ramp up
            for(int i = currBrightness; i <= _baseData.color; i++) {
              FastLED.setBrightness(i);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                break;
              }
            }
          } else {
            //ramp down
            for(int i = currBrightness; i >= _baseData.color; i--) {
              FastLED.setBrightness(i);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                break;
              }
            }
          }
          currBrightness = _baseData.color;
        } else if(_baseData.extra == 5) { 
          //percent brightness transversal, will calculate a percent brightness based on a 100% or 256% scale
          if(_baseData.color > 255) {
            _baseData.color = 255;
          }
          //all set to the same percent brightness out of 255
          int bKey = (int) ((BRIGHTNESS / (float) 255) * _baseData.color); //256% scale
          //_baseData.color = (int) ((BRIGHTNESS / (float) 100) * _baseData.color); //100% scale
          if(bKey > currBrightness) {
            //ramp up
            for(int i = currBrightness; i <= bKey; i++) {
              FastLED.setBrightness(i);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                break;
              }
            }
            currBrightness = bKey;
            if(bKey > 1) {
              powerStatus = 1;
            }
          } else {
            //ramp down
            for(int i = currBrightness; i >= bKey; i--) {
              FastLED.setBrightness(i);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                break;
              }
            }
            currBrightness = bKey;
            if(bKey <= 1) {
              powerStatus = 0;
            }
          }
        }
        /*
        * fix here
        */
        if(currBrightness > 0) {
          powerStatus = 1;
        } else if(currBrightness == 0 && powerStatus) {
          powerStatus = 0;
          protectedRedo = 1;
        }
        /*
        * fix end
        */
        firstRun = 0;
      } //this loop ensures we only let the for loops run once
    } else if(_baseData.qcomm == 3) { //granular brightness fade to/from
      if(firstRun) {
        if(_baseData.rh > BRIGHTNESS) {
          _baseData.rh = BRIGHTNESS;
        } else if(_baseData.rh < 0) {
          _baseData.rh = 0;
        }
        if(_baseData.gs > BRIGHTNESS) {
          _baseData.gs = BRIGHTNESS;
        } else if(_baseData.gs < 0) {
          _baseData.gs = 0;
        }
        if(_baseData.rh < _baseData.gs) {
          //rh is floor, ramp up
          for(int i = _baseData.rh; i <= _baseData.gs; i++) {
            FastLED.setBrightness(i);
            ledsProject();
            if(delayTime(_baseData.speed)) {
              break;
            }
          }
          FastLED.setBrightness(_baseData.gs);
          currBrightness = _baseData.gs;
        } else {
          //rh is ceiling, ramp down
          for(int i = _baseData.rh; i >= _baseData.gs; i--) {
            FastLED.setBrightness(i);
            ledsProject();
            if(delayTime(_baseData.speed)) {
              break;
            }
          }
          FastLED.setBrightness(_baseData.gs);
          currBrightness = _baseData.gs;
        }
        
        firstRun = 0;
      }
    } else if(_baseData.qcomm == 5) { //absolute gradient index pick
      if(firstRun) {
        firstRun = 0;
        paletteSelect(_baseData.extra);
        //fill_palette(leds, NUM_LEDS, start, increment, _palette, _brightness, _blendType);
        if(_baseData.speed == 0) {
          fill_palette(leds, NUM_LEDS, _baseData.color, 256.0 / NUM_LEDS, colorScroll, 255, LINEARBLEND);
        } else {
          fill_palette(leds, NUM_LEDS, _baseData.color, _baseData.speed, colorScroll, 255, LINEARBLEND);
        }
        ledsProject();
      }
    } else if(_baseData.qcomm == 6) { //absolute palette position pick
      if(firstRun) {
        firstRun = 0;
        paletteSelect(_baseData.extra);
        fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, _baseData.color, 255, LINEARBLEND));
        ledsProject();
      }
    } else if(_baseData.qcomm == 7) { //absolute color fade
      if(firstRun) {
        firstRun = 0;
        CRGB x0 = leds[0];
        for(int i = 0; i < 256; i += (_baseData.sw1 + 1)) {
          if(_baseData.extra == 0) { //absolute fade CRGB
            fill_solid(leds, NUM_LEDS, blend(CRGB(_baseData.rh, _baseData.gs, _baseData.bv), CRGB(_baseData.rh2, _baseData.gs2, _baseData.bv2),i));
          } else if(_baseData.extra == 1) { //absolute fade CHSV
            fill_solid(leds, NUM_LEDS, blend(CHSV(_baseData.rh, _baseData.gs, _baseData.bv), CHSV(_baseData.rh2, _baseData.gs2, _baseData.bv2),i));
          } else if(_baseData.extra == 2) { //fade converge from current color CRGB
            fill_solid(leds, NUM_LEDS, blend(x0, CRGB(_baseData.rh, _baseData.gs, _baseData.bv),i));
          } else if(_baseData.extra == 3) { //fade converge from current color CHSV
            fill_solid(leds, NUM_LEDS, blend(x0, CHSV(_baseData.rh, _baseData.gs, _baseData.bv),i));
          }
          ledsProject();
          if(delayTime(_baseData.speed)) {
            break;
          }
        }
      }
    } else if(_baseData.qcomm == 8) { //save default color
      if(firstRun) {
        if(_baseData.ext1 == 0) {
          defaultColor = leds[0];
        } else if(_baseData.ext1 == 1) {
          defaultColor.r = _baseData.rh;
          defaultColor.g = _baseData.gs;
          defaultColor.b = _baseData.bv;
        } else if(_baseData.ext1 == 2) {
          //some eeprom, non-volatile storage of a color(s) to allow for patterns like this:
          //https://www.amazon.com/Yuewilai-Magnetic-Dimmable-Rechargeable-Operated/dp/B0BDCSV7VR?th=1
          //desc: single room, multi color pattern where a specific fixture will display the same color each time, no random selection
          //be careful to not call this too many times
        } else if(_baseData.ext1 == 3) {
          //save a default palette index position
          //only save the index, as palette will be send with the request command below
          //use incoming data to just call qcomm == 5
        }
        firstRun = 0;
      }
    } else if(_baseData.qcomm == 9) { //load default color
      if(firstRun) {
        if(_baseData.ext1 == 2) {
          //some eeprom, non-volatile magic, see above
        }  else if(_baseData.ext1 == 3) {
          // uint8_t eepromIndex = readEEPROM(byte addr);
          // paletteSelect(_baseData.extra);
          // fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, eepromIndex, 255, LINEARBLEND));
          // ledsProject();
        } else {
          fill_solid(leds, NUM_LEDS, defaultColor);
          ledsProject();
          
        }
        firstRun = 0;
      }
    } else if(_baseData.qcomm == 10) { //whites
      if(firstRun) {
        firstRun = 0;
        if(_baseData.extra == 0) {
          switch(_baseData.color) { //custom whites, incandescent, pastels maybe
            case 0: //pure white
              fill_solid(leds, NUM_LEDS, CRGB(255,255,255));
              break;
            case 1: //greenish white
              fill_solid(leds, NUM_LEDS, CRGB(250,203,72));
              break;
            case 2: //good work light
              fill_solid(leds, NUM_LEDS, CRGB(255,139,78));
              break;
            case 3: //purplish white
              fill_solid(leds, NUM_LEDS, CHSV(176,135,255));
              break;
            case 4: //golden
              fill_solid(leds, NUM_LEDS, CRGB(255,120,25));
              break;
            case 5: //salmon incandescent
              fill_solid(leds, NUM_LEDS, CRGB(255,73,21));
              break;
            case 6: //moonlight
              fill_solid(leds, NUM_LEDS, CHSV(176,135,255)); //same as purplish white
              break;
            //color temps here
            case 200: //2000K
              fill_solid(leds, NUM_LEDS, CRGB(255,57,4));
              break;
            case 270: //2700K
              fill_solid(leds, NUM_LEDS, CRGB(255,78,8));
              break;
            case 300: //3000K
              fill_solid(leds, NUM_LEDS, CRGB(255,87,15));
              break;
            case 350: //3500K
              fill_solid(leds, NUM_LEDS, CRGB(255,100,27));
              break;
            case 400: //4000K
              fill_solid(leds, NUM_LEDS, CRGB(255,106,44));
              break;
            case 450: //4500K
              fill_solid(leds, NUM_LEDS, CRGB(255,132,56));
              break;
            case 500: //5000K
              fill_solid(leds, NUM_LEDS, CRGB(255,157,90));
              break;
            case 550: //5500K
              fill_solid(leds, NUM_LEDS, CRGB(255,138,98));
              break;
            case 600: //6000K
              fill_solid(leds, NUM_LEDS, CRGB(255,166,149));
              break;
            case 650: //6500K
              fill_solid(leds, NUM_LEDS, CRGB(239,164,173));
              break;
            case 700: //7000K
              fill_solid(leds, NUM_LEDS, CRGB(220,181,192));
              break;
            case 750: //7500K
              fill_solid(leds, NUM_LEDS, CRGB(219,198,205));
              break;
            case 800: //8000K
              fill_solid(leds, NUM_LEDS, CRGB(202,212,239));
              break;
            case 850: //8500K
              fill_solid(leds, NUM_LEDS, CRGB(179,155,255));
              break;
            case 900: //9000K
              fill_solid(leds, NUM_LEDS, CRGB(111,148,255));
              break;
            case 950: //9500K
              fill_solid(leds, NUM_LEDS, CRGB(94,125,255));
              break;
            case 1000: //10000K
              fill_solid(leds, NUM_LEDS, CRGB(37,82,255));
              break;
          }
        } else if(_baseData.extra == 1) { //alternative color temp selection
          fill_solid(leds, NUM_LEDS, ColorFromPalette(cTemp, _baseData.color, 255, LINEARBLEND));
        } else if(_baseData.extra == 2) { //gray mosaic, varying light levels
          fill_solid(leds, NUM_LEDS, CHSV(0,0,random(0,_baseData.randfactor) + _baseData.color));
        } else if(_baseData.extra == 3) { //gray mosaic, selected colorTemp
          fill_solid(leds, NUM_LEDS, ColorFromPalette(cTemp, _baseData.color, random(0,_baseData.randfactor) + _baseData.rh, LINEARBLEND));
        } else if(_baseData.extra == 4) { //gray level
          fill_solid(leds, NUM_LEDS, CHSV(0,0,_baseData.color));
        } else if(_baseData.extra == 5) {
          //default white
          _baseData.color = 400;
          _baseData.color = 400;
          _baseData.extra = 0;
          
          //be careful, may create infinte loop
          firstRun = 1;
          protectedRedo = 1;
          //2700K 4000K 6000K 8500K
        }
        activeColor = leds[0];
        ledsProject();
      }
    } else if(_baseData.qcomm == 11) { //SOLID COLOR, defined or random or variation around a defined color
      //RGB SOLID COLOR
      if(firstRun) {
        if(_baseData.color < 256) { //single hue color
          fill_solid(leds, NUM_LEDS, CHSV(_baseData.color, 255, 255));
        } else if(_baseData.extra == 0) { //we specified a CRGB color
          fill_solid(leds, NUM_LEDS, CRGB(_baseData.rh, _baseData.gs, _baseData.bv));
        } else if(_baseData.extra == 1) { //we specified a CHSV color
          fill_solid(leds, NUM_LEDS, CHSV(_baseData.rh, _baseData.gs, _baseData.bv));
        } else if(_baseData.extra == 2) { //random solid color, static
          fill_solid(leds, NUM_LEDS, CHSV(random8(), 255, 255));
        } else if(_baseData.extra == 3) { //vary around supplied hue
          fill_solid(leds, NUM_LEDS, CHSV(_baseData.rh + (random(0, _baseData.randfactor * 2) - _baseData.randfactor) , _baseData.gs, _baseData.bv));
        } else if(_baseData.extra == 4) { //vary around supplied sat
          fill_solid(leds, NUM_LEDS, CHSV(_baseData.rh, _baseData.gs + (random(0, _baseData.randfactor * 2) - _baseData.randfactor), _baseData.bv));
        } else if(_baseData.extra == 5) { //vary around supplied val
          fill_solid(leds, NUM_LEDS, CHSV(_baseData.rh, _baseData.gs, _baseData.bv + (random(0, _baseData.randfactor * 2) - _baseData.randfactor)));
        } else if(_baseData.extra == 6) { //vary around supplied red
          fill_solid(leds, NUM_LEDS, CRGB(_baseData.rh + (random(0, _baseData.randfactor * 2) - _baseData.randfactor), _baseData.gs, _baseData.bv));
        } else if(_baseData.extra == 7) { //vary around supplied green
          fill_solid(leds, NUM_LEDS, CRGB(_baseData.rh, _baseData.gs + (random(0, _baseData.randfactor * 2) - _baseData.randfactor), _baseData.bv));
        } else if(_baseData.extra == 8) { //vary around supplied blue
          fill_solid(leds, NUM_LEDS, CRGB(_baseData.rh, _baseData.gs, _baseData.bv + (random(0, _baseData.randfactor * 2) - _baseData.randfactor)));
        }
        activeColor = leds[0];
        ledsProject();
        firstRun = 0;
      }
    } else if(_baseData.qcomm == 12) { //RAINBOW FADE   DYNAMIC
      //could eventually incude location based rainbow for smooth rainbow waves across a space (active grid)
  //      color: 0-255 are for solid color fade, 256 for each pod having random color and fading through
  //      speed: speed of fading, could eventually have random speed fading
  //      extra: color step (smooth rainbow? or skip through?)
  //      zone: used later with addressed rainbow for location awareness
  //    NOTES: extra must be 1<, speed should be 1<
      if(FFACTOR == 2) { //stripwise
        _baseData.qcomm = 14;
      } else { //podwise, default
        _baseData.qcomm = 13;
      }
      protectedRedo = 1;
    } else if(_baseData.qcomm == 13) { //RAINBOW FADE   PODWISE
      if(firstRun) {
        if(_baseData.color > 255) {
          //the random starting color needs to be set once only
          //unsync rainbow between pods
          _baseData.color = random8();
        } 
        ghue = _baseData.color;
        firstRun = 0;
      }
      fill_solid(leds, NUM_LEDS, CHSV(ghue,255,255));
      delayTime(_baseData.speed);
      ghue+= _baseData.extra;
      ledsProject();
    } else if(_baseData.qcomm == 14) { //RAINBOW FADE   STRIPWISE
      //incldue option to adjust step count
      //include option to have rainbow fill based on the NUM_LEDS, like you can specify how many full or fractional rainbows you want over the course of the strip
      if(firstRun) {
          if(_baseData.color > 255) {
            //the random starting color needs to be set once only
            //unsync rainbow between pods
            _baseData.color = random8();
          } 
          ghue = _baseData.color;
          firstRun = 0;
        }
      if(_baseData.rh == 0) {
        fill_rainbow(leds, NUM_LEDS, ghue, _baseData.gs);
        delayTime(_baseData.speed);
        ghue+= _baseData.extra;
        ledsProject();
      } else if(_baseData.rh == 1) { //basic fill_rainbow, default is one rainbow per strip
        fill_rainbow(leds, NUM_LEDS, ghue, 256/NUM_LEDS);
        delayTime(_baseData.speed);
        ghue+= _baseData.extra;
        ledsProject();
      } else if(_baseData.rh == 2) { //# of desired rainbows specified in .gh
        fill_rainbow(leds, NUM_LEDS, ghue, 256 / NUM_LEDS * _baseData.gs);
        delay(_baseData.speed);
        ghue+= _baseData.extra;
        ledsProject();
      } else if(_baseData.rh == 3) { //fill with one color, then swipe across to the next color
        for(int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CHSV(ghue, _baseData.rh2, _baseData.gs2);
          ledsProject();
          delayTime(_baseData.speed);
          fadeToBlackBy(leds,NUM_LEDS,_baseData.bv);
        }
        ghue += _baseData.extra;
        delayTime(_baseData.bv + random(0, _baseData.randfactor));
        //_.extra is the step between comets
        //_.gs is the time between comets
      }
    } else if(_baseData.qcomm == 15) { //RGB PALETTE TWINKLE, like [leaves] DYNAMIC
      /*
      * THIS IS THE LEAVES PATTERN
      */
        //MULTI COLOR RGB PALETTE FILL, cool colors, warm colors, like water ceiling twinkle, RGB (NO WHITES)
        //color is pallette type 0-10 are my picks/favs, 11 on are generated, red w/ rand vairation??

        //monochromatic/whole string twinkle
        //moves slowly between a region around a number and brightness defined in color (hr,gs,bv)
        //speed is speed
        //extra defines if you want to fade around the color (HUE) or around a brightness level with a specific color (VALUE)
        //given a color/hue, build a fairy lights pattern around it, like with [leaves]
        if(FFACTOR == 2) {
          //stripwise
          _baseData.qcomm = 17;
        } else {
          //podwise
          _baseData.qcomm = 16;
        }
        protectedRedo = 1;
      } else if(_baseData.qcomm == 16) { //RGB PALETTE TWINKLE  PODWISE
        if(firstRun) {  
          //VV now #9 in palSrelect
          //fillNewPalette( _baseData.color, _baseData.extra, _baseData.rh ); //(hue, magnitude of variance, variance key(variate hue or sat) )
          paletteSelect(_baseData.extra);
          if(_baseData.ext3 == 0) {
            ghue = random8();
          } else {
            ghue = _baseData.ext3;
          }
          firstRun = 0;
        }
        fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, ghue, 255, LINEARBLEND));
        ledsProject();
        //makes the palette move
        delayTime(_baseData.speed);
        for(int i = 0; i < NUM_LEDS; i++) {
          if(_baseData.sw1 == 0) {
            ghue++;
          } else {
            ghue += _baseData.sw1 + random(0, _baseData.randfactor);
          }
        }
      } else if(_baseData.qcomm == 17) { //RGB PALETTE TWINKLE  STRIPWISE
        if(firstRun) {
          //VV now 9 in palSrelect
          //fillNewPalette( _baseData.color, _baseData.extra, _baseData.rh ); //(hue, magnitude of variance, variance key(variate hue or sat) )
          paletteSelect(_baseData.extra);
          firstRun = 0;
        }
        for(int i = 0; i < NUM_LEDS; i++) {
          leds[i] = ColorFromPalette(colorScroll, colorIndex[i], 255, LINEARBLEND);
        }
        ledsProject();
        //makes the palette move
        delayTime(_baseData.speed);
        for(int i = 0; i < NUM_LEDS; i++) {
          if(_baseData.sw2 == 0) {
            colorIndex[i]++;
          } else {
            colorIndex[i] += _baseData.sw2 + random(0, _baseData.randfactor);
          }
        }
      } else if(_baseData.qcomm == 18) { //WHITE PALETTE TWINKLE, fairy lights DYNAMIC
        //FAIRY LIGHTS TWINKLE
        //TWINKLE/sparkle WHITES, NO COLOR along string, not great for monochromatic pods
        //color is color pallette, 0 for random color, 1 for red, 2...
        //delay is speed of new light
        //extra is the fadeToBlack value
        if(FFACTOR == 2) { //stripwise
          _baseData.qcomm = 20;
        } else { //podwise
          _baseData.qcomm = 19;
        }
        protectedRedo = 1;
      } else if(_baseData.qcomm == 19) { //WHITE PALETTE TWINKLE, fairy lights PODWISE
        // _.rh          floor position in palette
        // _.gs          delta from floor
        // _.bv          base time delay for timeUp
        // _.extra       base time delay for timeDown
        // _.randfactor  random(0, rf) added to each time delay
        if(firstRun) {
          if(_baseData.color >= 256) {
            if(_baseData.color == 256) {
              //basic regular fairy lights
              //ERROR add green fireflies one
              colorScroll = CRGBPalette32(CRGB(0,0,0), CRGB(255,120,25), CRGB(255,255,255));
            } else if(_baseData.color == 259) {
              blackIsolationPalette(_baseData.rh2, _baseData.gs2, _baseData.bv2);
            } else if(_baseData.color == 257) {
              colorScroll = leaves;
            } else if(_baseData.color == 258) {
              colorScroll = anchor;
            }
          } else {
            //given hue
            colorScroll = CRGBPalette32(CHSV(0,0,0), CHSV(_baseData.color,255,255), CHSV(0,0,255));
          }
          firstRun = 0;
        }
        //rampUp
        for(int i = _baseData.rh; i <= (_baseData.rh + _baseData.gs); i++ ) {
          fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, i, 255, LINEARBLEND));
          ledsProject();
          if(delayTime(_baseData.speed)) {
            break;
          }
        }
        //timeUp
        delayTime(_baseData.bv + random(0,_baseData.randfactor));
        //rampDown
        for(int i = (_baseData.rh + _baseData.gs); i >= _baseData.rh; i-- ) {
          fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, i, 255, LINEARBLEND));
          ledsProject();
          if(delayTime(_baseData.speed)) {
            break;
          }
        }
        //timeDown
        delayTime(_baseData.extra + random(0,_baseData.randfactor));
      } else if(_baseData.qcomm == 20) { //WHITE PALETTE TWINKLE, fairy lights STRIPWISE
        if(firstRun) {
          if(_baseData.color >= 300) {
            paletteSelect(_baseData.color - 300);
          } else if(_baseData.color == 256) {
            //basic regular fairy lights
            colorScroll = CRGBPalette32(CRGB(0,0,0), CRGB(255,120,25));
          } else if(_baseData.color == 257) {
            //color input is hsv
            blackIsolationPalette(_baseData.rh2, _baseData.gs2, _baseData.bv2);
          } else {
            //some future proofing in case others are added
            colorScroll = CRGBPalette32(CRGB(0,0,0), CRGB(255,120,25));
          }
          firstRun = 0;
        }
        for(int i = 0; i < NUM_LEDS; i++) {
          leds[i] = ColorFromPalette(colorScroll, colorIndex[i], 255, LINEARBLEND);
        }
        ledsProject();
        for(int i = 0; i < NUM_LEDS; i++) {
          colorIndex[i]++;
        }
        delayTime(_baseData.speed);
  /////////////////////////////////////////////////////above all good////////////////////////////////////////////////////////////
      } else if(_baseData.qcomm == 21) { //SOLID COLOR PALETTE FADE, DYNAMIC
      //SHAPE FADE
        /*
        * implementation notes
        *  - if you get weird palette skipping, use non-circular palette
        */
        if(FFACTOR == 2) { //stripwise
          _baseData.qcomm = 23;
        } else { //podwise
          _baseData.qcomm = 22;
        }
        protectedRedo = 1;
    } else if(_baseData.qcomm == 22) { //SOLID COLOR PALETTE FADE, PODWISE, park at each color
      if(firstRun) {
        numAnchorPoints = paletteSelect(_baseData.extra);
        firstRun = 0;
        paletteIndex = 0;
      }
      bool skipSecondary = false;
      delta = random(0,2);
      uint8_t stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, delta);
      if(powerStatus) {
        if(delta == 1) { //fade up
          for(int i = paletteIndex; i <= stopBreak; i++) {
            fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, i, 255, LINEARBLEND));
            ledsProject();
            if(delayTime(_baseData.speed)) {
              skipSecondary = true;
              break;
            }
          }
        } else { //fade down
          for(int i = paletteIndex; i >= stopBreak; i--) {
            fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, i, 255, LINEARBLEND));
            ledsProject();
            if(delayTime(_baseData.speed)) {
              skipSecondary = true;
              break;
            }
          }
        }
        if(!skipSecondary) {
          delayTime(_baseData.color) + random(0,_baseData.randfactor);
        }
      }
      paletteIndex = stopBreak;
      fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, stopBreak, 255, LINEARBLEND));
    } else if(_baseData.qcomm == 23) { //SOLID COLOR PALETTE FADE, STRIPWISE
      //_.sw1 is palette incremnent
      if(firstRun) {
        numAnchorPoints = paletteSelect(_baseData.extra);
        firstRun = 0;
        paletteIndex = 0;
      }
      delta = random(0,2);
      bool skipSecondary = false;
      uint8_t stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, delta);
      if(powerStatus) {
        if(delta == 1) { //fade up
          if(_baseData.sw2 == 0) {
            //scroll the palette, up a color
            for(int i = paletteIndex; i <= stopBreak; i++) {
              fill_palette(leds, NUM_LEDS, i, _baseData.sw1, colorScroll, 255, LINEARBLEND);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                skipSecondary = true;
                break;
              }
            }
          } else if(_baseData.sw2 == 1) {
            //swipe forward across string, up a color
            for(int i = 0; i < NUM_LEDS; i++) {
              leds[i] = ColorFromPalette(colorScroll, stopBreak, 255, LINEARBLEND);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                skipSecondary = true;
                break;
              }
            }
          }
        } else { //fade down
          if(_baseData.sw2 == 0) {
            for(int i = paletteIndex; i >= stopBreak; i--) {
              //scroll the palette, down a color
              //fill_palette(leds, NUM_LEDS, start, increment, _palette, _brightness, _blendType);
              fill_palette(leds, NUM_LEDS, i, _baseData.sw1, colorScroll, 255, LINEARBLEND);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                skipSecondary = true;
                break;
              }
            }
          } else if(_baseData.sw2 == 1) {
            //swipe forward across string, down a color
            for(int i = 0; i < NUM_LEDS; i++) {
              leds[i] = ColorFromPalette(colorScroll, stopBreak, 255, LINEARBLEND);
              ledsProject();
              if(delayTime(_baseData.speed)) {
                skipSecondary = true;
                break;
              }
            }
          }
        }
        if(!skipSecondary) {
          delayTime(_baseData.color) + random(0,_baseData.randfactor);
        }
      }
      paletteIndex = stopBreak;
      if(_baseData.sw2 == 0) {
        fill_palette(leds, NUM_LEDS, stopBreak, _baseData.sw1, colorScroll, 255, LINEARBLEND);
      } else if(_baseData.sw2 == 1) {
        fill_solid(leds, NUM_LEDS, stopBreak);
      }
    } else if(_baseData.qcomm == 24) { //SOLID COLOR PALETTE JUMP, DYNAMIC
      //monochromatic palette color jump
      //given 2 to 4 colors, jump between them randomly or in sync
      //could be good for police car effect
      //add a fading option, 
      /*
      * pick a random hue, or pick a palette, so don't touch _.rh-_.bv2,
      *    if palette, randomly jump around, jump around to anchor points, directional or random
      * display
      * specify wait time and possible random wait time
      * specify fading
      * could include qualifier to not have a uniform fade, like comet tail in this vid https://www.youtube.com/watch?v=yM5dY7K2KHM
      */
      if(FFACTOR == 2) { //stripwise
          _baseData.qcomm = 26;
        } else { //podwise
          _baseData.qcomm = 25;
        }
        protectedRedo = 1;
    } else if(_baseData.qcomm == 25) { //SOLID COLOR PALETTE JUMP, PODWISE
      if(firstRun) {
        numAnchorPoints = paletteSelect(_baseData.extra);
        firstRun = 0;
        paletteIndex = 0;
      }
      int stopBreak;
      if(_baseData.ext4 == 0) {
        //pick color up or down
        stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, random(0,2));
      } else if(_baseData.ext4 == 1) {
        //always forward
        stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 0);
      } else { 
        //random everytime
        stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 2);
      }
      
      fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, stopBreak, 255, LINEARBLEND));
      ledsProject();
      paletteIndex = stopBreak;
      delayTime(_baseData.speed + random(0,_baseData.randfactor));
    } else if(_baseData.qcomm == 26) { //SOLID COLOR PALETTE JUMP, STRIPWISE
      //stopBreak is start
      //_.sw1 is increment
      if(firstRun) {
        numAnchorPoints = paletteSelect(_baseData.extra);
        firstRun = 0;
        paletteIndex = 0;
      }
      int stopBreak;
      if(_baseData.ext4 == 0) {
        //pick color up or down
        stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, random(0,2));
      } else if(_baseData.ext4 == 1) {
        //always forward
        stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 0);
      } else { 
        //random everytime
        stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 2);
      }
      if(_baseData.sw2 == 0) {
        //fill the string with a gradient
        fill_palette(leds, NUM_LEDS, stopBreak, _baseData.sw1, colorScroll, 255, LINEARBLEND);
        paletteIndex = stopBreak;
      } else {
        //fill each LED with a different color
        for(int i = 0; i < NUM_LEDS; i++) {
          stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 2);
          leds[i] = ColorFromPalette(colorScroll, stopBreak, 255, LINEARBLEND);
          paletteIndex = stopBreak;
        }
      }
      ledsProject();
      delayTime(_baseData.speed + random(0,_baseData.randfactor));
    } else if(_baseData.qcomm == 27) { //SOLID COLOR PALETTE PICK STATIC, DYNAMIC
      if(FFACTOR == 2) { //stripwise
          _baseData.qcomm = 29;
        } else { //podwise
          _baseData.qcomm = 28;
        }
        protectedRedo = 1;
    } else if(_baseData.qcomm == 28) { //SOLID COLOR PALETTE PICK STATIC, PODWISE
      if(firstRun) {
        int stopBreak = 0;
        numAnchorPoints = paletteSelect(_baseData.extra);
        if(_baseData.sw2 == 2) {
        //fill the strip with a random index in the gradient
        fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, random8(), 255, LINEARBLEND));
        } else {
        //fill the strip with a random anchor point
        stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 2);
        fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, stopBreak, 255, LINEARBLEND));
        }
        ledsProject();
        paletteIndex = stopBreak;
        firstRun = 0;
      }
    } else if(_baseData.qcomm == 29) { //SOLID COLOR PALETTE PICK STATIC, STRIPWISE
      //display the whole palette, but static, would look cool with a dipole vaporwave
      //_.color is startPos
      //_.ext1  is palette color increment
      if(firstRun) {
        numAnchorPoints = paletteSelect(_baseData.extra);
        int stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 2);
        if(_baseData.sw2 == 0) {
          //fill the string with a gradient
          if(_baseData.sw1 == 0) {
            //scale to fit length
            fill_palette(leds, NUM_LEDS, stopBreak, 255/NUM_LEDS, colorScroll, 255, LINEARBLEND);
          } else {
            //given offset
            fill_palette(leds, NUM_LEDS, stopBreak, _baseData.sw1, colorScroll, 255, LINEARBLEND);
          }
          paletteIndex = stopBreak;
        } else if(_baseData.sw2 == 1) {
          //fill each LED with a different color, anchor point
          for(int i = 0; i < NUM_LEDS; i++) {
            stopBreak = queryNextBreak(paletteIndex, numAnchorPoints, 2);
            leds[i] = ColorFromPalette(colorScroll, stopBreak, 255, LINEARBLEND);
            paletteIndex = stopBreak;
          }
        } else if(_baseData.sw2 == 2) {
          //fill each LED with a different color, any index in the gradient
          for(int i = 0; i < NUM_LEDS; i++) {
            leds[i] = ColorFromPalette(colorScroll, random8(), 255, LINEARBLEND);
          }
        }
        ledsProject();
        firstRun = 0;
      }
  //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\END OF THE SHAPE FADES//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    } else if(_baseData.qcomm == 30) { //random color JUMP, DYNAMIC   
      /*
      * essentially a simple way to implement a rainbow sparkle ceiling
      * have fancy options for fading and perhaps programtatic palette color selection
      */
      if(FFACTOR == 2) { //stripwise
          _baseData.qcomm = 32;
        } else { //podwise
          _baseData.qcomm = 31;
        }
        protectedRedo = 1;
    } else if(_baseData.qcomm == 31) { //random color JUMP, PODWISE
      if(firstRun) {
        difference = millis();
        difference2 = millis();
        fill_solid(leds, NUM_LEDS, CHSV(random8(), 255,255));
        firstRun = 0;
      }
      if(millis() - difference > _baseData.speed + random(0,_baseData.randfactor)) {
        difference = millis();
        fill_solid(leds, NUM_LEDS, CHSV(random8(), 255,255));
      }
      if(millis() - difference2 > _baseData.extra) {
        difference2 = millis();
        fadeToBlackBy(leds, NUM_LEDS, _baseData.color);
      }
      ledsProject();
    } else if(_baseData.qcomm == 32) { //random color JUMP, STRIPWISE
      if(firstRun) {
        difference = millis();
        difference2 = millis();
        leds[random(0, NUM_LEDS)] = CHSV(random8(), 255,255);
        firstRun = 0;
      }
      if(millis() - difference > _baseData.ext1 + random(0,_baseData.ext2)) {
        difference = millis();
        leds[random(0, NUM_LEDS)] = CHSV(random8(), 255,255);
      }
      if(millis() - difference2 > _baseData.ext3) {
        difference2 = millis();
        fadeToBlackBy(leds, NUM_LEDS, _baseData.ext4);
      }
      ledsProject();
    } else if(_baseData.qcomm == 33) { //sine wave palettes, DYNAMIC
      /*
      * pick a color/palette and sine wave profile. 
      * manipulate values of the color with the sine wave
      * stripwise: be able to do color waves
      * podwise: be able to do semi-circle fades
      */
      if(FFACTOR == 2) { //stripwise
        _baseData.qcomm = 35;
      } else { //podwise
        _baseData.qcomm = 34;
      }
      protectedRedo = 1;
    } else if(_baseData.qcomm == 34) { //sine wave palettes, PODWISE
      //beastsin8(bpm, low, high, offset, timeBase);
      //sine wave palettes
      //sine curves
      //uint8_t sinA = 0;
      //uint8_t sinB = 0;
      //uint8_t sinC = 0;
      if(firstRun) {
        paletteSelect(_baseData.extra);
        ghue = 0;
        firstRun = 0;
      }
      switch(_baseData.color) {
        //sine wave select
        case 0:
          //zoomy rainbow style, podwise
          //_.ext1 && _.ext1 are bpm
          sinA = beatsin8(_baseData.ext1, 0, 255);
          sinB = beatsin8(_baseData.ext2, 0, 255);
          fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, (sinA+sinB)/2, 255, LINEARBLEND));
          break;
        case 1:
          //hidden color pulse
          //mostly black but will raise in brightness for specific colors in pattern, would look good slow
          //PODWISE    slow palette advance hopefully makes it a little more special
          /*
          * looks good with
          *  _satelliteData.qcomm = 34;
              _satelliteData.color = 1;
              _satelliteData.extra = 10;
              _satelliteData.ext1 = 10;
              _satelliteData.ext2 = 200;
              _satelliteData.ext3 = 5;
          */
          sinA = beatsin8(_baseData.ext1, _baseData.ext2, 255);
          if(_baseData.sw1 == 0) {
            fill_palette(leds, NUM_LEDS, ghue, _baseData.sw2, colorScroll, sinA, LINEARBLEND);
          } else if (_baseData.sw1 == 1) {
            fill_palette(leds, NUM_LEDS, ghue, 255/NUM_LEDS, colorScroll, sinA, LINEARBLEND);
          }
          if(millis() % _baseData.ext3 == 0) {
            ghue++;
          }
          break;
        case 2:
          //sine wave palette bounce, similar to [case 4] below, locked to ends of a palette
          sinA = beatsin8(_baseData.ext1, 0, 255, 0, 0);
          fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, sinA, 255, LINEARBLEND)); //PODWISE, doesn't need for loop
          break;
        case 3:
          //actual straight up zoomy rainbow, keep for both podwise and stripwise
          sinA = beatsin8(_baseData.ext1, 0, 255);
          sinB = beatsin8(_baseData.ext2, 0, 255);
          fill_rainbow(leds, NUM_LEDS, (sinA + sinB)/2, 1);
          break;
        case 4:
          //basic solid color hue bounce, podwise,
          //ex. fade between hue 200-220 following a sine curve
          sinA = beatsin8(_baseData.ext1, _baseData.ext2, _baseData.ext3, 0, 0);
          fill_solid(leds, NUM_LEDS, CHSV(sinA, 255, 255)); //PODWISE, doesn't need for loop
          
          break;        
      }
      ledsProject();
    } else if(_baseData.qcomm == 35) { //sine wave palettes, STRIPWISE
      //beastsin8(bpm, low, high, offset, timeBase);
      if(firstRun) {
        paletteSelect(_baseData.extra);
        ghue = 0;
        firstRun = 0;
      }
      switch(_baseData.color) {
        //sine wave select
        case 0:
          //zoomy rainbow style, podwise
          sinA = beatsin8(_baseData.ext1, 0, 255);
          sinB = beatsin8(_baseData.ext2, 0, 255);
          fill_palette( leds, NUM_LEDS, (sinA+sinB)/2, 1, colorScroll, 255, LINEARBLEND);
          ledsProject();
          break;
        case 1:
          //hidden color pulse
          //mostly black but will raise in brightness for specific colors in pattern, would look good slow
          //STRIPWISE    make the palette fill statically then have brightness waves going across that reveal the colors of the palette
          /*
          * looks good with
          *  _satelliteData.qcomm = 34;
              _satelliteData.color = 1;
              _satelliteData.extra = 10;
              _satelliteData.ext1 = 10;
              _satelliteData.ext2 = 200;
              _satelliteData.ext3 = 5;
          */
          sinA = beatsin8(_baseData.ext1, _baseData.ext2, 255);
          if(_baseData.sw1 == 0) {
            fill_palette(leds, NUM_LEDS, ghue, _baseData.sw2, colorScroll, sinA, LINEARBLEND);
          } else if (_baseData.sw1 == 1) {
            fill_palette(leds, NUM_LEDS, ghue, 255/NUM_LEDS, colorScroll, sinA, LINEARBLEND);
          }
          if(millis() % _baseData.ext3 == 0) {
            ghue++;
          }
          break;
        case 2:
          //sine wave palette bounce, similar to [case 4] below, locked to ends of a palette
          for(int i = 0; i < NUM_LEDS; i++) {
            sinA = beatsin8(_baseData.ext1, 0, 255, 0, 0);
            leds[i] = ColorFromPalette(colorScroll, sinA, 255, LINEARBLEND); //STRIPWISE
          }
          ledsProject();
          break;
        case 3:
          //actual straight up zoomy rainbow, keep for both podwise and stripwise
          sinA = beatsin8(_baseData.ext1, 0, 255);
          sinB = beatsin8(_baseData.ext2, 0, 255);
          fill_rainbow(leds, NUM_LEDS, (sinA + sinB)/2, 1);
          ledsProject();
          break;
        case 4:
          //basic solid color hue bounce, podwise,
          //ex. fade between hue 200-220 following a sine curve
          //STRIPWISE  fluctuating color gradient along length
          sinA = beatsin8(_baseData.ext1, _baseData.ext2, _baseData.ext3);
          leds[ghue] = CHSV(sinA, 255, 255); //STRIPWISE
          ghue = (ghue + 1) % NUM_LEDS;
          ledsProject();
          break;        
      }
    } else if(_baseData.qcomm == 36) { //comet, DYNAMIC
      //some comet pattern,
      //PODWISE, light POW then uneven fade off
      //STRIPSIE, send a pulse down the length of a stripwise
      if(FFACTOR == 2) { //stripwise
        _baseData.qcomm = 38;
      } else { //podwise
        _baseData.qcomm = 37;
      }
      protectedRedo = 1;
    } else if(_baseData.qcomm == 37) { //comet, PODWISE
      _baseData.qcomm = 38;
      protectedRedo = 1;
    } else if(_baseData.qcomm == 38) { //comet, STRIPWISE
      //use int stripIndex to keep track of 
      /*
      * _.extra  comet size
      * _.speed  delay in comet movement
      * _.color  hue of comet
      */
      if(firstRun) {
        stripIndex = 0;  
        delta = 0;
        firstRun = 0;
        if(_baseData.color >= 300) {
          paletteSelect(_baseData.color - 300);
        }
      }
      //iPos += iDirection;
      if(delta == 1) {
        stripIndex++;
      } else {
        stripIndex--;
      }
  //      if (iPos == (NUM_LEDS - cometSize) || iPos == 0)
  //        iDirection *= -1;
      if(stripIndex == (NUM_LEDS - _baseData.extra) || stripIndex == 0) {
        if(delta == 0) {
          delta = 1;
        } else {
          delta = 0;
        }
      }
  //      for (int i = 0; i < cometSize; i++)
  //        g_LEDs[iPos + i].setHue(hue);
      for(int i = 0; i < _baseData.extra; i++) {
        if(_baseData.color < 256) {
          leds[stripIndex + i] = CHSV(_baseData.color, 255, 255); //could include option for the comet to change colors with ghue, _.color being hue increment
        } else {
          leds[stripIndex + i] = ColorFromPalette(colorScroll, random8(), 255, LINEARBLEND);
        }
      }
      for(int i = 0; i < NUM_LEDS; i++) {
        if(random(10) > 5) {
          //ERRORS this might not work like at all
          leds[i] = leds[i].fadeToBlackBy(100);
        }
      }
      ledsProject();
      delayTime(_baseData.speed);
    } else if(_baseData.qcomm == 39) { //intense flash and fade, brightness ramps
      if(firstRun) {
        if(_baseData.extra == 0) { //pyramid
          int bKey = (int) ((BRIGHTNESS / (float) 255) * _baseData.color); //256% scale
          bool skipRest = false;
          for(int i = 0; i < bKey; i++) {
            FastLED.setBrightness(i);
            ledsProject();
            if(delayTime(_baseData.speed) + random(0, _baseData.randfactor)) {
              skipRest = true;
              break;
            }
          }
          if(!skipRest) {
            for(int i = bKey; i >= 0; i--) {
              FastLED.setBrightness(i);
              ledsProject();
              if(delayTime(_baseData.speed + random(0, _baseData.ext1))) {
                skipRest = true;
                break;
              }
            }
          }
        } else if(_baseData.extra == 1) { //triangle pulse on and fade
          int bKey = (int) ((BRIGHTNESS / (float) 255) * _baseData.color); //256% scale
          for(int i = bKey; i >= 0; i--) {
            FastLED.setBrightness(i);
            ledsProject();
            if(delayTime(_baseData.speed + random(0, _baseData.randfactor))) {
              break;
            }
            if(bKey - i > _baseData.ext1) {
              i += random(0,_baseData.ext1);
            }
          }
        } else if(_baseData.extra == 2) { //semi-circle fade
          
        }
        firstRun = 0;
      }
    } else if(_baseData.qcomm == 40) { //BASIC PALETTE SCROLL DYNAMIC
    //select a pattern and project and advance along the strip
    if(FFACTOR == 2) { //stripwise
        _baseData.qcomm = 42;
      } else { //podwise
        _baseData.qcomm = 41;
      }
      protectedRedo = 1;
    } else if(_baseData.qcomm == 41) { //BASIC PALETTE SCROLL PODWISE
      if(firstRun) {
        int numberAnchorPoints = paletteSelect(_baseData.extra);
        if(_baseData.ext3 == 0) {
          for(int i = 0; i < NUM_STRIPS; i++) {
            stripeIndex[i] = random8();
            stripeColors[i] =  ColorFromPalette(colorScroll, stripeIndex[i], 255, LINEARBLEND);
          }
        } else {
          uint8_t breakIdx = 0;
          uint8_t fracMark = 0;
          for(int i = 0; i < NUM_STRIPS; i++) {
            stripeIndex[i] = _baseData.ext3 + breakIdx;
            if(i > ((NUM_STRIPS - 1) / numberAnchorPoints) * fracMark) {
              fracMark++;
              breakIdx = queryNextBreak(breakIdx, numberAnchorPoints, 1);
            }
          }
          for(int i = 0; i < NUM_STRIPS; i++) {
            stripeColors[i] =  ColorFromPalette(colorScroll, stripeIndex[i], 255, LINEARBLEND);
          }
        }
        firstRun = 0;
      }
      //fill_solid(leds, NUM_LEDS, ColorFromPalette(colorScroll, ghue, 255, LINEARBLEND));
      //ledsProject();
      ledsStripesProject();
      for(int i = 0; i < NUM_STRIPS; i++) {
        if(_baseData.ext2 == 0) {
          stripeIndex[i] += _baseData.ext4;
        } else {
          stripeIndex[i] -= _baseData.ext4;
        }
        stripeColors[i] = ColorFromPalette(colorScroll, stripeIndex[i], 255, LINEARBLEND);
      }
      delayTime(_baseData.speed);
    } else if(_baseData.qcomm == 42) { //BASIC PALETTE SCROLL STRIPWISE
      if(firstRun) {
        paletteSelect(_baseData.extra);
        if(_baseData.ext3 == 0) {
          ghue = random8();
        } else {
          ghue = _baseData.ext3;
        }
        firstRun = 0;
      }
      if(_baseData.sw2 == 0) {
        //fill the string with a gradient
        if(_baseData.sw1 == 0) {
          //scale to fit length
          fill_palette(leds, NUM_LEDS, ghue, 255/NUM_LEDS, colorScroll, 255, LINEARBLEND);
        } else {
          //given offset
          fill_palette(leds, NUM_LEDS, ghue, _baseData.sw1, colorScroll, 255, LINEARBLEND);
        }
      } else {
        //fill each LED with a different color
        for(int i = 0; i < NUM_LEDS; i++) {
          leds[i] = ColorFromPalette(colorScroll, colorIndex[i]++, 255, LINEARBLEND);
        }
      }
      ledsProject();
      if(_baseData.ext2 == 0) {
        ghue += _baseData.ext4;
      } else {
        ghue -= _baseData.ext4;
      }
      delayTime(_baseData.speed);
    }
    
    if(_baseData.qcomm >= 6 && powerStatus == 0 && !protectedRedo) { //THE AUTO RAMP UP
      int bKey = (int) ((BRIGHTNESS / (float) 255) * _baseData.brightKey); //256% scale
      for(int i = bKey; i >= 0; i-=2) {
          FastLED.setBrightness(bKey - i);
          ledsProject();
          delay(1);
      }
      currBrightness = bKey;
      powerStatus = 1;
    }
  }
}
