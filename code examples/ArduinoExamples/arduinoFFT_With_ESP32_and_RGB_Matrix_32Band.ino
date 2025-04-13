
// REQUIRES the following Arduino libraries:
// - Lien vidéo: https://youtu.be/lqTnZvAtL_0
// - RGB LED Matrix with an ESP32 - How to get started : https://youtu.be/0gGnr2HYCnQ
// - ArduinoFFT Library: https://github.com/kosme/arduinoFFT
// - Adafruit RGB matrix Panel Library: https://github.com/adafruit/RGB-matrix-Panel
// - Adafruit_GFX Library: https://github.com/adafruit/Adafruit-GFX-Library
// - Adafruit_BusIO Library: https://github.com/adafruit/Adafruit_BusIO
// - Getting Started ESP32 / ESP32S : https://www.youtube.com/watch?v=9b0Txt-yF7E
// Find All "Great Projects" Videos : https://www.youtube.com/c/GreatProjects


#include <Adafruit_GFX.h>
#include <RGBmatrixPanel.h>
#include <gamma.h>
#include <arduinoFFT.h>

#define A    23
#define B    19
#define C    5
#define D    17
#define CLK  16 
#define LAT  4 
#define OE   15
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false, 64);
//ArduinoFFT FFT = ArduinoFFT();

uint32_t lastTime, lastdoT, lastblack; 
#define SAMPLES 64
const double samplingFrequency = 44100;
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency);

int Exval[32] = {0};
int Exdot[32] = {0};
int vR, yh, vpx, yx;
int sensorPin = 36;    // select the input pin for the Mic
// standard colors
uint16_t myRED = matrix.Color333(7,0,0);
uint16_t myGREEN = matrix.Color333(0,7,0);
uint16_t myBLUE = matrix.Color333(0,0,7);
uint16_t myWHITE = matrix.Color333(7, 7,7);
uint16_t myYELLOW = matrix.Color333(7,7,0);
uint16_t myCYAN = matrix.Color333(0,7,7);
uint16_t myMAGENTA = matrix.Color333(7,0,7);
uint16_t myShadow = matrix.Color333(4,0,7);
uint16_t myROSE = matrix.Color333(7,0,4);
uint16_t myBLACK = matrix.Color333(0,0,0);
uint16_t myCOLORS[7] = {myRED, myYELLOW, myCYAN, myBLUE, myWHITE, myMAGENTA, myGREEN};
uint8_t ac = 0;
long          hue = 0, hueval;
int           ax1, ax2, ax3, ax4, ay1, ay2, ay3, ay4;
int           sx1, sx2, sx3, sx4;
float         sy1, sy2, sy3, sy4;

void getdoT() {
  for(int x=0; x<32; x++) {
      int xl = x+32, xr = 63 - xl;
      yh = Exdot[x]; 
      if (yh < 32) { 
      matrix.fillRect(xl, yh-1, 1, 2, myBLACK);
      matrix.fillRect(xr, yh-1, 1, 2, myBLACK);
      matrix.fillRect(xl, yh, 1, 1, (myCOLORS[ac]));
      matrix.fillRect(xr, yh, 1, 1, (myCOLORS[ac]));
      Exdot[x]++; 
  }
  }
      ac++;
      if (ac > 6) { ac = 0; }    
}
void getblack() {
  for(int x=0; x<32; x++) {
      int xl = x+32, xr = 63 - xl;
      yh = Exval[x]; 
      if (yh < 31) { 
      matrix.fillRect(xl, yh, 1, 1, myBLACK);
      matrix.fillRect(xr, yh, 1, 1, myBLACK);
      Exval[x]++; 
  }
  }    
}
void displayBand(int ban, int vmic){
      int xl = ban+32, xr = 63 - xl;
      for(int x=32; x>vmic; x--) { 
      hueval = hue
        + (uint8_t)((ax1 * ax1 + ay1 * ay1) >> 6)
        + (uint8_t)((ax2 * ax2 + ay2 * ay2) >> 6)
        + (uint8_t)((ax3 * ax3 + ay3 * ay3) >> 5)
        + (uint8_t)((ax4 * ax4 + ay4 * ay4) >> 5);
      matrix.drawPixel(xl, x, matrix.ColorHSV(hueval * 3, 255, 255, true));
      matrix.drawPixel(xr, x, matrix.ColorHSV(hueval * 3, 255, 255, true));
      ax1--; ax2--; ax3--; ax4--;
  }
      if (Exdot[ban] > vmic) { Exdot[ban] = vmic; }
      if (Exval[ban] > vmic) { Exval[ban] = vmic; }
}
void getdata(){
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(sensorPin); // A conversion takes about 1uS on an ESP32
    vImag[i] = 0;
  }
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
  sx1 = (int)(cos(sy1) * 20);
  sx2 = (int)(cos(sy2) * 40);
  sx3 = (int)(cos(sy3) * 80);
  sx4 = (int)(cos(sy4/32) * 120);
  ay1  = (int)(sin(sy1) * 20);
  ay2  = (int)(sin(sy2) * 40);
  ay3  = (int)(sin(sy3) * 80);
  ay4  = (int)(sin(sy4/32) * 120);
  hue = sy4;
  sy4 =0.00;
  for (int i = 2; i < SAMPLES/2; i++){ 
    ax1 = sx1; ax2 = sx2; ax3 = sx3; ax4 = sx4;
    vR = map((int)vReal[i], 24000, 0, 0, 31);
    if (vR < 0) { vR = 0; }
    sy4+=vR;
    if (vReal[i] > 500) {
      if (i<=2 )           displayBand(0, vR);
      if (i>2   && i<=3  ) displayBand(1, vR);
      if (i>3   && i<=5  ) displayBand(2, vR);
      if (i>5   && i<=7  ) displayBand(3, vR);
      if (i>7   && i<=24 ) displayBand(i-4, vR);
      if (i>24  && i<=27 ) displayBand(21, vR);
      if (i>27  && i<=32 ) displayBand(22, vR);
      if (i>32  && i<=41 ) displayBand(23, vR);
      if (i>41  && i<=50 ) displayBand(24, vR);
      if (i>50  && i<=60 ) displayBand(25, vR);
      if (i>60  && i<=70 ) displayBand(26, vR);
      if (i>70  && i<=74 ) displayBand(27, vR);
      if (i>74  && i<=80 ) displayBand(28, vR);
      if (i>80 && i<=100 ) displayBand(29, vR);
      if (i>100 && i<=120 ) displayBand(30, vR);
      if (i>120 )           displayBand(31, vR);
    }
    ay1--; ay2--; ay3--; ay4--;
}
    sy1+=0.02;
    sy2+=0.07;
    sy3+=0.13;
}
void setup() {
  pinMode(sensorPin, INPUT);
  matrix.begin();                           // setup the LED matrix
  matrix.setTextWrap(false); // Don't wrap at end of line - will do ourselves
  matrix.fillScreen(0);
  delay(1000);
}
void loop() {
  if(millis() - lastTime >= 7) {
      getdata();
      lastTime = millis();
    }
    if (millis() - lastblack >= 4) {
      getblack();
      lastblack = millis();
  }
  if(millis() - lastdoT >= 100) {
      getdoT();
      lastdoT = millis();
    }
}
