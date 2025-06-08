// OPEN A NEW SKETCH WINDOW IN ARDUINO
// CLICK IN THIS BOX, CTL-A, CTL-C (Copy code from text box.)
// CLICK IN SKETCH, CTL-A, CTL-V (Paste code into sketch.)

// Breathing Rate Detection System -- Final Integration
//
// Pieced together from code created by: Clark Hochgraf and David Orlicki Oct 18, 2018
// Modified by: Mark Thompson April 2020 to integrate MATLAB read and Write 
//              and integrate the system

#include <MsTimer2.h>
#include <SPI.h>
#include <Tone2.h>


const int TSAMP_MSEC = 100;
const int NUM_SUBSAMPLES = 160, NUM_SAMPLES = 3500;
const int DAC0 = 3, DAC1 = 4, DAC2 = 5, LM61 = A0, VDITH = A1;
const int V_REF = 5.0;
const int SPKR = 12;

volatile boolean sampleFlag = false;

const long DATA_FXPT = 1000; // Scale value to convert from float to fixed

const float INV_FXPT = 1.0 / DATA_FXPT; // division slow: precalculate

float eqOutputFlt = 0.0;

float volts, degC;

int nSmpl = 1, sample;

Tone toneT2;
Tone toneT1;
int toneTimer = 0;

int alarmInt = 0;

float xv, yv, yLF, yMF, yHF, stdLF, stdMF, stdHF;
float printArray[9];
int numValues = 0;


int loopTick = 0;
bool statsReset;
bool isToneEn = false;

unsigned long startUsec, endUsec, execUsec;

//  Define a structure to hold statistics values for each filter band
struct stats_t
{
  int tick = 1;
  float mean, var, stdev;
} statsLF, statsMF, statsHF;

//**********************************************************************
void setup() // Leave alone
{

  configureArduino();
  Serial.begin(115200);delay(5);

   //Handshake with MATLAB 
  Serial.println(F("%Arduino Ready"));
  while (Serial.read() != 'g'); // spin

  toneT2.begin(13);
  toneT1.begin(SPKR);
  MsTimer2::set(TSAMP_MSEC, ISR_Sample); // Set sample msec, ISR name
  MsTimer2::start(); // start running the Timer  


}


////**********************************************************************
void loop()
{
// Breathing rate detection

  long fxdInputValue, lpfInput, lpfOutput;
  long eqOutputFxd;
  int alarmCode;  //  Alarm code

  syncSample();

  //  Read sample from ADC
  xv = analogReadDitherAve();
  
  // Convert the ADC value to temperature

  fxdInputValue = long(DATA_FXPT * xv + 0.5); //Input for eq f(x)

  eqOutputFxd = Equalizer( fxdInputValue ); // Equalizer f(x) used here
  eqOutputFlt = float(eqOutputFxd) * INV_FXPT;

  yLF = IIR_LOWRATE(eqOutputFlt);  // 5th Cheb Low Pass 12
  yHF = IIR_HIGHRATE(eqOutputFlt); // High Pass
  yMF = IIR_MIDRATE(eqOutputFlt);  //Band Pass
  
  // Compute stats for each filter
  statsReset = (statsLF.tick % 100 == 0); //Interval size
  getStats(yLF, statsLF, statsReset); 
  stdLF = statsLF.stdev;
  getStats(yMF, statsMF, statsReset); 
  stdMF = statsMF.stdev;
  getStats(yHF, statsHF, statsReset); 
  stdHF = statsHF.stdev;

  numValues = 9;  // The number of columns to be sent to the serial monitor (or MATLAB)

  alarmInt = AlarmCheck(stdLF, stdMF, stdHF);
  setAlarm(alarmInt, true);

  printArray[0] = loopTick;    //  The sample number -- always print this
  printArray[1] = eqOutputFxd; //  Column 2
  printArray[2] = yLF;         // Low pass output
  printArray[3] = yMF;         // Band pass output
  printArray[4] = yHF;         // High pass output
  printArray[5] = stdLF;       // Deviation LF
  printArray[6] = stdMF;       // Deviation MF
  printArray[7] = stdHF;       // Deviation HF
  printArray[8] = alarmInt;    //debug for the AlarmCheck variable in matlab

 WriteToSerial( numValues, printArray );  //  Write to the serial monitor (or MATLAB)

 if (++loopTick >= NUM_SAMPLES) while(true); // spin forever

  //  Execute the noise filter.  
  // eqOutput = NoiseFilter( eqOutput, loopTick );
  
  // ******************************************************************
  //  When finding the impulse responses of the filters use this as an input
  //  Create a Delta function in time with the first sample a 1 and all others 0
  //  xv = (loopTick == 0) ? 1.0 : 0.0; // impulse test input

  // ******************************************************************
  //  Use this when the test vector generator is used as an input
  //  xv = testVector();
} // loop()

//******************************************************************
int AlarmCheck(float stdLF, float stdMF, float stdHF) {
  if (stdLF > stdMF && stdLF > stdHF) return 1; // Low rate (<12 BPM)
  else if (stdHF > stdLF && stdHF > stdMF) return 2; // High rate (>40 BPM)
  else return 0; // Normal rate (12–40 BPM)
}

//******************************************************************
long Equalizer(long xInput )
{

  int i;
  long yN=0; //  Current output
  const int equalizerLength = 4;
  static long xN[equalizerLength] = {0};
  long h[] = {1, 1, -1, -1};  // Impulse response of the equalizer

  //  Update the xN array

  for ( i = equalizerLength-1 ; i >= 1; i-- )
  {
    xN[i] = xN[i - 1];
  }

  xN[0] = xInput;

  //  Convolve the input with the impulse response

  for ( i = 0; i <= equalizerLength-1 ; i++)
  {
    yN += h[i] * xN[i];
  }

  if (loopTick < equalizerLength)
  {
    return 0;
  }
  else
  {
   return yN;
  }

}

//*******************************************************************************
float IIR_LOWRATE (float xv)
{  

  //CHEBY low, order 5, R = 0.1, 12 BPM

  const int MFILT = 6;
  static float GAIN = 3.6e-06;
  static float b[] = {0.1000000, 0.5000000, 1.0000000, 1.0000000, 0.5000000, 0.1000000};
  static float a[] = {1.0000000, -4.7618175, 9.0932768, -8.7039957, 4.1757810, -0.8032330};
  
  //---------------------------------------------------------------
  //  Two arrays to contain the input and output sequences in time
  static float xM[MFILT] = {0.0}, yM[MFILT] = {0.0}; 


 //  Shift the input and output values in time by one sample, then bring in the next sample 
  for (int i = MFILT-1; i > 0; i--) // shift x, y histories
  {
    yM[i] = yM[i-1];
    xM[i] = xM[i-1];
  } 
  xM[0] = GAIN*xv; // insert new input
  yM[0] = 0.0;     // init output accumulator

  //  Execute the IIR filter by multiplying each output historical value by the a ( the numerator) coefficients
  //  and each new input value by the b (the denominator) coefficients.
  
  for (int i = MFILT-1; i > 0; i--) yM[0] += ( -a[i]*yM[i] + b[i]*xM[i]);

  // Update the output with the newest value xM[0] times b[0]
  return (yM[0] += b[0]*xM[0]);
}
//*******************************************************************************
float IIR_HIGHRATE (float xv)
{  

  //CHEBY high, order 5, R = 0.1, 40 BPM

  const int MFILT = 6;
  static float GAIN = 4.79778;
  static float b[] = {0.1000000, -0.5000000, 1.0000000, -1.0000000, 0.5000000, -0.1000000};
  static float a[] = {1.0000000, -3.5520165, 5.2100854, -3.8946403, 1.4744879, -0.2216499};
  
  //---------------------------------------------------------------
  //  Two arrays to contain the input and output sequences in time
  static float xM[MFILT] = {0.0}, yM[MFILT] = {0.0}; 


 //  Shift the input and output values in time by one sample, then bring in the next sample 
  for (int i = MFILT-1; i > 0; i--) // shift x, y histories
  {
    yM[i] = yM[i-1];
    xM[i] = xM[i-1];
  } 
  xM[0] = GAIN*xv; // insert new input
  yM[0] = 0.0;     // init output accumulator

  //  Execute the IIR filter by multiplying each output historical value by the a ( the numerator) coefficients
  //  and each new input value by the b (the denominator) coefficients.
  
  for (int i = MFILT-1; i > 0; i--) yM[0] += ( -a[i]*yM[i] + b[i]*xM[i]);

  // Update the output with the newest value xM[0] times b[0]
  return (yM[0] += b[0]*xM[0]);
}
//*******************************************************************************
float IIR_MIDRATE(float xv)
{  

  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 5;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****

//CHEBY bandpass, order 5, R= 0.1, [12 40] BPM

G[0] = 0.1167359;
b[0][0] = 1.0000000; b[0][1] = -0.0007646; b[0][2]= -0.9996017;
a[0][0] = 1.0000000; a[0][1] =  -1.7232206; a[0][2] =  0.8364823;
G[1] = 0.1167359;
b[1][0] = 1.0000000; b[1][1] = 2.0009417; b[1][2]= 1.0009420;
a[1][0] = 1.0000000; a[1][1] =  -1.8036776; a[1][2] =  0.8525724;
G[2] = 0.1167359;
b[2][0] = 1.0000000; b[2][1] = 1.9996397; b[2][2]= 0.9996400;
a[2][0] = 1.0000000; a[2][1] =  -1.9045146; a[2][2] =  0.9260447;
G[3] = 0.1167359;
b[3][0] = 1.0000000; b[3][1] = -1.9997055; b[3][2]= 0.9997056;
a[3][0] = 1.0000000; a[3][1] =  -1.7461564; a[3][2] =  0.9279234;
G[4] = 0.1167359;
b[4][0] = 1.0000000; b[4][1] = -2.0001113; b[4][2]= 1.0001113;
a[4][0] = 1.0000000; a[4][1] =  -1.9648295; a[4][2] =  0.9790860;
//  **** Stop copying MATLAB code here  ****

  //  Initialize the timer for each point computed
  startTime = micros();


  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }

  execUsec += micros()-startTime;
  
  return yv;
}

//*******************************************************************
void getStats(float eqOutputFlt, stats_t &s, bool reset)
{
  float oldMean, oldVar;
  
  if (reset == true)
  {
    s.stdev = sqrt(s.var/s.tick);
    s.tick = 1;
    s.mean = eqOutputFlt;
    s.var = 0.0;  
  }
  else
  {
    oldMean = s.mean;
    s.mean = oldMean + (eqOutputFlt - oldMean)/(s.tick+1);
    oldVar = s.var; 
    s.var = oldVar + (eqOutputFlt - oldMean)*(eqOutputFlt - s.mean);      
  }
  s.tick++;  
}

//*******************************************************************
float analogReadDitherAve(void)
{ 
 
float sum = 0.0;
int index;
  for (int i = 0; i < NUM_SUBSAMPLES; i++)
  {
    index = i;
    digitalWrite(DAC0, (index & B00000001)); // LSB bit mask
    digitalWrite(DAC1, (index & B00000010));
    digitalWrite(DAC2, (index & B00000100)); // MSB bit mask
    sum += analogRead(LM61);
  }
  return sum/NUM_SUBSAMPLES; // averaged subsamples 

}

//*********************************************************************
void setAlarm(int aCode, boolean isToneEn) {
  //int toneTimer = 0;
  //noTone(SPKR); // Turn off previous tone
  switch (aCode) {
    
    case 1:
    toneT1.play(400); break;  // Low rate: 400 Hz
    
    case 2:
    if(toneTimer < 10){ //under 10, should play for a second
      toneT1.play(1000);
    }
    else{ //no longer under 10
      if(toneTimer < 20){ //still above 10 but under 20 now
        toneT1.stop(); //no sound for a second hopefully
      }
      else{ //once over 20, goes back to 0
        toneTimer = 0;
      }
    }
    break; 
    
    default: toneT1.stop(); break; // Normal: no tone
  }
  toneTimer++;
}


//*************************************************************
float testVector(void)
{
  // Variable rate sinusoidal input
  // Specify segment frequencies in bpm.
  // Test each frequency for nominally 60 seconds.
  // Adjust segment intervals for nearest integer cycle count.
    
  const int NUM_BAND = 6;
  const float CAL_FBPM = 10.0, CAL_AMP = 2.0; 
  
  const float FBPM[NUM_BAND] = {5.0, 10.0, 15.0, 20.0, 30.0, 70.0}; // LPF test
  static float bandAmp[NUM_BAND] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  //  Determine the number of samples (around 600 ) that will give you an even number
  //  of full cycles of the sinewave.  This is done to avoid a large discontinuity 
  //  between bands.  This forces the sinewave in each band to end near a value of zer
  
  static int bandTick = int(int(FBPM[0]+0.5)*(600/FBPM[0]));
  static int simTick = 0, band = 0;
  static float Fc = FBPM[0]/600, cycleAmp = bandAmp[0];

  //for (int i = 0; i < NUM_BAND; i++) bandAmp[i] = CAL_AMP*(CAL_FBPM/FBPM[i]);  

  //  Check to see if the simulation tick has exceeded the number of tick in each band.
  //  If it has then switch to the next frequency (band) again computing how many
  //  ticks to go through to end up at the end of a cycle.
  
  if ((simTick >= bandTick) && (FBPM[band] > 0.0))
  {

    //  The simTick got to the end of the band cycle.  Go to the next frequency
    simTick = 0;
    band++;
    Fc = FBPM[band]/600.0;
    cycleAmp = bandAmp[band];
    bandTick = int(int(FBPM[band]+0.5)*(600/FBPM[band]));
  }
 
 // float degC = 0.0; // DC offset
 // degC += cycleAmp*sin(TWO_PI*Fc*simTick++);  
  //degC += 1.0*(tick/100.0); // drift: degC / 10sec
  //degC += 0.1*((random(0,101)-50.0)/29.0); // stdev scaled from 1.0
  return degC;
}

//*******************************************************************
void configureArduino(void)
{
  pinMode(DAC0,OUTPUT); digitalWrite(DAC0,LOW);
  pinMode(DAC1,OUTPUT); digitalWrite(DAC1,LOW);
  pinMode(DAC2,OUTPUT); digitalWrite(DAC2,LOW);
  pinMode(SPKR, OUTPUT); digitalWrite(SPKR,LOW);
  analogReference(DEFAULT); // DEFAULT, INTERNAL
  analogRead(LM61); // read and discard to prime ADC registers
  Serial.begin(115200); // 11 char/msec 
}


//**********************************************************************
void WriteToSerial( int numValues, float dataArray[] )
{

  int index=0; 
  for (index = 0; index < numValues; index++)
  {
    if (index >0)
    {
      Serial.print('\t');
    }
      Serial.print(dataArray[index], DEC);
  }

  Serial.print('\n');
  delay(20);

}  // end WriteToMATLAB

////**********************************************************************
float ReadFromMATLAB()
{
  int charCount;
  bool readComplete = false;
  char inputString[80], inChar;


  // Wait for the serial port

  readComplete = false;
  charCount = 0;
  while ( !readComplete )
  {
    while ( Serial.available() <= 0);
    inChar = Serial.read();

    if ( inChar == '\n' )
    {
      readComplete = true;
    }
    else
    {
      inputString[charCount++] = inChar;
    }
  }
  inputString[charCount] = 0;
  return atof(inputString);

} // end ReadFromMATLAB

//*******************************************************************
void syncSample(void) //ADC PART
{
  while (sampleFlag == false); // spin until ISR trigger
  sampleFlag = false;          // disarm flag: enforce dwell  
}

//**********************************************************************
void ISR_Sample()
{
  sampleFlag = true;
}
