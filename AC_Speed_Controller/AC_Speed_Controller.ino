#include <EEPROM.h>
#include <PID_AutoTune_v0.h>
#include <TimerOne.h>
#include <PID_v1.h>

#define triacPin 5      // Output to Triac
#define inputPin A0     // Input from CNC controller
#define tachPin 3       // Input from Tach signal
#define zeroPin 2       // Input from Zero Cross Detector
#define autoTune 4      // Input to activate PID AutoTune
#define FREQ 60 	// 60Hz power in these parts

bool firstRun = true; //Initialize PID settings after code reload

double Setpoint = 0, Input = 0, Output = 0;  //Define Variables we'll be connecting PID to
double wait = 3276700000;  //find the squareroot of this in your spare time please
int tachMax = 29667;  // Maximum motor RPM
int inputScaler = tachMax/1023; //scalar value to convert analog input to RPM value for PID input
volatile long tachCount = 0;  // Counter for tach signal
volatile byte state = 255;  // controls what interrupt should be attached or detached while in the main loop
unsigned long int period = 1000000 / (2 * FREQ);  //The Timerone period in uS, 60Hz = 8333 uS
int inputWindow = 1000;  // time in milliseconds that ADC0 input is sampled and PID setpoint is adjusted (default 1000 = 1s)
double now, then, timeOld;  //timer variables for main loop
int inputTemp = 0;

//setup PID and AutoTune parameters
byte ATuneModeRemember=2;
double kp=.0821,ki=0.725,kd=.028;
double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
boolean tuning = false;
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, REVERSE);
PID_ATune aTune(&Input, &Output);

void setup() {
  //Retrieve stored PID tunings
  firstRun = EEPROM.read(0);
  if (!firstRun){
    getPIDtune();
  }
  else {
    setPIDtune();
  }
  
  // setup pins
  pinMode(triacPin, OUTPUT);
  digitalWrite(triacPin, LOW);
  pinMode(tachPin, INPUT_PULLUP);
  pinMode(zeroPin, INPUT);
  pinMode(autoTune, INPUT_PULLUP);
  
  //set PID calculation frequency
  myPID.SetSampleTime(200);
  
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, period);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  //setup timer for triac gate trigger
  Timer1.initialize(period);
  Timer1.disablePwm(9);
  Timer1.disablePwm(10);
  
  //Initialize timer variables
  timeOld = 0;
  then = 0;
  
  //setup ISR routines
  attachInterrupt(0,zeroCross, FALLING);  //IRQ0 is pin 2. Call zeroCrossingInterrupt on FALLING signal
  attachInterrupt(1,tachSense, FALLING);  //IRQ1 is pin 3. Call tachSense on FALLING signal
  
}

void loop() {
  if(!digitalRead(autoTune)) {
    changeAutoTune();
  }
  now = millis();
  if(now-then >= inputWindow){
    Setpoint = analogRead(triacPin)*inputScaler;
    then = now;
  }
  if (tachCount >= 10){
    Input = (60000/(millis() - timeOld)) * tachCount;
    timeOld = millis();
    tachCount = 0;
  }
  if (!tuning){
    myPID.Compute();
  }
  else {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning){ //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      setPIDtune();
      AutoTuneHelper(false);
    }
  }
}


void zeroCross(){  //Zero cross detection ISR
  Timer1.restart();
  if (Output>=7915) {			//if Output is < 5%
    digitalWrite(triacPin, LOW);	//stay off all the time
  }
  else	//otherwise we want the motor at some middle setting
  {
    Timer1.attachInterrupt(nowIsTheTime,Output);
    state = false;
  }

}

void tachSense(){  //Tach input counter
  tachCount++;
}

void nowIsTheTime ()
{
  if (!state){
    Timer1.attachInterrupt(nowIsTheTime, (Output + 8333));
  }
  digitalWrite(triacPin,HIGH);
  wait = sqrt(wait);		//delay wont work in an interrupt.
  if (!wait)                      // this takes 80uS or so on a 16Mhz proc
  {
    wait = 3276700000;
  }
  digitalWrite(triacPin,LOW);
  if (state){
    Timer1.detachInterrupt();
  }
  else state = true;
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    Output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void setPIDtune(){
  EEPROM.write(0,0);
  EEPROM.write(1,kp);
  EEPROM.write(2,ki);
  EEPROM.write(3,kd);
}

void getPIDtune(){
  kp = EEPROM.read(1);
  ki = EEPROM.read(2);
  kd = EEPROM.read(3);
}

