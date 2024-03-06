//27-8-22 Untested on the ground

#include <PID_v1.h> //pid library

// Define Input Connections
#define CH3 23   //Left Flysky Stick
#define CH2 22  //Right Flysky Stick

#define DAC1 25   //left
#define DAC2 26   //right

#define L_Rev 32  //reverse pin
#define L_Break 27   //Left break

#define R_Rev 33 //reverse pin
#define R_Break 14 //Right break


#define L_enc_A 5  // Left Rotary Encoder phase A
#define L_enc_B 17 // Left Rotary Encoder phase B
#define R_enc_A 2  // Right Rotary Encoder Phase A
#define R_enc_B 15 // Right Rotary Encoder Phase B

#define PPR 2560  //pulses per revolution. Actual PPR for the encoder 
                  //is 1024 but 2560 is after dividing it by 0.4 to 
                  //accomodate the gear step up

#define test  //comment #define test out to disable the pid code

#ifdef test  // if #define test in commented then the code between #ifdef test and 
             // #endif will not be compiled

double Setpoint_L, Input_L, Output_L;   //Define Variables we'll be connecting to
                                        //for the left track

//Define the aggressive and conservative Tuning Parameters
//the tuning parameters will be shared across both tracks
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters for the left track
PID myPID_L(&Input_L, &Output_L, &Setpoint_L, consKp, consKi, consKd, DIRECT);


double Setpoint_R, Input_R, Output_R;   //Define Variables we'll be connecting to
                                        //for the right track

//Specify the links and initial tuning parameters for the right track
PID myPID_R(&Input_R, &Output_R, &Setpoint_R, consKp, consKi, consKd, DIRECT);

#endif 

// Integers to represent values from sticks and pots
int L_Val;  //flysky
int R_Val;  //flysky

//rpm_map will map the rpm so that it is in the same range as the flysky map
//making it good values for the pid to work with.
int rpm_map_L; 
int rpm_map_R; 

//to count the pulses
volatile unsigned long count_L=0; 
volatile unsigned long count_R=0;  

//rpm used to calculate rpm as the function in the loop
int rpm_L = 0;
int rpm_R = 0;

//variable to store the encoder readings
int enc_L;
int enc_R;


hw_timer_t * timer = NULL;

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

//pulse functions will increment the count everytime the encoder pulses and
//performs rpm calculations based on that in real-time

void pulse_L() {
  
  count_L = count_L + 1; 

  enc_L = digitalRead(L_enc_B);       //reads the pulse of the encoders phase B when phase A
  if (digitalRead (L_enc_B) != 1){    //pulses. Phase A is read on the rising edge and causes an 
    rpm_L = (count_L * 60 / PPR);     //inturrupt trigering the pulse function, and phase b is 90 
  } else {                            //degrees out of phase compared to A. This can be used to 
    rpm_L = (count_L * 60 / PPR)*-1;  //determine the direction the encoder is turning
  }
}


void pulse_R() {

  count_R = count_R + 1; 

  enc_R = digitalRead(R_enc_B);       //The encoders are opposite to each other, thus the * -1
  if (digitalRead (R_enc_B) != 1){    //is multiplied when b is not equal to 1 for one off the 
    rpm_R = (count_R * 60 / PPR)*-1;  //functions and when b is equal for the other pulse function
  } else {                            //to correct the direction of travel of the track
    rpm_R = (count_R * 60 / PPR);
  }
}

//the funtion below is triggered every second and thus refreshes the rpm printed every second
void IRAM_ATTR onTimer(){
  
Serial.print("phase A (rpm) ");
Serial.print(rpm_map_L);
Serial.print(" phase B ");
Serial.println(enc_L);
Serial.print("phase B (rpm) ");
Serial.print(rpm_map_R);
Serial.print(" phase B ");
Serial.println(enc_R);

count_L = 0;    //resets the count every second when the funtion is triggered otherwise
count_R = 0;    //the rpm will keep rising infinitly
   
}

//reversing function
void reverse(int val, int rev_pin){
if(val < 0 ){
    digitalWrite(rev_pin, HIGH);
} else {
    digitalWrite(rev_pin, LOW);
}
return;
}   


void setup(){

Serial.begin(115200);   // Set up serial monitor and has to be 115200 for the esp32
  
pinMode(CH3, INPUT); //channel 3 input
pinMode(CH2, INPUT); //channel 4 input

pinMode(L_Rev, OUTPUT); 
pinMode(R_Rev, OUTPUT);

pinMode(L_Break, OUTPUT); 
pinMode(R_Break, OUTPUT);
  
// Set encoder pins as inputs  
pinMode (L_enc_A,INPUT);
pinMode (L_enc_B,INPUT);
pinMode (R_enc_A,INPUT);
pinMode (R_enc_B,INPUT);   

//Attach inturrupt
attachInterrupt(digitalPinToInterrupt(L_enc_A), pulse_L, RISING);  //inturrupt the pulse at rising edge
attachInterrupt(digitalPinToInterrupt(R_enc_A), pulse_R, RISING);  //and trigger the pulse functions

//timer function
timer = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
timerAlarmWrite(timer, 1000000, true); // 1000000 * 1 us = 1 s, autoreload true
timerAlarmEnable(timer); // enable

#ifdef test   //comment #define test out to disable the pid code

Input_L = rpm_map_L;
Input_R = rpm_map_R;    //initialize the variables we're linked to

Setpoint_L = 100;
Setpoint_R = 100;
  
myPID_L.SetMode(AUTOMATIC);    //turn the PID on
myPID_R.SetMode(AUTOMATIC);

//turn the PID on
myPID_L.SetOutputLimits(-240,240);   // Clamping and this will help
myPID_R.SetOutputLimits(-240,240);   //reducing overcompensation

#endif
}

void loop() {
  
L_Val = readChannel(CH3, -255, 255, 0);  //Reads value from FlySky to the ESP32
R_Val = readChannel(CH2, -255, 255, 0);

rpm_map_L = map(rpm_L, -355, 355, -255, 255);
rpm_map_R = map(rpm_R, -355, 355, -255, 255);
  
reverse(L_Val, L_Rev);
reverse(R_Val, R_Rev);
//digitalWrite(R_Rev,HIGH);

  
if(abs(L_Val) > 4 ){ //deadzone value
digitalWrite(L_Break, LOW);
dacWrite(DAC1,abs(L_Val));

} else {
dacWrite(DAC1,0);
digitalWrite(L_Break, HIGH);
}

if(abs(R_Val) > 4 ){ 
digitalWrite(R_Break, LOW);
dacWrite(DAC2,abs(R_Val));
  
} else {
dacWrite(DAC2,0);
digitalWrite(R_Break, HIGH);
}


#ifdef test

Input_L = rpm_map_L;
Input_R = rpm_map_R;

double gap_L = abs(L_Val-Input_L); //distance away from setpoint

//we're close to setpoint, use conservative tuning parameters
if (gap_L < 10) {  
myPID_L.SetTunings(consKp, consKi, consKd);
 
} else {
//we're far from setpoint, use aggressive tuning parameters
myPID_L.SetTunings(aggKp, aggKi, aggKd);
}
  
myPID_L.Compute();
analogWrite(DAC1, Output_L);

double gap_R = abs(R_Val-Input_R); //distance away from setpoint

//we're close to setpoint, use conservative tuning parameters
if (gap_R < 10) {  
myPID_R.SetTunings(consKp, consKi, consKd);

} else {
//we're far from setpoint, use aggressive tuning parameters
myPID_R.SetTunings(aggKp, aggKi, aggKd);
}
  
myPID_R.Compute();
analogWrite(DAC2, Output_R);

#endif 

}
