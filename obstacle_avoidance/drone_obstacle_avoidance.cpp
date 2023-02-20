#include<NewPing.h>  // Ultrasonic sensor functions of drone using Arduino; Download this library by going to <Tools> & then <Manage Libraries>
#include<Servo.h>    // Controls motor functions of drone using Arduino; Download this library by going to <Tools> & then <Manage Libraries>
#include <FastLED.h> // May need to use other library since we're not using Arduino; download from github: https://github.com/FastLED/FastLED

CRGB leds[1];
Servo out1; //roll
Servo out2; //pitch
Servo out3; //throttle
int trig1 = 2;
int echo1 = 2;
//int trig2 = A1;
//int echo2 = A1;
int trig3 = 4;
int echo3 = 4;
int trig4 = 5;
int echo4 = 5;
int trig5 = 6;
int echo5 = 6;
int trig6 = 7;
int echo6 = 7;
int MAX_DISTANCE = 400;               
NewPing sonar1(trig1,echo1,MAX_DISTANCE);
//NewPing sonar2(trig2,echo2,MAX_DISTANCE);
NewPing sonar3(trig3,echo3,MAX_DISTANCE);
NewPing sonar4(trig4,echo4,MAX_DISTANCE);
NewPing sonar5(trig5,echo5,MAX_DISTANCE);
NewPing sonar6(trig6,echo6,MAX_DISTANCE);
float dist1,dist2,dist3,dist4,dist5,dist6; 
unsigned long counter_1, counter_2, counter_3, current_count;
byte last_CH1_state, last_CH2_state, last_CH3_state;
int input_PITCH;    //input on D9 of arduino
int input_ROLL;     //input on D8 of arduino
int input_THROTTLE; //input on D10 of arduino
int DN;
int UP;
int antiDN;
int prev_time = 0; 
void setup(){    
 // Serial.begin(115200);
FastLED.addLeds<WS2812, 3, GRB>(leds, 1);
PCICR |= (1 << PCIE0);                                                   
PCMSK0 |= (1 << PCINT0);  
PCMSK0 |= (1 << PCINT1);                                              
PCMSK0 |= (1 << PCINT2);
  out1.attach(13); //roll
  out2.attach(11); //pitch
  out3.attach(12); //throttle
pinMode(A0,INPUT);
}

void loop(){
  int a=160;
  int b=160;
  int c=160;
  int d=160;
  int e=160;

if(input_ROLL>1600){
d=210;  
}
else if(input_ROLL<1400){
c=210;  
}
if(input_PITCH>1600){
a=210;  
}
else if(input_PITCH<1400){
b=210;  
}

dist1 = sonar1.ping_cm();
dist2 = sonar6.ping_cm();
dist3 = sonar3.ping_cm();
dist4 = sonar4.ping_cm();
dist5 = sonar5.ping_cm();
dist6 = 200; // This Sensor is disabled due to my bad sensor.If you want to enable just replace "200" with "sonar6.ping_cm()"
 
byte roll = map(input_ROLL, 1108, 1856, 55, 125); //maping roll
byte pitch = map(input_PITCH, 1180, 1856, 55, 125); //maping pitch
byte throttle = map(input_THROTTLE, 1160, 1830, 55, 125); //maping throttle

 byte antifront = map(dist1, 6, 210, 55, 60); //force to back
 byte antiback = map(dist2, 6, 210, 125, 120); //force to front
 byte antileft = map(dist3, 6, 210, 125, 120); //force to right
 byte antiright = map(dist4, 6, 210, 55, 60); //force to left
 byte antidown = map(dist6, 1, 100, 125, 120);  //force to up
 byte antiUP = map(dist5, 2, 210, 55, 60);    //force to down   
int val = pulseIn(A0,HIGH);
if(val < 1500){

//    For Pitch..............
if(dist1<a && dist1>0 && (dist2>b || dist2==0)){
  out2.write(antifront);
  b=210;
}
else if((dist1>=a || dist1==0) && (dist2>=b || dist2==0)){
  out2.write(pitch);
}
else if(dist2<b && dist2>0 && (dist1>a || dist1==0)){
  out2.write(antiback);
  a=210;
}
else if(dist1<a && dist2<b && dist1>0 && dist2>0){
  out2.write(92);
  c=210;
  d=210;
  e=210;
    if(dist3>c || dist3==0){
    out1.write(55);
   }
   else if(dist4>d || dist4==0){
    out1.write(125);
   }
   else if(dist5>e ||dist5==0){
    out3.write(110);
   }
   else if(dist6>50 ||dist6==0){
    out3.write(60);
   }
}


//    For Roll..............
if(dist3<c && dist3>0 && (dist4>d || dist4==0)){
  out1.write(antileft);
  d=210;
}
else if(dist3>=c || dist3==0 && dist4>=d || dist4==0){
  out1.write(roll);
}
else if(dist4<d && dist4>0 && (dist3>c || dist3==0)){
  out1.write(antiright);
  c=210;
}
else if(dist3<c && dist4<d && dist3>0 && dist4>0){
  out1.write(92);
  a=210;
  b=210;
  e=210;  
  if(dist1>a || dist1 ==0){
    out2.write(125);
  }
  else if(dist2>b || dist2 ==0){
    out2.write(55);
  }
  else if(dist5>e || dist5 ==0){
    out3.write(110);
  }
  else if(dist6>50 || dist6 ==0){
    out3.write(60);
  }
}



//    For Throttle..............
if(dist5<e && dist5>0 && (dist6>50 || dist6==0)){
  out3.write(antiUP);
}
else if(dist5>=e || dist5==0 && dist6>=50 || dist6==0){
  out3.write(throttle);
}
else if(dist6<50 && dist6>0 && (dist5>e || dist5==0)){
  out3.write(antidown);
}
else if(dist5<e && dist6<50 && dist5>0 && dist6>0){
  out3.write(88);
  a=210;
  b=210;
  c=210;  
  d=210;
  if(dist1>a || dist1==0){
    out2.write(125);
  }
  else if(dist2>b || dist2==0){
    out2.write(55);
  }
  else if(dist3>c|| dist3==0){
    out1.write(125);
  }
  else if(dist4>d || dist4==0){
    out1.write(55);
  }
}

if(input_THROTTLE <= 1400){
  out3.write(throttle);
}

int FB = (dist1<a && dist2<b && dist1>0 && dist2>0); 
int LR = (dist3<c && dist4<d && dist3>0 && dist4>0); 

if(FB && LR){
  out1.write(92);
  out2.write(92);
  if(input_THROTTLE <= 1400){
  out3.write(throttle);
  }
  else if(dist5>e || dist5==0){
    out3.write(110);
  }
  else if(dist6>50){
  out3.write(60);
  }   
else{out3.write(88);}
}



}

else if(val >= 1500){
  out1.write(roll);
  out2.write(pitch);
  out3.write(throttle);  
}



int f_clear = (dist1>a || dist1==0);  //front no obstacle
int b_clear = (dist2>b || dist2==0);   //back no obstacle
int l_clear = (dist3>c || dist3==0);   //left no obstacle
int r_clear = (dist4>d || dist4==0);  //right no obstacle
int up_clear = (dist5>e || dist5==0);    //up no obstacle
int down_clear = (dist6>50 || dist6==0); //down no obstacle

if(f_clear && b_clear && l_clear && r_clear && up_clear && down_clear){
   if(val<1500){
     leds[0] = CRGB(0, 255, 0);
     FastLED.show();
  out1.write(roll);
  out2.write(pitch);
  out3.write(throttle);  
   }
   else if(val>=1500){
     leds[0] = CRGB(235, 255, 200);
     FastLED.show();
   }
}

else if(dist1<a && dist1>0 || dist2<b && dist2>0 || dist3<c && dist3>0 || dist4<d && dist4>0 || dist5<e &&& dist5>0 || dist6<50 && dist6>0){
   if(val<1500){ 
     leds[0] = CRGB(255, 0, 0); 
     FastLED.show(); 
   }
   else if(val>=1500){
     leds[0] = CRGB(235, 255, 200);
     FastLED.show();
   }
}

} //Here the main loop ends


//   The Below Codes Reads the PWM value of Receiver
ISR(PCINT0_vect){
current_count = micros();
if(PINB & B00000001){                              
    if(last_CH1_state == 0){                         
      last_CH1_state = 1;                           
      counter_1 = current_count;                     
  }
}
else if(last_CH1_state == 1){                        
last_CH1_state = 0;                             
input_ROLL = current_count - counter_1;  
}
if(PINB & B00000010 ){                                                                          
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
  }
}
else if(last_CH2_state == 1){                                           
last_CH2_state = 0;                                                     
input_PITCH = current_count - counter_2;                             
} 
if(PINB & B00000100 ){                                                                   
   if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
  }
}
else if(last_CH3_state == 1){                                             
last_CH3_state = 0;                                                    
input_THROTTLE = current_count - counter_3;                            
} 
}
