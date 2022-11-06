
/*
  MKMR Arduino Code 

  modified 30 Oct 2022
  by Melih KORKMAZ

*/

int count = 0;

/*  Arduino – ROS headers   */
#include <ros.h>
#include <mkmr_msgs/ArduinoOutput.h>
#include <mkmr_msgs/ArduinoInput.h>
#include <PID_v1.h>


/* One-second interval for measurements  */
int interval = 1000;
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;




/*  An array of pin numbers to which driver pins are attached 
 *  ENA IN1 IN2 IN3 IN4 ENB
*/
int DRV1[] = {4, 26, 27, 28, 29, 5}; 
int DRV2[] = {6, 30, 31, 32, 33, 7}; 
int DRVpinCount = 6;           // the number of pins



/*  An array of pin numbers to which encoders pins are attached 
 *  ENCA - Encoder output to Arduino Interrupt pin. Tracks the pulse count
 *  ENCB - // Other encoder output to Arduino to keep track of wheel direction
 *  Third Value -1 or 1 -- encoder direct or reverse 
 *  CLOCKWISE
 *  DC1 FRONT LEFT - DC2 FRONT RIGHT - DC3 BACK RIGHT - DC4 BACK LEFT 
*/
#define ENC_COUNT_REV 330 // 330 rpm 
int ENC[4][3] = {{2, 46,-1},{3, 47,1},{18, 48,1},{20, 49,-1}};
int ENCCount = 4;           // the number of encoders


volatile long ENCPulseCount[]= {0,0,0,0};
double ENCRPMActual[]= {0,0,0,0};
double ENCRPMTarget[]= {0,0,0,0};
double ENCRPMActualABS[]= {0,0,0,0};
double ENCRPMTargetABS[]= {0,0,0,0};


/* Creating a Nodehandle object   */
ros::NodeHandle nh;
mkmr_msgs::ArduinoOutput pub_data;
ros::Publisher obj_pub("arduino_output", &pub_data);
mkmr_msgs::ArduinoOutput pub_data2;
ros::Publisher obj_pub2("arduino_test", &pub_data2);


/*Creating a callback for the topic arduino_input, whenever a value come through this topic, this callback will execute*/
void messageCb( const mkmr_msgs::ArduinoInput& msg){

  
  ENCRPMTarget[0] = msg.velocity_1;
  ENCRPMTarget[1] = msg.velocity_2;
  ENCRPMTarget[2] = msg.velocity_3;
  ENCRPMTarget[3] = msg.velocity_4;

  
  ENCRPMTargetABS[0] = abs(msg.velocity_1);
  ENCRPMTargetABS[1] = abs(msg.velocity_2);
  ENCRPMTargetABS[2]  = abs(msg.velocity_3);
  ENCRPMTargetABS[3] = abs(msg.velocity_4);
  
}

/* Creating a subscriber with a name arduino_input, and its callback  */
ros::Subscriber<mkmr_msgs::ArduinoInput> obj_sub("arduino_input", messageCb );


/* PID*/
double PIDOutput[]= {0,0,0,0};
//Specify the links and initial tuning parameters
double Kp=0.3; // PID Proportional control Gain   //0.3
double Ki=1; // PID Integral control gain     // 0.2 
double Kd=0; // PID Derivitave control gain    //0.1



//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

PID PID1(&ENCRPMActualABS[0], &PIDOutput[0], &ENCRPMTargetABS[0], Kp, Ki, Kd, DIRECT);
PID PID2(&ENCRPMActualABS[1], &PIDOutput[1], &ENCRPMTargetABS[1], Kp, Ki, Kd, DIRECT);
PID PID3(&ENCRPMActualABS[2], &PIDOutput[2], &ENCRPMTargetABS[2], Kp, Ki, Kd, DIRECT);
PID PID4(&ENCRPMActualABS[3], &PIDOutput[3], &ENCRPMTargetABS[3], Kp, Ki, Kd, DIRECT);


PID allPIDS[4] = {PID1,PID2,PID3,PID4};
int PIDCount = 4;           // the number of PID objects


/* Voltage Measurement */
int analogInput = 0;
float vout = 0.0;
float vin = 0.0;
float R1 = 100000.0; // 100K ohm direnç
float R2 = 10000.0; // 10K ohm direnç
int measure_value = 0; 

void setup() {
  /* Initializing ROS node and subscriber publisher object  */
  nh.initNode();
  nh.subscribe(obj_sub);
  nh.advertise(obj_pub);
  nh.advertise(obj_pub2);

  
/*  the array elements are numbered from 0 to (DRVpinCount - 1)
 *  use a for loop to initialize each pin as an output
*/
  for (int thisPin = 0; thisPin < DRVpinCount; thisPin++) {
    pinMode(DRV1[thisPin], OUTPUT);
    pinMode(DRV2[thisPin], OUTPUT);
  }
  
/*  the array elements are numbered from 0 to (ENCCount - 1)
 *  use a for loop to initialize each pin as an input intterrupt & input
*/  
  for (int thisId = 0; thisId < ENCCount; thisId++) {
    pinMode(ENC[thisId][0], INPUT_PULLUP);
    pinMode(ENC[thisId][1], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ENC[0][0]), ENC1_PULSE_FUNC, RISING); // Every time the pin goes high, this is a pulse  
  attachInterrupt(digitalPinToInterrupt(ENC[1][0]), ENC2_PULSE_FUNC, RISING); // Every time the pin goes high, this is a pulse  
  attachInterrupt(digitalPinToInterrupt(ENC[2][0]), ENC3_PULSE_FUNC, RISING); // Every time the pin goes high, this is a pulse  
  attachInterrupt(digitalPinToInterrupt(ENC[3][0]), ENC4_PULSE_FUNC, RISING); // Every time the pin goes high, this is a pulse  



  for (int thisId = 0; thisId < PIDCount; thisId++) {
    allPIDS[thisId].SetMode(AUTOMATIC);
    //allPIDS[thisId].SetOutputLimits(70, 170) ;
    allPIDS[thisId].SetSampleTime(1);
  }
  
  allPIDS[0].SetOutputLimits(45, 165); // FL
  allPIDS[1].SetOutputLimits(30, 150); // FR
  allPIDS[2].SetOutputLimits(40, 160); // BR
  allPIDS[3].SetOutputLimits(35, 155); // BL

  
   pinMode(analogInput, INPUT);
}

void loop() {

  for (int thisId = 0; thisId < PIDCount; thisId++) {
    allPIDS[thisId].Compute();
  }


  //analogWrite(DRV1[0], PIDOutput[0]);
  // DRV1
  analogWrite(DRV1[0], PIDOutput[0]);
  analogWrite(DRV1[5], PIDOutput[1]); 
  // DRV2
  analogWrite(DRV2[0], PIDOutput[2]);
  analogWrite(DRV2[5], PIDOutput[3]); 

/*

  // DRV1
  analogWrite(DRV1[0], ENCRPMTargetABS[0]);
  analogWrite(DRV1[5], ENCRPMTargetABS[1]); 
  // DRV2
  analogWrite(DRV2[0], ENCRPMTargetABS[2]);
  analogWrite(DRV2[5], ENCRPMTargetABS[3]); 
*/

  
  for (int thisId = 0; thisId < PIDCount; thisId++) {

    if(ENCRPMTarget[thisId]>0){
      FORWARD(thisId);
      }
  
    if(ENCRPMTarget[thisId]<0){
      BACKWARD(thisId);
      }

    if(ENCRPMTarget[thisId]==0){
      STOP(thisId);
      }
   
  }

   

   
  // Record the time
  currentMillis = millis();

  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;




   for (int Id = 0; Id < ENCCount; Id++) {
      // Calculate revolutions per minute
      ENCRPMActual[Id] = (float)(ENCPulseCount[Id] * 60 / ENC_COUNT_REV);
      ENCRPMActualABS[Id] = abs(ENCRPMActual[Id]);
      Serial.print("  Pulses: ");
      Serial.print(ENCPulseCount[Id]);
      Serial.print("      RPM: ");
      Serial.print(ENCRPMActual[Id]);
      Serial.println();
   
      ENCPulseCount[Id] = 0;
    }

  }

  pub_data.FL_RPM  = ENCRPMActual[0]; 
  pub_data.FL_VEL =  PIDOutput[0];
 
  

  pub_data.FR_RPM  = ENCRPMActual[1]; 
  pub_data.FR_VEL =  PIDOutput[1];
  
  pub_data.BR_RPM  = ENCRPMActual[2]; 
  pub_data.BR_VEL =  PIDOutput[2];

  pub_data.BL_RPM  = ENCRPMActual[3]; 
  pub_data.BL_VEL =  PIDOutput[3];



  /*Publishing data*/
  obj_pub.publish(&pub_data);



/* Voltage Measurement */
   measure_value = analogRead(analogInput);
   vout = (measure_value * 5.0) / 1024.0; 
   vin = vout / (R2/(R1+R2)); 
   if (vin<0.09) {
   vin=0.0; 
    } 

          Serial.print(measure_value);
          
          Serial.print("  ** ");
          
          Serial.print(vout);

                 
          Serial.print("  ** ");
          
          Serial.print(vin);
          
      Serial.println();

  pub_data2.FL_RPM  = ENCRPMActualABS[0]; 
  pub_data2.FL_VEL =  ENCRPMTargetABS[0];
  pub_data2.FL_VELDEG = PIDOutput[0]; 

  pub_data2.BL_RPM  = vin; 
    
  pub_data2.BR_RPM  = ENCRPMActualABS[1]; 
  pub_data2.BR_VEL =  ENCRPMTargetABS[1];
  pub_data2.BR_VELDEG = PIDOutput[1]; 
  
  /*Publishing data*/
  //obj_pub2.publish(&pub_data2);


  
  /*Spining the node each times to listen from the topic*/
  nh.spinOnce();
  delay(1);

   

   
}



/* pulse measurement function for the first encoder */
void ENC1_PULSE_FUNC() {
  int Id =0;
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC[Id][1]);
 
  if(val == LOW) {
    ENCPulseCount[Id]=ENCPulseCount[Id]+ENC[Id][2];
  }
  else {
    ENCPulseCount[Id]=ENCPulseCount[Id]-ENC[Id][2];
  }
}

/* pulse measurement function for the second encoder */
void ENC2_PULSE_FUNC() {
  int Id =1;
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC[Id][1]);
 
  if(val == LOW) {
    ENCPulseCount[Id]=ENCPulseCount[Id]+ENC[Id][2];
  }
  else {
    ENCPulseCount[Id]=ENCPulseCount[Id]-ENC[Id][2];
  }
}

/* pulse measurement function for the third encoder */
void ENC3_PULSE_FUNC() {
  int Id =2;
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC[Id][1]);
 
  if(val == LOW) {
    ENCPulseCount[Id]=ENCPulseCount[Id]+ENC[Id][2];
  }
  else {
    ENCPulseCount[Id]=ENCPulseCount[Id]-ENC[Id][2];
  }
}

/* pulse measurement function for the fourth encoder */
void ENC4_PULSE_FUNC() {
  int Id =3;
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC[Id][1]);
 
  if(val == LOW) {
    ENCPulseCount[Id]=ENCPulseCount[Id]+ENC[Id][2];
  }
  else {
    ENCPulseCount[Id]=ENCPulseCount[Id]-ENC[Id][2];
  }
}



/*  */
void FORWARD(int motor_id) {
  if (motor_id==0) {
    digitalWrite(DRV1[1], LOW);
    digitalWrite(DRV1[2], HIGH);
  }
  if (motor_id==1) {
    digitalWrite(DRV1[3], HIGH);
    digitalWrite(DRV1[4], LOW);
  }
  if (motor_id==2) {
    digitalWrite(DRV2[1], HIGH);
    digitalWrite(DRV2[2], LOW);
  }
  if (motor_id==3) {
    digitalWrite(DRV2[3], LOW);
    digitalWrite(DRV2[4], HIGH);
  }  
}



/*  */
void BACKWARD(int motor_id) {
  if (motor_id==0) {
    digitalWrite(DRV1[1], HIGH);
    digitalWrite(DRV1[2], LOW);
  }
  if (motor_id==1) {
    digitalWrite(DRV1[3], LOW);
    digitalWrite(DRV1[4], HIGH);
  }
  if (motor_id==2) {
    digitalWrite(DRV2[1], LOW);
    digitalWrite(DRV2[2], HIGH);
  }
  if (motor_id==3) {
    digitalWrite(DRV2[3], HIGH);
    digitalWrite(DRV2[4], LOW);
  }   
}



  /*  */
void STOP(int motor_id) {
  if (motor_id==0) {
    digitalWrite(DRV1[1], LOW);
    digitalWrite(DRV1[2], LOW);
  }
  if (motor_id==1) {
    digitalWrite(DRV1[3], LOW);
    digitalWrite(DRV1[4], LOW);
  } 
  if (motor_id==2) {
    digitalWrite(DRV2[1], LOW);
    digitalWrite(DRV2[2], LOW);
  }
  if (motor_id==3) {
    digitalWrite(DRV2[3], LOW);
    digitalWrite(DRV2[4], LOW);
  }     
}
