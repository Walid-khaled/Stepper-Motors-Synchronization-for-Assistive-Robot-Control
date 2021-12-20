/*
 * This code was developed by Walid Khaled - Nile University graduating student 2021 
 * It is used to run 6DOF Manipulator with gripper attached.
 * You could run from Arduino Serial Monitor; just type the angles and coordinated time e.. -90 0 90 10 20 30 5
 * Also, it could be run from MATLAB e.. [-90 0 90 10 20 30 5], [-90, 0, 90, 10, 20, 30, 5]
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <MultiStepper.h>
#include <AccelStepper.h>

//Stepper Motors Pin Configuration
AccelStepper stepper1 (AccelStepper::DRIVER, 2, 3);     //PULSE Pin, Direction Pin , Stepper 1
AccelStepper stepper2 (AccelStepper::DRIVER, 4, 5);     //PULSE Pin, Direction Pin , Stepper 2
AccelStepper stepper3 (AccelStepper::DRIVER, 6, 7);     //PULSE Pin, Direction Pin , Stepper 3
AccelStepper stepper4 (AccelStepper::DRIVER, 8, 9);     //PULSE Pin, Direction Pin , Stepper 4
AccelStepper stepper5 (AccelStepper::DRIVER, 10, 11);   //PULSE Pin, Direction Pin , Stepper 5
AccelStepper stepper6 (AccelStepper::DRIVER, 12, 13);   //PULSE Pin, Direction Pin , Stepper 6

MultiStepper steppers; // upt to 10 steppers we can use MultiStepper

//Driver Mode
int DPulsePerRev[] = {400, 400, 400, 400, 800, 400}; //MicroStepping mode 400 is half-step, 800 is quarter-step

//Set Gear Ratios for Joints
float GearRatio[] = {10, 50, 50, (3969/289), 1, (3585/187)}; //(3969/289)=13.73 and (3585/187)=19.17

//Set Belt Ratios for Joints
float BeltRatio[] = {4.2, 1, 1, 3.09, 8.8889, 1};

//Set Positions for Motors
long positions[6]; //Array of desired stepper positions
long prev_positions[6]; //Array of previous stepper positions

//Set Speeds for Motors
float n = 120;
float MotorSpeeds[] = {((n/60)*DPulsePerRev[0]), ((n/60)*DPulsePerRev[1]), ((n/60)*DPulsePerRev[2]), ((n/60)*DPulsePerRev[3]), ((n/60)*DPulsePerRev[4]), ((n/60)*DPulsePerRev[5])};
float TimeTaken;
float Actual_MotorSpeeds[6];
float Actual_n_RPM[6];

//Set Limit Switches 
long LSwitchS[6];
int Lswitch1 = 14;
int Lswitch2 = 15;
int Lswitch3 = 16;
int Lswitch4 = 17;
int Lswitch5 = 18;
int Lswitch6 = 19;

//Set Inputs 
String MATLABdata;
char *arr_char_strings[7];
char *ptr = NULL;
byte index;
String arr_strings[7];
float arr_input[7];

//Set Gripper
#include <Servo.h>
Servo myservo;  
int pos = 0;    

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  //Add steppers to the MultiStepper Object
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);

  myservo.attach(A6);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if (Serial.available() > 0){
    MATLABdata=Serial.readString();                      // read data as string.
    //Serial.println("Data Received");
    
    if (MATLABdata == "1"){
      Serial.println("Calibration ....");
      calibration();
    }
    
    else if (MATLABdata == "2"){
      Serial.println("Zero Postions ....");
      zeros();
    }
    
    else if (MATLABdata == "3"){
      Serial.println("Homing ....");
      homing();
    }

    else if (MATLABdata == "4"){
      Serial.println("CG End Position");
      cg_end_position();
    }

    else if (MATLABdata == "5"){
      Serial.println("Gripper Open");
      open_gripper();
    }

    else if (MATLABdata == "6"){
      Serial.println("Gripper Close");
      close_gripper();
    }

    else{
      char buf[MATLABdata.length()+1];                   // length (with one extra character for the null terminator, a null character (ASCII code 0) to tell where the end of a string is).
      MATLABdata.toCharArray(buf,MATLABdata.length()+1); // Copy it over in buffer, sizeof (buf) = MATLABdata.length()+1.
  
      index = 0;                                         // define index.
      ptr = strtok(buf, " ");                            // tokenizing of a string using the strtok() function that returns a variable of type char separated based on a delimiter (" ").
      while(ptr != NULL){                                // iterate the list until it is empty.
        arr_char_strings[index] = ptr;                   // append in array of type char.
        index++;                                         // increment index by 1.
        ptr = strtok(NULL, " ");                         // takes a list of delimiters.
      }
      
      for (int r = 0; r < index; r++){
        arr_strings[r]=String(arr_char_strings[r]);      // convert from char to string type.
        arr_input[r]=arr_strings[r].toFloat();           // convert from string to float type.
      }
      moving();                                          // calling moving function.
    }                                           
  }    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calibration() {
  //Configure each Stepper Motor
  stepper1.setMaxSpeed(MotorSpeeds[0]);
  stepper2.setMaxSpeed(MotorSpeeds[1]);
  stepper3.setMaxSpeed(MotorSpeeds[2]);
  stepper4.setMaxSpeed(MotorSpeeds[3]);
  stepper5.setMaxSpeed(MotorSpeeds[4]);
  stepper6.setMaxSpeed(MotorSpeeds[5]);

  //Set Smallest Increment for each Motor, Note Motors Found to Rotate in the Direction of Limit Switches in the Following Sign Convention
  positions[0] = 1;
  positions[1] = 1;
  positions[2] = 0;
  positions[3] = -1;
  positions[4] = 1;
  positions[5] = -1;

  while (true) {
    //Take the Readings of Limit Switches Continuously for Motors except Motor 3
    LSwitchS[0] = digitalRead(Lswitch1);
    LSwitchS[1] = digitalRead(Lswitch2);
    LSwitchS[2] = digitalRead(Lswitch3);
    LSwitchS[3] = digitalRead(Lswitch4);
    LSwitchS[4] = digitalRead(Lswitch5);
    LSwitchS[5] = digitalRead(Lswitch6);
    //If One of Motor Reaches the Limit Switch, Set the Position to Zero
    for (int i = 0; i < 6; i++) {
      if (LSwitchS[i] == 1) {
        positions[i] = 0;
      }
    }
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

    //Temporarily Zeros after each Step
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
    stepper5.setCurrentPosition(0);
    stepper6.setCurrentPosition(0);

    //Increment by 1
    positions[0] = positions[0] + 1;
    positions[1] = positions[1] + 1;
    positions[2] = positions[2] - 0;
    positions[3] = positions[3] - 1;
    positions[4] = positions[4] + 1;
    positions[5] = positions[5] - 1;

    //Limit Switches Zeros
    if ((LSwitchS[0] == HIGH) && (LSwitchS[1] == HIGH) && (LSwitchS[3] == HIGH) && (LSwitchS[4] == HIGH) && (LSwitchS[5] == HIGH)){
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);
      stepper5.setCurrentPosition(0);
      stepper6.setCurrentPosition(0);
      break;
    }
  }

  delay(2000);

  //Moving to Actual Zeros
  positions[0] = -7733.33;//-7933.33
  positions[1] = -2333.33;
  positions[2] = -3000;
  positions[3] = 7000;//7732.912
  positions[4] = -1975.31;//-2034.57;
  positions[5] = 3550;//3408
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper5.setCurrentPosition(0);
  stepper6.setCurrentPosition(0);
  
  //Motor 3 Calibration and Zero 
  positions[2] = -1;
  while ((digitalRead(Lswitch3) == LOW)) {
    positions[0] = 0; positions[1] = 0; positions[3] = 0; positions[4] = 0; positions[5] = 0;
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    if (digitalRead(Lswitch3) == HIGH) {
      stepper3.setCurrentPosition(0);
      break;
    }
    positions[2] = positions[2] - 1;
  }
  positions[0] = 0; positions[1] = 0; positions[2] = 7777.77; positions[3] = 0; positions[4] = 0; positions[5] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  stepper3.setCurrentPosition(0);

  //J2 Zero 
  positions[0] = 0; positions[1] = -5000; positions[2] = 0; positions[3] = 0; positions[4] = 0; positions[5] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  stepper2.setCurrentPosition(0);

  //Update Previous Positions
  prev_positions[0] = 0; prev_positions[1] = 0; prev_positions[2] = 0; prev_positions[3] = 0; prev_positions[4] = 0; prev_positions[5] = 0; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void zeros() {
  //Configure each Stepper Motor
  stepper1.setMaxSpeed(MotorSpeeds[0]);
  stepper2.setMaxSpeed(MotorSpeeds[1]);
  stepper3.setMaxSpeed(MotorSpeeds[2]);
  stepper4.setMaxSpeed(MotorSpeeds[3]);
  stepper5.setMaxSpeed(MotorSpeeds[4]);
  stepper6.setMaxSpeed(MotorSpeeds[5]);
  for (int y = 0; y < 6; y++) {
    positions[y]=0;
    //Update Previous Positions
    prev_positions[y]=0;;
  } 
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void homing() {
  //Configure each Stepper Motor
  stepper1.setMaxSpeed(MotorSpeeds[0]);
  stepper2.setMaxSpeed(MotorSpeeds[1]);
  stepper3.setMaxSpeed(MotorSpeeds[2]);
  stepper4.setMaxSpeed(MotorSpeeds[3]);
  stepper5.setMaxSpeed(MotorSpeeds[4]);
  stepper6.setMaxSpeed(MotorSpeeds[5]);
  positions[0] = 0; positions[1] = 5000; positions[2] = -5000; positions[3] = 0; positions[4] = 0; positions[5] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); 
  
  //Update Previous Positions
  prev_positions[0] = 0; prev_positions[1] = -5000; prev_positions[2] = 5000; prev_positions[3] = 0; prev_positions[4] = 0; prev_positions[5] = 0; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cg_end_position() {
  //Configure each Stepper Motor
  stepper1.setMaxSpeed(MotorSpeeds[0]);
  stepper2.setMaxSpeed(MotorSpeeds[1]);
  stepper3.setMaxSpeed(MotorSpeeds[2]);
  stepper4.setMaxSpeed(MotorSpeeds[3]);
  stepper5.setMaxSpeed(MotorSpeeds[4]);
  stepper6.setMaxSpeed(MotorSpeeds[5]);
  positions[0] = 0; positions[1] = 5000; positions[2] = 0; positions[3] = 0; positions[4] = 0; positions[5] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  
  //Update Previous Positions
  prev_positions[0] = 0; prev_positions[1] = -5000; prev_positions[2] = 0; prev_positions[3] = 0; prev_positions[4] = 0; prev_positions[5] = 0; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moving() {

  if (arr_input[0]<-170 || arr_input[0]>170 || arr_input[1]<-132 || arr_input[1]>0 || arr_input[2]<0 || arr_input[2]>140 ||
  arr_input[3]<-164 || arr_input[3]>164 || arr_input[4]<-103 || arr_input[4]>103 || arr_input[5]<-160 || arr_input[5]>160){
    Serial.println("Wrong Input: Please Check the Angles Limitations J1 Limit [-170:170], J2 Limit [-132:0], J3 Limit [0:140], J4 Limit [-164:164], J5 Limit [-103:103], J6 Limit [-160:160]");
  }

  if (arr_input[0]>=-170 && arr_input[0]<=170 && arr_input[1]>=-132 && arr_input[1]<=0 && arr_input[2]>=0 && arr_input[2]<=140 &&
  arr_input[3]>=-164 && arr_input[3]<=164 && arr_input[4]>=-103 && arr_input[4]<=103 && arr_input[5]>=-160 && arr_input[5]<=160){
    //Note Positions of Motors 2,3,4,5 have to be multiplied by -1 in order to match Anin Configuration (Angles Limits and Directions)
    positions[0] = ((arr_input[0] / 360.0) * (DPulsePerRev[0]) * (GearRatio[0]) * (BeltRatio[0]));
    positions[1] = (-(arr_input[1] / 360.0) * (DPulsePerRev[1]) * (GearRatio[1]) * (BeltRatio[1]));
    positions[2] = (-(arr_input[2] / 360.0) * (DPulsePerRev[2]) * (GearRatio[2]) * (BeltRatio[2]));
    positions[3] = (-(arr_input[3] / 360.0) * (DPulsePerRev[3]) * (GearRatio[3]) * (BeltRatio[3]));
    positions[4] = (-(arr_input[4] / 360.0) * (DPulsePerRev[4]) * (GearRatio[4]) * (BeltRatio[4]));
    positions[5] = ((arr_input[5] / 360.0) * (DPulsePerRev[5]) * (GearRatio[5]) * (BeltRatio[5]));
  
    TimeTaken=arr_input[6];
    //Serial.println("///////////////////Actual_MotorSpeeds and Actual_n_RPM for all Motors///////////////////");
    for (int k = 0; k < 6; k++) {
      if ((positions[k]-prev_positions[k]) >=0) {
        Actual_MotorSpeeds[k]= (positions[k]-prev_positions[k])/ TimeTaken; 
        Actual_n_RPM[k] = ((60 * (positions[k]-prev_positions[k])) / (TimeTaken*DPulsePerRev[k]));
      }
      if ((positions[k]-prev_positions[k]) <0) {
        Actual_MotorSpeeds[k]= (prev_positions[k]-positions[k])/ TimeTaken;
        Actual_n_RPM[k] = ((60 * (prev_positions[k]-positions[k])) / (TimeTaken*DPulsePerRev[k]));
      }
      //Serial.print("Actual_MotorSpeed[");Serial.print(k);Serial.print("] = ");Serial.println(Actual_MotorSpeeds[k]);
      //Serial.print("Actual_n_RPM[");Serial.print(k);Serial.print("] = ");Serial.println(Actual_n_RPM[k]);
    }
    
    stepper1.setMaxSpeed(Actual_MotorSpeeds[0]);
    stepper2.setMaxSpeed(Actual_MotorSpeeds[1]);
    stepper3.setMaxSpeed(Actual_MotorSpeeds[2]);
    stepper4.setMaxSpeed(Actual_MotorSpeeds[3]);
    stepper5.setMaxSpeed(Actual_MotorSpeeds[4]);
    stepper6.setMaxSpeed(Actual_MotorSpeeds[5]);
  
    steppers.moveTo(positions);
    steppers.runSpeedToPosition(); 
    Serial.println("MOVING COMPLETED!");

    //Update Previous Positions
    for (int s = 0; s < 6; s++) {
      prev_positions[s]=positions[s];
      //Serial.print("prev_positions[");Serial.print(s);Serial.print("] = ");Serial.println(prev_positions[s]);
    } 
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void open_gripper() {
  if (pos!=-1){
    for (pos = 50; pos >= 0; pos -= 1){ 
      myservo.write(pos);              
      delay(10);                       
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void close_gripper() {
  if (pos!=101 && pos!=0){
    for (pos = 0; pos <= 100; pos += 1){
      myservo.write(pos);              
      delay(5);  
    }                        
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

