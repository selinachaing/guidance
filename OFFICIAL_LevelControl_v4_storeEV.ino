
#include <Encoder.h>
#include <Servo.h>

//FSR////////////////////////////////////////////////////////////////////////////////////////////////////////

const int FSR_PIN[4] = {A0, A1, A2, A3}; 
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 3230.0; // Measured resistance of 3.3k resistor
int fsrADC[4] = {0, 0, 0, 0}; //=analog(FSR_PIN[i])

float fsrForce[4] = {0, 0, 0, 0};     //"force" they read in readFSR function 
float fsrForceRecord_small[4] = {50, 50, 50, 50};
float fsrForceRecord_big[4] = {300, 300, 300, 300};
float force_threshold[4] = {0,0,0,0};  //default threshold = 0, later will assaign value from ForceRecord in readFSR fuction 

int fsrLevel[3] = {0,1,2}; 
//0--> no poke; 1--> small force ; 2--> big force

//global
int level_global = 0; //level is being read
int rotate_global = 0;
int inst_global = 0; //inst is being entered ? 0--> no / exec done. 1--> processing

//Enc value
int bigForce_EncV[4] = {0, 0, 0, 0}; // in level=1 --> assaign oldPosition to here
int smallForce_EncV[4] = {0, 0, 0, 0}; // in level=2 --> assaign oldPosition to here

//first time or not 
int flag_first[4] = {0, 0, 0, 0};//0,1,2 

//flag////////////////////////////////////////////////////////////////////////////////////////////////////////

int flag_rotateCheck[2] = {0,0}; ///check if pks is at the right location 0-->not arrive 1--> arrive
int flag_pokeCheck[4]= {0, 0, 0, 0};

float pokeForce[4] = {0, 0, 0, 0}; //poke圈數

int pokeStateList[3] = {0,1,2}; //0-->stop; 1-->poke; 2-->back
int pokeState[4] = {0, 0, 0, 0}; 

int instStateList[4] = {0,1,2,3}; //t,r,s,x
int instState = 3;

//servo////////////////////////////////////////////////////////////////////////////////////////////////////////

Servo servo_Tilt[4];
int servo_Tilt_Pin[4] = {29, 33, 39, 43};
int servo_Tilt_Angle = 90;

//dc 380////////////////////////////////////////////////////////////////////////////////////////////////////////

Encoder Enc1(21, 35);
Encoder Enc2(20, 27);
Encoder Enc3(19, 31);
Encoder Enc4(18, 37);
const int pinAIN1[4] = {26, 30, 36, 40}; //Direction
const int pinAIN2[4] = {28, 32, 38, 42}; //Direction
const int pinPWMA[4] = {13, 11, 10, 6}; //Voltage  2~13,44~46
const int pinSTBY[2] = {12, 7};

long oldPosition[4] = { -999, -999, -999, -999};
long nowPosition[4] = {0, 0, 0, 0};
long newPosition[4] = {0, 0, 0, 0};
const unsigned short acceptableCountError = 5;
const unsigned short activationCountError = 1;
bool motorActive[4] = {0, 0, 0, 0};
long targetCount[4] = {0, 0, 0, 0};
long integral[4] = {0, 0, 0, 0};

long error_array[4] = {0,0,0,0};
long voltageFeed[4] = {0, 0, 0, 0};
const bool turnDirection = true;

const int motor[4] = {0, 1, 2, 3};
int encoderValue[1] = {4548};   //379*12
int maximalVoltage = 255;

//dc 298/////////////////////////////////////////////////////////////////////////////////////////////////////

Encoder Enc1_rotate(2, 41);
Encoder Enc2_rotate(3, 47);
const int pinAIN1_rotate[2] = {46, 50}; //Direction
const int pinAIN2_rotate[2] = {48, 52}; //Direction
const int pinPWMA_rotate[2] = {4, 5}; //Voltage  2~13,44~46
const int pinSTBY_rotate[1] = {14};

long oldPosition_rotate[2] = { -999, -999};
long nowPosition_rotate[2] = {0, 0};
long newPosition_rotate[2] = {0, 0};
const unsigned short acceptableCountError_rotate = 10;
const unsigned short activationCountError_rotate = 1;
bool motorActive_rotate[2] = {0, 0};
long targetCount_rotate[2] = {0, 0};
long integral_rotate[2] = {0, 0};
long error_array_rotate[2] = {0,0};

const int KiActivationErrorRange = 100;
const int KiRange2 = 10;
const float KiRange2Factor = 2;
const int KdActivationErrorRange = 100;
const float Kp = 2;
const float Ki = 0.00001;
const float Kd = 0;
unsigned long usPassed[2] = {0, 0};
float encoderSpeed[2] = {0, 0};
long voltageFeed_rotate[2] = {0, 0};
const bool turnDirection_rotate = true;

const int motor_rotate[2] = {0, 1};
int encoderValue_rotate[1] = {3576};   //298*12

int teeth_of_brace = 85;
int teeth_of_brace_bigger = 90;
int teeth_of_car = 28;
int full_angle = 360;

const int complementary = 180;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(FSR_PIN[i], INPUT);
    pinMode(pinPWMA[i], OUTPUT);
    pinMode(pinAIN1[i], OUTPUT);
    pinMode(pinAIN2[i], OUTPUT);
    servo_Tilt[i].attach(servo_Tilt_Pin[i]);
    servo_Tilt[i].write(servo_Tilt_Angle);
    if (i < 2) {
      pinMode(pinSTBY[i], OUTPUT);
    }
  }
  Serial.println("Guidance++ DC380 & SERVO");
  
  for (int i = 0; i < 2; i++) {
    pinMode(pinPWMA_rotate[i], OUTPUT);
    pinMode(pinAIN1_rotate[i], OUTPUT);
    pinMode(pinAIN2_rotate[i], OUTPUT);
    if (i < 1) { 
      pinMode(pinSTBY_rotate[i], OUTPUT);
    }
  }
   Serial.println("Guidance++ DC298");
   //motorDrive(motor[0], turnDirection, 255);
}

void loop() {
  
  readEncoder();
  
    if(flag_first[0] == 1 ){
      voltageFeed[0] = maximalVoltage;
      motorDrive(motor[0], turnDirection, voltageFeed[0]);////////////////////////////////////////////////////////////////////
    }
    if(flag_first[1] == 1 ){
      voltageFeed[1] = maximalVoltage;
      motorDrive(motor[1], turnDirection, voltageFeed[1]);////////////////////////////////////////////////////////////////////
    }
    if(flag_first[2] == 1 ){
      voltageFeed[2] = maximalVoltage;
      motorDrive(motor[2], turnDirection, voltageFeed[2]);////////////////////////////////////////////////////////////////////
    }
    if(flag_first[3] == 1 ){
      voltageFeed[3] = maximalVoltage;
      motorDrive(motor[3], turnDirection, voltageFeed[3]);////////////////////////////////////////////////////////////////////
    }

  //380
  for ( int i = 0; i < 4; i++) {
    if(flag_first[i] == 0 || flag_first[i] == 2){
      pokeStateControl(i);
      getVoltageFeed(i);
      updateMotor(i);
    }
  }
  
  //298
  for ( int i = 0; i < 2; i++) {
    getVoltageFeedFromPID(i);
    updateMotor_rotate(i);
  }
  
  //FSR
  for( int i = 0; i < 4; i++){ /////////////////////////////////////////////
    if(flag_first[i] == 1 && inst_global == 1){
      fsrADC[i] =  analogRead(FSR_PIN[i]);
      readFSR(i,fsrADC[i]);
    }
  }
}

void serialEvent() {
  if (Serial.available() > 0) {
    char inst;
    inst = Serial.read();
    inst_global = 1;
    Serial.println("-----------enter your inst------------");
    Serial.print("Instruction: ");
    Serial.println(inst);

    //全部都先把馬達圈數清零
    for(int i = 0; i < 4 ;i++){
        pokeForce[i] = 0;
    }

    if (inst == 't') {
      
      // t,p,120,-10,1
      instState = instStateList[0];
      
      char comma = Serial.read();
      char pks = Serial.read(); //p,q
      int angle_fromUnity = Serial.parseInt(); //rotate
      float angle_Rotate_Input;
       
      if( angle_fromUnity < 0 ){
        angle_Rotate_Input = angle_fromUnity + complementary; //+180
      }
      else if( angle_fromUnity > 0 ){
        angle_Rotate_Input = angle_fromUnity - complementary; //-180
      }
      
      //tilt
      int angle_Tilt_Input = Serial.parseInt(); 
      int angle_Tilt_Write = servo_Tilt_Angle - abs(angle_Tilt_Input);
      int level = Serial.parseInt(); //level

      if(level != level_global){ //////////////////////////////////////////////////////////////////////////////////////////////////
        for( int i = 0 ; i < 4; i++){
          flag_first[i] = 0;
        }
        level_global = level;
      }
      
      float circle_Rotate = angle_Rotate_Input/120; //大圈-->112.5 , 小圈-->120
      
      Motor_rotate_rotate(0,circle_Rotate); 
      Motor_rotate_rotate(1,circle_Rotate);

      //t,p/q,-120,-90/+90,level

      if( pks == 'p'){
        if(angle_Tilt_Input< 0){
          servo_Tilt[0].write(angle_Tilt_Write);
          servo_Tilt[1].write(90);
          levelToForce(0,level);
          levelToForce(1,0);
        }
        else if(angle_Tilt_Input > 0){
          servo_Tilt[0].write(90);
          servo_Tilt[1].write(angle_Tilt_Write);
          levelToForce(0,0);
          levelToForce(1,level);
        }
        else{ 
          servo_Tilt[0].write(angle_Tilt_Write);
          servo_Tilt[1].write(angle_Tilt_Write);
          levelToForce(0,level);
          levelToForce(1,level);
        }
      }
      else if( pks == 'q'){
        if(angle_Tilt_Input < 0){
          servo_Tilt[2].write(angle_Tilt_Write);
          servo_Tilt[3].write(90);
          levelToForce(2,level);
          levelToForce(3,0);
        }
        else if(angle_Tilt_Input > 0){
          servo_Tilt[2].write(90);
          servo_Tilt[3].write(angle_Tilt_Write);
          levelToForce(2,0);
          levelToForce(3,level);
        }
        else{
          servo_Tilt[2].write(angle_Tilt_Write);
          servo_Tilt[3].write(angle_Tilt_Write);
          levelToForce(2,level);
          levelToForce(3,level);
        }
      }
    }

    
    else if (inst == 'r') {
      instState = instStateList[1];
      
      char comma = Serial.read();
      char pks = Serial.read(); //p,q
      int angle_fromUnity = Serial.parseInt(); //rotate
      float angle_Rotate_Input;
       
      if( angle_fromUnity < 0 ){
        angle_Rotate_Input = angle_fromUnity + complementary; //+180
      }
      else if( angle_fromUnity > 0 ){
        angle_Rotate_Input = angle_fromUnity - complementary; //-180
      }
      
      Serial.print("Input:"); Serial.println(angle_fromUnity);
      Serial.print("Rotate to:"); Serial.println(angle_Rotate_Input);
      
      int tactor_Num = Serial.parseInt(); //tilt : -1 , +1
      int level = Serial.parseInt(); //level

      Serial.print("pks and tactor:"); Serial.print(pks); Serial.println(tactor_Num);
      Serial.print("Rotate to:"); Serial.println(angle_Rotate_Input);
      Serial.print("Level:"); Serial.println(level);

      if(level != level_global){ //////////////////////////////////////////////////////////////////////////////////////////////////
        for( int i = 0 ; i < 4; i++){
          flag_first[i] = 0;
        }
        level_global = level;
      }

      float circle_Rotate = angle_Rotate_Input/120; //120,112.5
      
      //encodervalue = level level目前是encoder值
      float circle_Poke = (float) level/encoderValue[0]; //4548
      
      Serial.print("Level:"); Serial.println(level);   
      Serial.print("circle_Poke:"); Serial.println(circle_Poke);

      //r,p/q,-120,-1/+1,level

      Motor_rotate_rotate(0,circle_Rotate); //p q rotate at the same time
      Motor_rotate_rotate(1,circle_Rotate);

      for(int i = 0; i < 4; i++){
        servo_Tilt[i].write(90);
      }

      if(pks == 'p'){
        if(tactor_Num < 0){ 
          // -1
          //p0(0) q1(3) down 
          levelToForce(0,level);
          levelToForce(3,level);
        }
        else if(tactor_Num > 0){ 
          // +1
          //p1(1) q0(2) down 
          levelToForce(1,level);
          levelToForce(2,level);
        }
      }
    }
    
    else if (inst == 's') {
      //s, 
      instState = instStateList[2];
      
      char comma = Serial.read();
      char pks = Serial.read(); //p,q
      int angle_fromUnity = Serial.parseInt(); //rotate
      float angle_Rotate_Input;
       
      if( angle_fromUnity < 0 ){
        angle_Rotate_Input = angle_fromUnity + complementary; //+180
      }
      else if( angle_fromUnity > 0 ){
        angle_Rotate_Input = angle_fromUnity - complementary; //-180
      }
      
      int angle_Tilt_Input = Serial.parseInt(); //tilt ???????????????????????????????
      int level = Serial.parseInt(); //level

      Serial.print("pks:"); Serial.println(pks);
      Serial.print("Rotate to:"); Serial.println(angle_Rotate_Input);
      Serial.print("Level:"); Serial.println(level);

      if(level != 0){
        for( int i = 0 ; i < 4; i++){
          flag_first[i] = 0;
        }
      }

      //tactor *4 will poke/////////////////////////////////////////////////////////////////////////////////
      levelToForce(0,level);
      levelToForce(1,level);
      levelToForce(2,level);
      levelToForce(3,level);
      
      //then rotate 要poke
//      Motor_rotate_rotate(0,circle_Rotate); //p q rotate at the same time
//      Motor_rotate_rotate(1,circle_Rotate);
      
      //rotate to the point
      
      
      //tactor*4 go back
//      Motor_rotate_rotate(0,-circle_Rotate); //p q rotate at the same time
//      Motor_rotate_rotate(1,-circle_Rotate);
      
    }    
    
    else if (inst == 'x') { 
      instState = instStateList[3];
      for(int i = 0; i < 4 ;i++){
        servo_Tilt[i].write(servo_Tilt_Angle);
        pokeForce[i] = 0;
      }
      flag_rotateCheck[0] = 0;
      flag_rotateCheck[1] = 0;
    }

    ////////////for checking///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (inst == 'm') { //380
      int motor_num = Serial.parseInt();;
      int teeth = Serial.parseInt();
      float teeth_f = teeth;
      float rotate_num = teeth_f/18;    
      
      Serial.print("Motor number: "); Serial.println(motor_num);
      Serial.print("Roll number: "); Serial.println(rotate_num);
      
      Motor_rotate(motor_num,rotate_num);
    }

    else if(inst == 'a'){ //298
      char comma = Serial.read();
      char pks = Serial.read();      
      int angle_fromUnity = Serial.parseInt(); 
      float angle_Rotate_Input = angle_fromUnity;
      
      float circle = angle_Rotate_Input/120;
      //float circle = angle_Rotate_Input/112.5;

      Serial.print("pks:"); Serial.println(pks);
      Serial.print("Rotate angle:"); Serial.println(angle_Rotate_Input);
      Serial.print("circle:"); Serial.println(circle);
      
      int motor_num;
      if( pks == 'p'){
        motor_num = 0;
        Motor_rotate_rotate(motor_num,circle);
      }
      else if( pks == 'q'){
        motor_num = 1;
        Motor_rotate_rotate(motor_num,circle);
      }
    }
    
    ////////////DEBUGGING//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if(inst == 'b'){ //380
      for( int i = 0; i < 4 ; i++ ){
         Serial.print("NOW POSITION: "); Serial.println(oldPosition[i]);
         Serial.print("ERROR: "); Serial.println(error_array[i]);
      }
    }
    else if(inst == 'd'){ //298
      for( int i = 0; i < 2 ; i++ ){
         Serial.print("NOW POSITION: "); Serial.println(oldPosition_rotate[i]);
         Serial.print("ERROR: "); Serial.println(error_array_rotate[i]);
      } 
    }
    
    else if(inst == 'i'){
      for( int i = 0; i < 4 ; i++ ){
         Serial.print("flag_first: ");Serial.print(i); Serial.print(":"); Serial.println(flag_first[i]);
        
      }
    }
    else if(inst == 'k'){
      for( int i = 0; i < 4 ; i++ ){
         Serial.print("force circle: "); Serial.println(pokeForce[i]);
      }
    }
    else if(inst == 'h'){
      motorDrive(0, turnDirection, 255);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    char icon = Serial.read();//確保不再跑一次
  }
}

//read encoder////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readEncoder()
{
  newPosition[0] = Enc1.read();
  newPosition[1] = Enc2.read();
  newPosition[2] = Enc3.read();
  newPosition[3] = Enc4.read();
  for (int i = 0; i < 4; i++) {
    if (newPosition[i] != oldPosition[i]) {
      oldPosition[i] = newPosition[i];
    }
  }
  newPosition_rotate[0] = Enc1_rotate.read();
  newPosition_rotate[1] = Enc2_rotate.read();
  for (int i = 0; i < 2; i++) {
    if (newPosition_rotate[i] != oldPosition_rotate[i]) {
      oldPosition_rotate[i] = newPosition_rotate[i];
    }
  }
}

void readFSR(int motor_number, int fsrADC ){
  
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
    
    if (fsrR <= 600){
      force = (fsrG - 0.00075) / 0.00000032639;
    }
    else{
      force =  fsrG / 0.000000642857;
    }
    
    fsrForce[motor_number] = force;
    Serial.println(fsrForce[motor_number]);
    
    //motorDrive(motor_number, turnDirection, maximalVoltage);////////////////////////////////////////////////////////////////////

    if( level_global == 1){
      force_threshold[motor_number] = fsrForceRecord_small[motor_number];//50
    }
    else if(level_global == 2){
      force_threshold[motor_number] = fsrForceRecord_big[motor_number];//100
    }

    if(fsrForce[motor_number] >= force_threshold[motor_number] ){ 
      //Serial.print("fsrForce:");Serial.print(motor_number);Serial.print(" "); Serial.println(fsrForce[motor_number]);
      
      motorStop[motor_number];
      Serial.println("STOP");

      flag_first[motor_number] = 2;
      flag_pokeCheck[motor_number] = 2;///////////////////////////////////
      pokeState[motor_number] = 1;
      inst_global = 0; //處理完畢

      if(  level_global == 1){ 
        //還沒觸發第一次紀錄>>>要記錄數值到smallForce_EncV[motor_number]  
        
        smallForce_EncV[motor_number] = oldPosition[motor_number];
        Serial.print("smallForce_EncV: "); Serial.println(smallForce_EncV[motor_number]);
      }
      else if( level_global == 2){ 
        //還沒觸發第一次紀錄>>>要記錄數值到bigForce_EncV[motor_number]    
        
        bigForce_EncV[motor_number] = oldPosition[motor_number];
        Serial.print("bigForce_EncV: "); Serial.println(bigForce_EncV[motor_number]);
      }
    }
  }
  else
  {
    // No pressure detected
  }
}

void levelToForce(int motor_number, int level){
  
  int pokeEncValue;
  float pokeCircle;
  Serial.print("Motor number:");Serial.print(motor_number); Serial.print("  Level:");Serial.println(level); 
  
  if( level == fsrLevel[1]){ //small
    
    if( flag_first[motor_number] == 0 ){   
      Serial.println("11111"); 
      flag_first[motor_number] = 1;
      
    }
    else{  
      Serial.println("11111 AGAIN"); 
      pokeEncValue = smallForce_EncV[motor_number];
      pokeCircle = (float) pokeEncValue/encoderValue[0];
      pokeForce[motor_number] = pokeCircle;
      Serial.print("motor_number:");Serial.println(motor_number);
      Serial.print("pokeCircle:");Serial.println(pokeForce[motor_number]);
    }
  }
  
  else if( level == fsrLevel[2]){  //big
    
    if( flag_first[motor_number] == 0){ 
      Serial.println("22222"); 
      flag_first[motor_number] = 1;
    }
    else{ 
      Serial.println("22222 AGAIN"); 
      pokeEncValue = bigForce_EncV[motor_number];
      pokeCircle = (float) pokeEncValue/encoderValue[0];
      pokeForce[motor_number] = pokeCircle;
    }
  }
  else{ 
    pokeForce[motor_number] = 0;
  }
}


//flag control////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pokeStateControl(int motor_num){

    int i = motor_num;
    
    //pokeState =0
    if( pokeState[i] == pokeStateList[0]){ 
      
      if( flag_rotateCheck[0] && flag_rotateCheck[1]){
        
        if(pokeForce[i] != 0){           //not first time
          Motor_rotate(i,pokeForce[i]);
          flag_pokeCheck[i] = 1;
          pokeState[i] = 1;
        }
      }
    }
    //pokeState =1
    else if( pokeState[i] == pokeStateList[1]){
      
      if( flag_pokeCheck[i] == 2 ){
        Serial.println("it's poke state 1 and pokeCheck 2");
        targetCount[i] = 0;
        pokeState[i] = 2;
      }
    }
    //pokeState =2
    else if( pokeState[i] == pokeStateList[2]){
      
      if( flag_pokeCheck[i] == 0 ){
        Serial.println("i don't move");
        pokeState[i] = 0;
        flag_rotateCheck[0] = 0;
        flag_rotateCheck[1] = 0;
      }
    }
}


//380 DC////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Motor_rotate(int motor_number, float number) {
  if ((motor_number >= 0) && (motor_number <= 4)) {
    moveToBalanceLevel(motor_number , number);
  }
}

void moveToBalanceLevel(int motor_number, float bal) {
  
  targetCount[motor_number] = bal * encoderValue[0] + oldPosition[motor_number];  //current position
  integral[motor_number] = 0;
  motorActive[motor_number] = true;
}

void getVoltageFeed(int motor_number)
{
  long error;
  error = targetCount[motor_number] - oldPosition[motor_number];
  error_array[motor_number] = error;

  if (abs(error) > activationCountError) { //error > 1
    motorActive[motor_number] = true;
    voltageFeed[motor_number] =  maximalVoltage;
    if( error < 0 ){
      voltageFeed[motor_number] =  -maximalVoltage;
    }
  }
  if (abs(error) > acceptableCountError && motorActive[motor_number]) { 
  }

  else {
    integral[motor_number] = 0;
    voltageFeed[motor_number] = 0;

    //////////////////////////////////////////////////////////////////////////////////////
    if(flag_pokeCheck[motor_number] == 1){
      flag_pokeCheck[motor_number] = 2;//if it was in poke down mode, then poke-->back
    }
    else if(flag_pokeCheck[motor_number] == 2){
      flag_pokeCheck[motor_number] = 0;//if it was in poke back mode, then back-->stop
    }
  }
}

void updateMotor(int motor_number) {
  
  if (voltageFeed[motor_number] > 0) { 
    motorDrive(motor_number, turnDirection, voltageFeed[motor_number]);
  }
  else if (voltageFeed[motor_number] < 0) {
    motorDrive(motor_number, !turnDirection, -voltageFeed[motor_number]);
  }
  else  {
    motorActive[motor_number] = false;
    motorStop(motor_number);
    if (!motorActive[(motor_number / 2) * 2] and !motorActive[(motor_number / 2) * 2 + 1] ) {
      motorsStandby(motor_number / 2);
    }
  }
}

void motorDrive(int motorNumber, bool motorDirection, unsigned short motorVoltage)
{
  bool pinIn1;  

  if (motorDirection == true)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  //Select the motor to turn, and set the direction and the Voltage
  if (motorNumber % 2 == 0) {
    digitalWrite(pinAIN1[motorNumber], pinIn1);
    digitalWrite(pinAIN2[motorNumber], !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA[motorNumber], motorVoltage);
  }
  else {
    digitalWrite(pinAIN1[motorNumber], !pinIn1);
    digitalWrite(pinAIN2[motorNumber], pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA[motorNumber], motorVoltage);
  }
  //Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY[motorNumber / 2], HIGH);
  //Serial.print(motorNumber); Serial.println(" GO");
  delay(1);
}

void motorBrake(int motorNumber)
{
  //This "Short Brake"s the specified motor, by setting Voltage to zero
  analogWrite(pinPWMA[motorNumber], 0);
}

void motorStop(int motorNumber)
{
  //This stops the specified motor by setting both IN pins to LOW
  digitalWrite(pinAIN1[motorNumber], LOW);
  digitalWrite(pinAIN2[motorNumber], LOW);
}

void motorsStandby(int STBY_number)
{
  digitalWrite(pinSTBY[STBY_number], LOW);
}

//298 DC////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Motor_rotate_rotate(int motor_number, float number) {

  if ((motor_number >= 0) && (motor_number <= 1)) {
    moveToBalanceLevel_rotate(motor_number , number);
  }
}

void moveToBalanceLevel_rotate(int motor_number, float bal) {

  //targetCount[motor_number] = bal * encoderValue[0];//需要絕對位置再用
  targetCount_rotate[motor_number] = bal * encoderValue_rotate[0] + oldPosition_rotate[motor_number];  //current position
  integral_rotate[motor_number] = 0; //reset integral in PID control
  motorActive_rotate[motor_number] = true;
}

void getVoltageFeedFromPID(int motor_number)
{
  long error;
  float I, D;
  error = targetCount_rotate[motor_number] - newPosition_rotate[motor_number];
  error_array_rotate[motor_number] = error;

  // Integral
  if (abs(error) < KiActivationErrorRange) {
    integral_rotate[motor_number] += error * usPassed[motor_number];
    if (abs(error) < KiRange2) {
      I = Ki * KiRange2Factor * integral_rotate[motor_number];
    } else {
      I = Ki * integral_rotate[motor_number];
    }
  } else {
    integral_rotate[motor_number] = 0;
    I = 0;
  }
  
  //Derivative
  if (abs(error) < KdActivationErrorRange) {
    D = Kd * encoderSpeed[motor_number];
  } else {
    D = 0;
  }

  //Derive driving voltage
  if (abs(targetCount_rotate[motor_number] - newPosition_rotate[motor_number]) > activationCountError_rotate) {
    motorActive_rotate[motor_number] = true;
  }
  if (abs(targetCount_rotate[motor_number] - newPosition_rotate[motor_number]) > acceptableCountError_rotate && motorActive_rotate[motor_number]) {
    // after motor has reached acceptableCountError, the error needs to excceed activationCountError to start the motor again.
    voltageFeed_rotate[motor_number] = Kp * (error) + I + D;

    if (voltageFeed_rotate[motor_number] > 0) {
      if (voltageFeed_rotate[motor_number] > maximalVoltage) {
        voltageFeed_rotate[motor_number] = maximalVoltage;
      }
    } else {
      if (voltageFeed_rotate[motor_number] < -maximalVoltage) {
        voltageFeed_rotate[motor_number] = -maximalVoltage;
      }
    }
  } 
  else {
    integral_rotate[motor_number] = 0;
    voltageFeed_rotate[motor_number] = 0;
    
    flag_rotateCheck[motor_number] = 1;
    //rotate到位了
  }
}

void updateMotor_rotate(int motor_number) {
  if (voltageFeed_rotate[motor_number] > 0) { 
    motorDrive_rotate(motor_number, turnDirection_rotate, voltageFeed_rotate[motor_number]);
  }
  else if (voltageFeed_rotate[motor_number] < 0) {
    motorDrive_rotate(motor_number, !turnDirection_rotate, -voltageFeed_rotate[motor_number]);
  }
  else  {
    motorActive_rotate[motor_number] = false;
    motorStop_rotate(motor_number);
    if (!motorActive_rotate[(motor_number / 2) * 2] and !motorActive_rotate[(motor_number / 2) * 2 + 1] ) {
      motorsStandby_rotate(motor_number / 2);
    }
  }
}

void motorDrive_rotate(int motorNumber, bool motorDirection, unsigned short motorVoltage)
{
  bool pinIn1; 
  if (motorDirection == true)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  //Select the motor to turn, and set the direction and the Voltage
  if (motorNumber % 2 == 0) {
    digitalWrite(pinAIN1_rotate[motorNumber], pinIn1);
    digitalWrite(pinAIN2_rotate[motorNumber], !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA_rotate[motorNumber], motorVoltage);
  }
  else {
    digitalWrite(pinAIN1_rotate[motorNumber], !pinIn1);
    digitalWrite(pinAIN2_rotate[motorNumber], pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA_rotate[motorNumber], motorVoltage);
  }
  //Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY_rotate[motorNumber / 2], HIGH);
}

void motorBrake_rotate(int motorNumber)
{
  analogWrite(pinPWMA_rotate[motorNumber], 0);
}

void motorStop_rotate(int motorNumber)
{

  //This stops the specified motor by setting both IN pins to LOW
  digitalWrite(pinAIN1_rotate[motorNumber], LOW);
  digitalWrite(pinAIN2_rotate[motorNumber], LOW);
}

void motorsStandby_rotate(int STBY_number)
{
  digitalWrite(pinSTBY_rotate[STBY_number], LOW);
}
