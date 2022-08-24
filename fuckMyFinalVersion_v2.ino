#include <Encoder.h>
#include <Servo.h>
//-------------------------------------------------------------------------------//SERVO

Servo servo[4];
int servoPin[4] = {29, 33, 39, 43};
int servoDefault = 90;

//-------------------------------------------------------------------------------//380

Encoder Enc1(21,35);
Encoder Enc2(20,27);
Encoder Enc3(19,31);
Encoder Enc4(18,37);
long newPosition1[4];
long oldPosition1[4] = {-999,-999,-999,-999};

//Define the Pins
const int pinAIN1[4] = {28, 32, 38, 42}; //Direction
const int pinAIN2[4] = {26, 30, 36, 40}; //Direction
const int pinPWMA[4] = {13, 11, 10, 6}; //Voltage  2~13,44~46
const int pinSTBY[2] = {12, 7};

//Constants to help remember the parameters
static boolean turnCW = 0;  //for motorDrive function
static boolean turnCCW = 1; //for motorDrive function

// Set motor variable
int cpr = 12;
int gearratio = 379; 

long current[4] = {0,0,0,0};
long target[4] = {0,0,0,0};//動態變化的target
long goal[4]= {0,0,0,0}; //紀錄該次指令的target-->不會變的target

long error[4] = {0,0,0,0};
long errorRange = 5;
int voltage[4] = {0,0,0,0};
long integral[4] = {0, 0, 0, 0};
bool motorActive[4] = {0,0,0,0};

//global
//long targetRoofLoop = 2; 
long targetRoofLoop = 1; 
long targetRoof = targetRoofLoop * cpr * gearratio; 
int maximalVoltage = 255;
const bool turnDirection = true;

//------------------//380 pid

const unsigned short acceptableCountError = 10;
const unsigned short activationCountError = 1;
const float Kp = 2;
const float Ki = 0;
const float Kd = 0; 
const int KiActivationErrorRange = 100;
const int KiRange2 = 10;
const float KiRange2Factor = 2;
const int KdActivationErrorRange = 100;
unsigned long usPassed[4] = {0, 0, 0, 0};
float encoderSpeed[4] = {0, 0, 0, 0};
long curCount[4] = {0, 0, 0, 0};

//------------------//380 fsr

const int fsrPIN[4] = {A0, A1, A2, A3}; 
int fsrADC[4] = {0, 0, 0, 0}; //偵測到的ADC-->動態變化
int threshold[4] = {10, 10, 10, 10};

//int fsrADC_7N[4] = {200,240,180,250}; //1.5~2N
//int fsrADC_7N[4] = {170,210,150,230}; //1N

int fsrADC_7N[4] = {170,210,150,230}; //1N
int fsrADC_11N[4] = {250,260,210,290}; //2.5N

//int fsrADC_11N[4] = {300,290,240,350}; //3~3.5N too big no use                                                                                                                  

int fsrADC_19N[4] = {350,420,400,375};//no use

long record[4] = {0, 0, 0, 0}; //test
long record_7N[4] = {0, 0, 0, 0};
long record_11N[4] = {0, 0, 0, 0};
long record_19N[4] = {0, 0, 0, 0};

int takeFirstRecord[4] = {0, 0, 0, 0}; //test
int takeFirstRecord_7N[4] = {0, 0, 0, 0};
int takeFirstRecord_11N[4] = {0, 0, 0, 0};
int takeFirstRecord_19N[4] = {0, 0, 0, 0};

//global
int level_N = 0; //what force

int counterToGo = 0; //how many motor haven't comeback

int motorToWait = 0;

bool counter_flag[4] = {0, 0, 0, 0};//test
bool counter_AddFlag[4] = {0, 0, 0, 0};

bool motorGo[4] = {false, false, false, false};

bool motorComplete[4] = {1,1,1,1}; //判斷馬達有沒有董()//----------這裡注意一下-----------//

int MotorState[4] = {0,0,0,0};    //380
int MotorState_rotate[2] = {0,0}; //298

//-------------------------------------------------------------------------------//298

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

const int motor_rotate[2] = {0, 1};
int encoderValue_rotate[1] = {3576};   //298*12

int teeth_of_brace = 85;
int teeth_of_brace_bigger = 90;
int teeth_of_car = 28;
int full_angle = 360;
const int complementary = 180;

int flag_rotateCheck[2] = {0,0}; 
///check if pks is at the right location 0-->not arrive 1--> arrive

//------------------//298 pid

const int KiActivationErrorRange_rotate = 100;
const int KiRange2_rotate = 10;
const float KiRange2Factor_rotate = 2;
const int KdActivationErrorRange_rotate = 100;
const float Kp_rotate = 2;
const float Ki_rotate = 0.00001; //0.00001
const float Kd_rotate = 0;
unsigned long usPassed_rotate[2] = {0, 0};
float encoderSpeed_rotate[2] = {0, 0};
long curCount_rotate[2] = {0, 0};

long voltageFeed_rotate[2] = {0, 0};
const bool turnDirection_rotate = true;

//-------------------------------------------------------------------------------//

long M[2] = {0,0}; //紀錄幾號馬達的終點
long Mnum[3] = {0,0,0}; //記錄幾號馬達
long gap;
long gapRange = 0;
int Mcount = 0;

float rotateLoop_Global = 0;
int teeth_forSupination = 2; //每個user不一樣 預設4

//-----------------INST------------------//

char inst;
char comma;
int pks;
int rotateAngle; 
int servoAngle, pokeMode;
int level;

int modeType = 0;
int motorGoNum = 0;

bool newInst = false;
bool doneRound = true;

char inst_b;
char comma_b;
int pks_b;
int rotateAngle_b; 
int servoAngle_b, pokeMode_b;
int level_b;

int servoExtreme = 70;
int servoY = 90 - servoExtreme;
int servoXZ = 10;

int rotateUP = 80;
int rotateDOWN = 100;

float wristband = 112.5;

int angleMove[4] = {0,0,0,0};
int toLow[4] = {15,20,20,5};
int toHigh[4] = {90,90,95,90};

//-------------------------------------------------------------------------------//

void setup()
{  
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(fsrPIN[i], INPUT);
    pinMode(pinPWMA[i], OUTPUT);
    pinMode(pinAIN1[i], OUTPUT);
    pinMode(pinAIN2[i], OUTPUT);
    servo[i].attach(servoPin[i]);
    servo[i].write(servoDefault);
    
    if (i < 2) {
      pinMode(pinSTBY[i], OUTPUT);
      digitalWrite(pinSTBY, HIGH);
    }
  }
  
  for (int i = 0; i < 2; i++) {
    pinMode(pinPWMA_rotate[i], OUTPUT);
    pinMode(pinAIN1_rotate[i], OUTPUT);
    pinMode(pinAIN2_rotate[i], OUTPUT);
    if (i < 1) { 
      pinMode(pinSTBY_rotate[i], OUTPUT);
    }
  }
}

void loop()
{ 
    feedInst();
    performInst();
    
    read_encoder();
    read_fsr();
    getusPassed();
    getEncoderSpeed();

    getusPassed2();
    getEncoderSpeed2();

    modeControl();  
    if( motorToWait != 0){
      
      counterToGo = motorToWait;
      motorToWait = 0;
    }
    
    for ( int i = 0; i < 4; i++) {     
      getVoltage1(i);
      updateMotor(i);
    }
    
    for ( int i = 0; i < 2; i++) {
      getVoltage2(i);
      updateMotor_rotate(i);
    }
}

//-----------------------//


void moveServo( int angleNumber ,int motorNum ){
  
  //Serial.println(angleNumber);

  angleMove[motorNum] = map(angleNumber,0,90,toLow[motorNum],toHigh[motorNum]);
  servo[motorNum].write(angleMove[motorNum]);
  //Serial.println(angleMove[motorNum]);
  
}

//-------------------------------------------------------------------------------//

void serialEvent() {
  
  if (Serial.available() > 0){
 
    inst_b = Serial.read();

    if(inst_b == 'a'){
      int motorNum = Serial.parseInt();
    }

    else if(inst_b == 'b'){ //初始校正位置使用
      //b,3,-14
      int motorNum = Serial.parseInt();
      int teeth;
      teeth = Serial.parseInt();
      float teeth_f = teeth;
      float rotateNum = teeth_f/18; 
      Motor_rotate(motorNum,rotateNum);
    } 

    else if(inst_b == 'd'){
      //d,1,45
      int motorNum = Serial.parseInt();
      int servoAngle = Serial.parseInt();
      moveServo(servoAngle, motorNum);
    }
    
    else if(inst_b == 'g'){
      //g,0,120
      //g,1,120

      int pks = Serial.parseInt();
      int rotateAngle = Serial.parseInt();
       
      float rotateAngleFloat = rotateAngle;
      float circle = rotateAngleFloat/wristband;
      
      int motor_num;
      
      if( pks == 0 ){
        motor_num = 0;
        Motor_rotate_rotate(motor_num,circle);
      }
      
      else if( pks == 1 ){
        motor_num = 1;
        Motor_rotate_rotate(motor_num,circle);
      }
    }
    
   
    else if(inst_b == 'h'){
      //h,0,120
      int pks = Serial.parseInt();  
      int rotateAngle = Serial.parseInt();
       
      float rotateAngleFloat = rotateAngle;
      float circle = rotateAngleFloat/wristband;

      if( pks == 0){
        Motor_rotate_rotate(0,circle);
        Motor_rotate_rotate(1,circle);
      }
    }

    else if(inst_b == 'i'){
      Motor_rotate_rotate(0,0);
      Motor_rotate_rotate(1,0);
    }
    
    else if(inst_b == 't'){

      pks_b = Serial.parseInt(); 
      rotateAngle_b = Serial.parseInt(); 
      servoAngle_b = Serial.parseInt();
      level_b = Serial.parseInt();

      //rotateUP = 80 ; rotateDOWN = 100 ; 

      if( (abs(rotateAngle_b)  > rotateUP && abs(rotateAngle_b) < rotateDOWN ) && (abs(rotateAngle)  > rotateUP && abs(rotateAngle) < rotateDOWN ) ){
        //Serial.println("boool");
         
        pks_b = pks;
        rotateAngle_b = rotateAngle;
        
      }

      newInst = true;

    }
    
      else if(inst_b == 'r'){
        
      pks_b = Serial.parseInt();
      rotateAngle_b = Serial.parseInt(); 
      pokeMode_b = Serial.parseInt();
      level_b = Serial.parseInt();

      if( (abs(rotateAngle_b)  > rotateUP && abs(rotateAngle_b) < rotateDOWN ) && (abs(rotateAngle)  > rotateUP && abs(rotateAngle) < rotateDOWN ) ){
        //Serial.println("boool");
        //pks_b = pks;
        rotateAngle_b = rotateAngle;
      }

      newInst = true;

    }

    else if(inst_b == 's'){
      
      pks_b = Serial.parseInt();
      rotateAngle_b = Serial.parseInt(); 
      
      newInst = true;

    }
    else if(inst_b == 'x'){ 
      newInst = true;
    }
    
    
    //-------------------------------------------------------------------------------// official inst.
    char icon = Serial.read();//確保不再跑一次
  }
}

void feedInst(){
  
  if( newInst == true && doneRound == true ){

    //only print once
    //-----------------------------------flush
    
    for( int i = 0; i < 4; i++){
      
      motorGo[i] = false;
      MotorState[i] = 0;

      servo[i].write(90);

      takeFirstRecord[i] = 0;
      takeFirstRecord_7N[i] = 0;
      takeFirstRecord_11N[i] = 0;
      takeFirstRecord_19N[i] = 0;

      GoBackAndStop(i);
    }
    
    for( int i = 0; i < 2; i++){
      flag_rotateCheck[i] = 0;
    }
    
    motorGoNum = 0;
    modeType = 0;
    counterToGo = 0;
    
    for( int i = 0; i < 2; i++){
      M[i] = 0;
    }
    for( int i = 0; i < 3; i++){
      Mnum[i] = 0;
    }

    gap = 0;
    gapRange = 0;
    Mcount = 0;
    
    //-----------------------------------flush end
    inst = inst_b;
    pks = pks_b;
    rotateAngle = rotateAngle_b;
    servoAngle = servoAngle_b;
    pokeMode = pokeMode_b;
    level = level_b;
    //Serial.println("------------------new!!!!!------------------");
    
    newInst = false;
  }
}

void performInst(){
  
  if( newInst == false && doneRound == true ){ //沒有新指令的時候 開始read
    
    if(inst == 't'){

      modeType = 1;

      int rotateMove;
      float rotateAngleFloat;
      float rotateLoop;

      if( pks == 0 ){
        if( rotateAngle < 0){
          rotateAngleFloat = rotateAngle + complementary; //+180 
        }
        else if( rotateAngle > 0){
          rotateAngleFloat = rotateAngle - complementary; //-180
        }
        else{ //rotateAngle = 0
          rotateAngleFloat = rotateAngle;
        }
      }
      
      else if( pks == 1 ){
          rotateAngleFloat = rotateAngle;
      } 

      rotateLoop = rotateAngleFloat/wristband;

      
      Motor_rotate_rotate(0,rotateLoop); 
      Motor_rotate_rotate(1,rotateLoop);
      
      
      int servoAngleNew;
      int motorNum;
      servoAngleNew = servoDefault - abs(servoAngle);

      if(  pks == 0 ){
        //--->0 -90~0
        //
        if((servoAngle < 0) && (abs(servoAngle) > servoXZ )){ //-------------------------------------

          //下後方
          if(abs(servoAngle) < servoExtreme ){  
            moveServo(servoAngleNew, 0); 
            setThershold(0,level);
            motorGo[0] = true;
            motorGoNum = 1;
          }
          //正後方
          else{ 
            moveServo(servoY, 0);
            moveServo(servoY, 2);
            setThershold(0,level);
            setThershold(2,level);
            motorGo[0] = true;
            motorGo[2] = true;
            motorGoNum = 2;
          }
        }
        
        //--->1 0~90
        else if((servoAngle > 0) && (abs(servoAngle) > servoXZ )){

          //下前方
          if(abs(servoAngle) < servoExtreme){  
            moveServo(servoAngleNew, 1);
            setThershold(1,level);
            motorGo[1] = true;
            motorGoNum = 1;
          }
          //正前方
          else {
            
            moveServo(servoY, 1);
            moveServo(servoY, 3);
            setThershold(1,level);
            setThershold(3,level);
            motorGo[1] = true;
            motorGo[3] = true;
            motorGoNum = 2;
          }
        }

        //xz正方向
        else{
          setThershold(0,level);
          setThershold(1,level);
          motorGo[0] = true;
          motorGo[1] = true;
          motorGoNum = 2;
        }
      }
      
      else if(  pks == 1 ){
        
        //--->2
        if((servoAngle < 0) && (abs(servoAngle) > servoXZ )){ //-------------------------------------

          //上後方
          if(abs(servoAngle) < servoExtreme){  
            moveServo(servoAngleNew, 2);
            setThershold(2,level);
            motorGo[2] = true;
            motorGoNum = 1;
          }
          //正後方
          else {
            moveServo(servoY, 0);
            moveServo(servoY, 2);
            setThershold(0,level);
            setThershold(2,level);
            motorGo[0] = true;
            motorGo[2] = true;
            motorGoNum = 2;
          }
        }
        
        //--->3
        else if((servoAngle > 0) && (abs(servoAngle) > servoXZ )){
          
          //上前方
          if(abs(servoAngle) < servoExtreme){  
            moveServo(servoAngleNew, 3);
            setThershold(3,level);
            motorGo[3] = true;
            motorGoNum = 1;
          }
          //正前方
          else{
            moveServo(servoY, 1);
            moveServo(servoY, 3);
            setThershold(1,level);
            setThershold(3,level);
            motorGo[1] = true;
            motorGo[3] = true;
            motorGoNum = 2;
          }
        }
        
        //--->2 3
        //xz正方向
        else{
          setThershold(2,level);
          setThershold(3,level);
          motorGo[2] = true;
          motorGo[3] = true;
          motorGoNum = 2;

        }
      }
    }
    
    //-------------------------------------------------------------------------------// official inst.
      else if(inst == 'r'){
        
      modeType = 1;
      
      //-----------------------------//298

      float rotateAngleFloat;
      float rotateLoop;
      
      if( rotateAngle < 0){
        rotateAngleFloat = rotateAngle + complementary; //+180
      }
      else if( rotateAngle > 0){
        rotateAngleFloat = rotateAngle - complementary; //-180
      }
      else{ //rotateAngle = 0
        rotateAngleFloat = rotateAngle;
      }

      rotateLoop = rotateAngleFloat/wristband;
      
      Motor_rotate_rotate(0,rotateLoop); 
      Motor_rotate_rotate(1,rotateLoop);
      
      
      //-----------------------------//380

      if(  pks == 0 ){
        //--->-1 (1,2)
        if( pokeMode == 1){ 
          
          //r,0,120,1,level

          setThershold(1,level);
          setThershold(2,level);
          motorGo[1] = true;
          motorGo[2] = true;

          motorGoNum = 2;

        }
        
        else if( pokeMode == 0){

          //r,0,120,0,level

          setThershold(0,level);
          setThershold(3,level);
          motorGo[0] = true;
          motorGo[3] = true;

          motorGoNum = 2;

        }
        else{}
      }
    }

    else if(inst == 's'){
      
      modeType = 2;

      Motor_rotate_rotate(0,0); 
      Motor_rotate_rotate(1,0);
      
      float rotateAngleFloat = rotateAngle;
      float rotateLoop = rotateAngleFloat/wristband;
      
      rotateLoop_Global = rotateLoop;
      
      motorGo[0] = true;
      motorGo[1] = true;
      motorGo[2] = true;
      motorGo[3] = true;

      motorGoNum = 4;

    }
    else if(inst == 'x'){
      //Serial.println("do nothing"); 
    }

    else {}
  }
}

//-------------------------------------------------------------------------------//

void Motor_rotate(int motorNumber, float loopnumber) {
  
  target[motorNumber] = loopnumber * cpr * gearratio;
  goal[motorNumber] = target[motorNumber];
  integral[motorNumber] = 0;
  motorActive[motorNumber] = true;
}

void Motor_toValue(int motorNumber, long targetValue) {
  
  target[motorNumber] = targetValue;
  //goal[motorNumber] =  target[motorNumber];//防止後面goal洗掉前面紀錄
  integral[motorNumber] = 0;
  motorActive[motorNumber] = true;
}

void GoBackAndStop(int motorNumber){ 
  Motor_toValue(motorNumber,0);
}

void FirstTime_rotate(int motorNumber){  
  goal[motorNumber] = targetRoof;
  Motor_toValue(motorNumber,targetRoof);
}

void Supination_rotate(int motorNumber , int teeth){ 
  float teeth_f = teeth;
  float rotateNum = teeth_f/18; 
  Motor_rotate(motorNumber,rotateNum); 
}

//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//

void modeControl(){
  
  for(int i = 0; i < 4; i++){

    if( MotorState[i] == 0 ){ //prepare to go; 0 --> 1
      
      if( motorGo[i] == true ){
        
        if(flag_rotateCheck[0] == true && flag_rotateCheck[1] == true){
          
          doneRound = false; 
          //Serial.print("motor");Serial.print(i);Serial.println("woooo");
          
          if( modeType == 1 ){
            
            FirstTime_rotate(i);
            counterToGo++;
            MotorState[i] = 1;
          }
          else if( modeType == 2 ){
            
            Supination_rotate(i,teeth_forSupination);
            counterToGo++;
            MotorState[i] = 1;
          }
        }
      }
    }

    else if( MotorState[i] == 1 ){ //prepare to stop at skin ; 1-->2
      
      if( motorGo[i] == true ){
        
        if(oldPosition1[i] >= goal[i]){
          MotorState[i] = 2;
        }
      }
    }
    
    else if( MotorState[i] == 2){ //at skin, prepare to back ; 2-->3

      //Serial.print("motor");Serial.print(i); Serial.print(" state 2");
      //Serial.print("doneRound");Serial.println(doneRound);
      
      if( modeType == 1){ 
        
        Motor_toValue(i,0);
        MotorState[i] = 3; 
        //Serial.print("motor");Serial.print(i); Serial.print(" state 2 --> 3");
        //Serial.print("doneRound");Serial.println(doneRound);
        
        //--------------------------//
        if( motorGoNum == 2 ){
          
          if( Mcount == 0 ){
            M[0] = goal[i];
            
            //Mnum[0] = i; +1
            Mnum[1] = i;
            
            Mcount++;
          }
          else if( Mcount == 1 ){
            M[1] = goal[i];
            
            //Mnum[1] = i; +1
            Mnum[2] = i;
            
            gap = M[1] - M[0];
            Mcount++;
          }
          else {}
        }
        //--------------------------//
      }
      
      else if( modeType == 2){ 
        
        Motor_rotate_rotate(0,rotateLoop_Global);
        Motor_rotate_rotate(1,rotateLoop_Global);

        if( flag_rotateCheck[0] == true && flag_rotateCheck[1] == true){
          
          Motor_toValue(i,0);
          MotorState[i] = 3; 
        }
      }
    }

    else if( MotorState[i] == 3 ){ //back to zero ; 3-->4
//
      //Serial.print("motor");Serial.print(i); Serial.print(" state 33333");
      //Serial.print("doneRound");Serial.println(doneRound);
      
      if( oldPosition1[i] <= errorRange ){
 
        counterToGo--;
        MotorState[i] = 4;
      }
    }

  }
  //end for
  
  for( int i = 0; i < 4; i++){

//    Serial.print("motor");Serial.print(i);Serial.println(" is at 4");
//    Serial.print("counterToGo:");Serial.println(counterToGo);
    
     if( MotorState[i] == 4 && counterToGo == 0){ //all back ; 4 --> 1  

      if( doneRound == true ){ 
        
        if( motorGo[i] == true ){     
  
          if( modeType == 1){
            
            //--------------------------//
            if( motorGoNum == 1 ){

              doneRound = false;
              
              Motor_toValue(i,goal[i]);
              motorToWait++;
              MotorState[i] = 1;
            }
           
            else if( motorGoNum == 2 ){
              
              if( i == Mnum[2] ){

                doneRound = false;
  
                Motor_toValue(Mnum[2],M[1]);
                MotorState[Mnum[2]] = 1;

              }
              else if(i == Mnum[1]){
                
                if( oldPosition1[Mnum[2]] >= gap){
                  
                   Motor_toValue(Mnum[1],M[0]);
                   MotorState[Mnum[1]] = 1;
                   
                   motorToWait = 2; // = 1, = 2

                }
              }
            }
            //--------------------------//
          }
          
          else if( modeType == 2){ 
            
            Motor_rotate_rotate(0,0);
            Motor_rotate_rotate(1,0);
  
            if( flag_rotateCheck[0] == true && flag_rotateCheck[1] == true){

              doneRound = false;
              
              Motor_toValue(0,goal[0]);
              MotorState[0] = 1; 
              Motor_toValue(1,goal[1]);
              MotorState[1] = 1; 
              Motor_toValue(2,goal[2]);
              MotorState[2] = 1; 
              Motor_toValue(3,goal[3]);
              MotorState[3] = 1; 
              
              motorToWait = 4;

            }
          }
        }
      }

      else{
          doneRound = true;
      }
    }
  }
}

//-------------------------------------------------------------------------------//

void read_encoder() {
  
  newPosition1[0] = Enc1.read();
  newPosition1[1] = Enc2.read();
  newPosition1[2] = Enc3.read();
  newPosition1[3] = Enc4.read(); 
  for(int i = 0; i < 4 ; i++){
    if (newPosition1[i] != oldPosition1[i]) {
      oldPosition1[i] = newPosition1[i];
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


void read_fsr(){
  for( int i = 0; i < 4; i++){
    
    fsrADC[i] =  analogRead(fsrPIN[i]);
    
    if(fsrADC[i] > threshold[i] && takeFirstRecord[i] == 0){
      
      if( level_N == 7 && takeFirstRecord_7N[i] == 0){
        
        takeFirstRecord_7N[i] = 1;
        record_7N[i] = oldPosition1[i];
        target[i] = record_7N[i];
        goal[i] = target[i];        
      }
      
      else if( level_N == 11 && takeFirstRecord_11N[i] == 0){
      
        takeFirstRecord_11N[i] = 1;
        record_11N[i] = oldPosition1[i];
        target[i] = record_11N[i]; 
        goal[i] = target[i];
      }
      
      else if( level_N == 19 && takeFirstRecord_19N[i] == 0){
        
        takeFirstRecord_19N[i] = 1;
        record_19N[i] = oldPosition1[i];
        target[i] = record_19N[i];
        goal[i] = target[i];
      }
      else {}
    }
  }
}
//-------------------------------------------------------------------------------////-------------------------------------------------------------------------------//

void setThershold( int motorNumber , int level){
  
  if( level == 1){
    level_N = 7;
    threshold[motorNumber] = fsrADC_7N[motorNumber];
  }
  else if( level == 2){
    level_N = 11;
    threshold[motorNumber] = fsrADC_11N[motorNumber];
  }
  else if( level == 19){
    level_N = 19;
    threshold[motorNumber] = fsrADC_19N[motorNumber];
  }
}

//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//

void motorDrive(int motorNumber, boolean motorDirection, int motorSpeed)
{
  boolean pinIn1;  
  
  if (motorDirection == turnCW){
    pinIn1 = HIGH;
  }
  else{
    pinIn1 = LOW;
  }

  //Select the motor to turn, and set the direction and the speed
  if(motorNumber % 2 == 0)
  {
    digitalWrite(pinAIN1[motorNumber], pinIn1);
    digitalWrite(pinAIN2[motorNumber], !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA[motorNumber], motorSpeed);
  }
  else
  {
    digitalWrite(pinAIN1[motorNumber], !pinIn1);
    digitalWrite(pinAIN2[motorNumber], pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA[motorNumber], motorSpeed);
  }
   
  //Finally , make sure STBY is disabled - pull it HIGH
  //digitalWrite(pinSTBY, HIGH);

  digitalWrite(pinSTBY[motorNumber / 2], HIGH);
}

void motorBrake(int motorNumber)
{
  /*
  This "Short Brake"s the specified motor, by setting speed to zero
  */
  analogWrite(pinPWMA[motorNumber], 0);
}


void motorStop(int motorNumber)
{
  /*
  This stops the specified motor by setting both IN pins to LOW
  */
  digitalWrite(pinAIN1[motorNumber], LOW);
  digitalWrite(pinAIN2[motorNumber], LOW);
}


void motorsStandby(int STBY_number)
{
  /*
  This puts the motors into Standby Mode
  */
  digitalWrite(pinSTBY[STBY_number], LOW);
}

//-------------------------------------------------------------------------------//

void getVoltage1(int motor_number)
{
  long error;
  float I, D;

  error = target[motor_number] - newPosition1[motor_number];

  // Integral
  if (abs(error) < KiActivationErrorRange) {
    integral[motor_number] += error * usPassed[motor_number];
    
    if (abs(error) < KiRange2) {
      I = Ki * KiRange2Factor * integral[motor_number];
    } else {
      I = Ki * integral[motor_number];
    }
  } else {
    integral[motor_number] = 0;
    I = 0;
  }
  
  //Derivative
  if (abs(error) < KdActivationErrorRange) {
    D = Kd * encoderSpeed[motor_number];
  } else {
    D = 0;
  }
  
  //Derive driving voltage
  if (abs(target[motor_number] - newPosition1[motor_number]) > activationCountError) {
    motorActive[motor_number] = true;
  }
  if (abs(target[motor_number] - newPosition1[motor_number]) > acceptableCountError && motorActive[motor_number]) {
    
    voltage[motor_number] = Kp * (error) + I + D;

    if (voltage[motor_number] > 0) {
      if (voltage[motor_number] > maximalVoltage) {
        voltage[motor_number] = maximalVoltage;
      }
    } else {
      if (voltage[motor_number] < -maximalVoltage) {
        voltage[motor_number] = -maximalVoltage;
      }
    }
    
  } else {
    //motorComplete[motor_number] = true;
    integral[motor_number] = 0;
    voltage[motor_number] = 0;
  }
}

void getusPassed() {
  static unsigned long prevTime[4] = {0, 0, 0, 0};
  unsigned long curTime[4] = {0, 0, 0, 0};

  for (int i = 0; i < 4; i++) {
    curTime[i] = micros();
    usPassed[i] = curTime[i] - prevTime[i];
    prevTime[i] = curTime[i];
  }
}

void getEncoderSpeed() {

  static unsigned long usPassedBetweenEncoderReadings[4] = {0, 0, 0, 0};
  static int prevCount[4] = {0, 0, 0, 0};
  float newSpeed[4] = {0, 0, 0, 0};

  for (int i = 0; i < 4; i++) {
    usPassedBetweenEncoderReadings[i] += usPassed[i];
    if (usPassed[i] > 1000) {
      newSpeed[i] = (float)(curCount[i] - prevCount[i]) * 0.001;
      prevCount[i] = curCount[i];
      usPassedBetweenEncoderReadings[i] -= 1000;
      encoderSpeed[i] = encoderSpeed[i] * 0.8 + (newSpeed[i]) * 0.2;
    }
  }
}

void updateMotor(int motorNumber) {
  
  if (voltage[motorNumber] > 0) { 
    motorDrive(motorNumber, turnDirection, voltage[motorNumber]);
  }
  else if (voltage[motorNumber] < 0) {
    motorDrive(motorNumber, !turnDirection, -voltage[motorNumber]);
  }
  else  {
    motorActive[motorNumber] = false;
    motorStop(motorNumber);  
    if (!motorActive[(motorNumber / 2) * 2] and !motorActive[(motorNumber / 2) * 2 + 1] ) {
      motorsStandby(motorNumber / 2);
    }
  }
}

//298 DC////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Motor_rotate_rotate(int motor_number, float number) {
  if ((motor_number >= 0) && (motor_number <= 1)) {
    moveToBalanceLevel_rotate(motor_number , number);
  }
}

void moveToBalanceLevel_rotate(int motor_number, float bal) {

  targetCount_rotate[motor_number] = bal * encoderValue_rotate[0];
  
  integral_rotate[motor_number] = 0; //reset integral in PID control
  motorActive_rotate[motor_number] = true;
}

void getVoltage2(int motor_number)
{
  long error;
  float I, D;
  error = targetCount_rotate[motor_number] - newPosition_rotate[motor_number];
  error_array_rotate[motor_number] = error;

  // Integral
  if (abs(error) < KiActivationErrorRange_rotate) {
    integral_rotate[motor_number] += error * usPassed_rotate[motor_number];
    if (abs(error) < KiRange2_rotate) {
      I = Ki_rotate * KiRange2Factor_rotate * integral_rotate[motor_number];
    } else {
      I = Ki_rotate * integral_rotate[motor_number];
    }
  } else {
    integral_rotate[motor_number] = 0;
    I = 0;
  }
  
  //Derivative
  if (abs(error) < KdActivationErrorRange_rotate) {
    D = Kd_rotate * encoderSpeed_rotate[motor_number];
  } else {
    D = 0;
  }

  //Derive driving voltage
  if (abs(targetCount_rotate[motor_number] - newPosition_rotate[motor_number]) > activationCountError_rotate) {
    motorActive_rotate[motor_number] = true;
  }
  if (abs(targetCount_rotate[motor_number] - newPosition_rotate[motor_number]) > acceptableCountError_rotate && motorActive_rotate[motor_number]) {
    // after motor has reached acceptableCountError, the error needs to excceed activationCountError to start the motor again.
    voltageFeed_rotate[motor_number] = Kp_rotate * (error) + I + D;

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
    
    flag_rotateCheck[motor_number] = 1;//-----------------------------這裡注意一下-----------------------------//
  }
}

void updateMotor_rotate(int motor_number) {
  if (voltageFeed_rotate[motor_number] > 0) { 
    motorDrive_rotate(motor_number, turnDirection_rotate, voltageFeed_rotate[motor_number]);
    flag_rotateCheck[motor_number] = 0;
  }
  else if (voltageFeed_rotate[motor_number] < 0) {
    motorDrive_rotate(motor_number, !turnDirection_rotate, -voltageFeed_rotate[motor_number]);
    flag_rotateCheck[motor_number] = 0;
  }
  else  {
    motorActive_rotate[motor_number] = false;
    motorStop_rotate(motor_number);
    
    if (!motorActive_rotate[(motor_number / 2) * 2] and !motorActive_rotate[(motor_number / 2) * 2 + 1] ) {
      motorsStandby_rotate(motor_number / 2);
    }
  }
}
//-------------------------------------------------------------------------------//

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

void getusPassed2() {
  static unsigned long prevTime2[2] = {0, 0};
  unsigned long curTime2[2] = {0, 0};
  
  for (int i = 0; i < 2; i++) {
    curTime2[i] = micros();
    usPassed_rotate[i] = curTime2[i] - prevTime2[i];
    prevTime2[i] = curTime2[i];
  }
}

void getEncoderSpeed2() {

  static unsigned long usPassedBetweenEncoderReadings2[2] = {0, 0};
  static int prevCount2[2] = {0, 0};
  float newSpeed2[2] = {0, 0};

  for (int i = 0; i < 2; i++) {
    usPassedBetweenEncoderReadings2[i] += usPassed_rotate[i];
    if (usPassed_rotate[i] > 1000) {
      newSpeed2[i] = (float)(curCount_rotate[i] - prevCount2[i]) * 0.001;
      prevCount2[i] = curCount_rotate[i];
      usPassedBetweenEncoderReadings2[i] -= 1000;
      encoderSpeed_rotate[i] = encoderSpeed_rotate[i] * 0.8 + (newSpeed2[i]) * 0.2;
    }
  }
}
