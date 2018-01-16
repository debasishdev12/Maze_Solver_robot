#include <QTRSensors.h>
#include<string.h>

//----------------------------------------------------------

//#define DEBUG
#define MAZE_SOLVE
//#define LINE_FOLLOW
//#define AUTO_CALIBRATION
#define MANUAL_CALIBRATION
//#define RIGHT_HAND_RULE
#define LEFT_HAND_RULE
//#define LOW_VOLTAGE
#define HIGH_VOLTAGE

//----------------------------------------------------------
//**********************************************************
#ifdef LINE_FOLLOW
  #undef MAZE_SOLVE
#endif

#ifdef MAZE_SOLVE
  #undef LINE_FOLLOW
#endif

#ifdef HIGH_VOLTAGE
  #undef LOW_VOLTAGE
#endif

#ifdef LOW_VOLTAGE
  #undef HIGH_VOLTAGE
#endif

#ifdef AUTO_CALIBRATION
  #undef MANUAL_CALIBRATION
#endif

#ifdef MANUAL_CALIBRATION
  #undef AUTO_CALIBRATION
#endif

#ifdef LEFT_HAND_RULE
  #undef RIGHT_HAND_RULE
#endif

#ifdef RIGHT_HAND_RULE
  #undef LEFT_HAND_RULE
#endif
//**********************************************************

#define NUM_SENSORS   8
#define NUM_SAMPLES_PER_SENSOR  5
#define EMITTER_PIN   12
#define THRESHOLD 400

#define MAX_SPEED             THRESHOLD           //60
#define left_base_speed       110            //70 88 100 120
#define right_base_speed      138          //79 99 112 135
#define calibration_speed     110
//#define left_motor_A                      //PB0
//#define left_motor_B                      //PB1
//#define right_motor_A                     //PB2
//#define right_motor_B                     //PB3
#define left_motor_pwm      3               //PD3
#define right_motor_pwm     5               //PD5

#define ON_LINE(sensor)(sensor>THRESHOLD)
#define OFF_LINE(sensor)(sensor<THRESHOLD)

const uint8_t turnSpeed = 60;


#ifdef MAZE_SOLVE
  uint8_t pathLength;
  uint8_t readLength;
  char path[50],reverse_path[50];
  byte irSensor = B00000000;
  bool replayStage = 0;
  const uint8_t maze_left_base_speed = 55;          //45
  const uint8_t maze_right_base_speed = 69;         //56
#endif


int lastError = 0;

QTRSensorsAnalog qtra((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int s[NUM_SENSORS];

void setup()
{
  DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB5);                      //8 9 10 11 
  DDRD |= (1<<PD3)|(1<<PD5);                                        //pwm pin
  PORTB &= (~(1<<PB0))&(~(1<<PB1))&(~(1<<PB2))&(~(1<<PB3))&(~(1<<PB5));   //8 9 10 11 12low
  PORTD &= (~(1<<PD3))&(~(1<<PD5));
  pinMode(6,INPUT_PULLUP);          //connect push button, press the button to start after calibrating the robot manually
//  digitalWrite(6,HIGH);
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Ready");
  #endif

  
  #ifdef AUTO_CALIBRATION
  
  for (int i = 0; i < 100; i++)  // make the calibration take about 10 seconds
  {
    if (i < 25 || i >= 50)
    {
      sharpRight(calibration_speed, calibration_speed);
    }
    else
    {
      sharpLeft(calibration_speed, calibration_speed);
    }
    qtra.calibrate();
  }

  brake();
  _delay_ms(500);
  
  while(1)           //CHANGE HERE
  {
    sharpLeft(60,60);
    qtra.read(s);
    if(s[3]>THRESHOLD)  break;
  }
  brake();
  _delay_ms(2000);
  
  #endif

  #ifdef MANUAL_CALIBRATION
  
  PORTB |= 1<<PB5;
  for (int i = 0; i < 200; i++)
  {
    qtra.calibrate();
  }
  PORTB &= ~(1<<PB5);
  brake();
  _delay_ms(2000);
  
  #endif
}

void loop()
{
  #ifdef LINE_FOLLOW
    pidMovement();
  #endif
  
  #ifdef MAZE_SOLVE

    #ifdef LEFT_HAND_RULE
      leftHandWall();
    #endif

    #ifdef RIGHT_HAND_RULE
      rightHandWall();
    #endif
  
  #endif
  
  return;
}

#ifdef LINE_FOLLOW

void pidMovement()
{
  double Kp = .02;
  double Kd = 1.1;
  position = qtra.readLine(s);
  int error  = position - 3500;
  int pid = Kp * error + Kd * (error - lastError);
  lastError = error;
  int leftMotorSpeed = left_base_speed - pid;
  int rightMotorSpeed = right_base_speed + pid;

  if (leftMotorSpeed > MAX_SPEED)  leftMotorSpeed = MAX_SPEED;
  if (rightMotorSpeed > MAX_SPEED) rightMotorSpeed = MAX_SPEED;
  if (leftMotorSpeed < -MAX_SPEED) leftMotorSpeed = -MAX_SPEED;
  if (rightMotorSpeed < -MAX_SPEED) rightMotorSpeed = -MAX_SPEED;

  moveForward(leftMotorSpeed, rightMotorSpeed);
}

#endif

inline void moveForward(int a, int b)
{
  if (a > 0 && b > 0)
  {
    PORTB |= (1<<PB1)|(1<<PB3);
    PORTB &= (~(1<<PB0))&(~(1<<PB2));
    analogWrite(left_motor_pwm, abs(a));
    analogWrite(right_motor_pwm, abs(b));
  }
  if (a < 0 && b < 0)
  {
    PORTB |= (1<<PB0)|(1<<PB2);
    PORTB &= (~(1<<PB1))&(~(1<<PB3));
    analogWrite(left_motor_pwm, abs(a));
    analogWrite(right_motor_pwm, abs(b));
  }
  if (a > 0 && b < 0)
  {
    PORTB |= (1<<PB1)|(1<<PB2);
    PORTB &= (~(1<<PB0))&(~(1<<PB3));
    analogWrite(left_motor_pwm, abs(a));
    analogWrite(right_motor_pwm, abs(b));
  }
  if (a < 0 && b > 0)
  {
    PORTB |= (1<<PB0)|(1<<PB3);
    PORTB &= (~(1<<PB1))&(~(1<<PB2));
    analogWrite(left_motor_pwm, abs(a));
    analogWrite(right_motor_pwm, abs(b));
  }
}
inline void sharpLeft(int a, int b)
{
  PORTB |= (1<<PB0)|(1<<PB3);
  PORTB &= (~(1<<PB1))&(~(1<<PB2));
  analogWrite(left_motor_pwm, a);
  analogWrite(right_motor_pwm, b);
}
inline void sharpRight(int a, int b)
{
  PORTB &= (~(1<<PB0))&(~(1<<PB3));
  PORTB |= (1<<PB1)|(1<<PB2);
  analogWrite(left_motor_pwm, a);
  analogWrite(right_motor_pwm, b);
}
inline void smoothLeft(int b)
{
  PORTB |= (1<<PB3);
  PORTB &= (~(1<<PB0))&(~(1<<PB2))&(~(1<<PB1));
  analogWrite(right_motor_pwm, abs(b));
}

inline void smoothRight(int a)
{
  PORTB |= (1<<PB1);
  PORTB &= (~(1<<PB0))&(~(1<<PB2))&(~(1 << PB3));
  analogWrite(left_motor_pwm, abs(a));
}

inline void brake()
{
  PORTB &= (~(1<<PB0))&(~(1<<PB1))&(~(1<<PB2))&(~(1<<PB3));
  analogWrite(left_motor_pwm, 0);
  analogWrite(right_motor_pwm, 0);
}






//############################################################  Maze_Solving  ######################################################

#ifdef MAZE_SOLVE

void checkPattern()
{
  int chk[NUM_SENSORS];
  irSensor = B00000000;
  unsigned int position = qtra.readLine(s);
  for(int i=0;i<NUM_SENSORS;i++)
  {
    if(s[i] > THRESHOLD)
      chk[i] = 1;
    else
      chk[i] = 0;
    int b = 7-i;
    irSensor = irSensor + (chk[i]<<b);
  }
//  #ifdef DEBUG
//    Serial.println(irSensor,BIN);
//  #endif
}

#ifdef LEFT_HAND_RULE

void leftHandWall()
{
  qtra.readLine(s);
  
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^TURN^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^RIGHT^^^^^^^^^^^^^^^^^^^^^^^^^^  
  
  /*
   *        |-------
   *        |
   *        |
   */
  if(ON_LINE(s[0]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(150);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(100);
    #endif
    
    brake();
    qtra.readLine(s);
    
    /*
     *          |------
     *          |
     *          |
     */
     
     if(OFF_LINE(s[3])&& OFF_LINE(s[4]))
     {
      turnRight();
     }
    
    /*
     *          |
     *          |------
     *          |
     */
    else
    {
      path[pathLength] = 'S';
      #ifdef DEBUG
        Serial.println("S");
      #endif
      pathLength++;
      if(path[pathLength-2] == 'B')
        shortPath();
      straight();
    }
  }
//^^^^^^^^^^^^^^^^^^^^^^^^^^TURN^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^LEFT^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  
  /*
   *      -------|
   *             |
   *             |
   */

  if(ON_LINE(s[7]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(160);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(100);
    #endif
    
    brake();
    
    /*
     *      -------|
     *             |
     *             |
     */
    
    /*
     *             |
     *             |
     *      -------|
     *             |
     *             |
     */
      
     turnLeft(); 
  }
//^^^^^^^^^^^^^^^^^^^^^^^^^^CROSS^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^INTERSECTION^^^^^^^^^^^^^^^^^^^^^^^^
  /*
   *              |
   *        ______|______
   *              |   
   *              |
   */
  //all sensors on black surface
  if(ON_LINE(s[0]) && ON_LINE(s[7]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(170);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(100);
    #endif
    
    brake();
    qtra.readLine(s);
    
    /*
     *      --------------
     *      **************
     *      **************
     *      
     */
     
    if(ON_LINE(s[0])&&ON_LINE(s[7])&&ON_LINE(s[3])&&ON_LINE(s[4]))
    {
      done();
    }
    
    /*     _______
     *        |
     *        |
     *        |
     */
     

    /*
     *        |
     *    ____|____
     *        |
     *        |
     */
     
    else
    {
      sharpLeft(turnSpeed,turnSpeed/.8);
      turnLeft();
    }
  }

//^^^^^^^^^^^^^^^^^^^^^^^^^^TURN^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^AROUND^^^^^^^^^^^^^^^^^^^^^^^^^^

  if(OFF_LINE(s[0])&&OFF_LINE(s[7])&&OFF_LINE(s[3])&&OFF_LINE(s[4]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(120);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(80);
    #endif
    
    brake();
    turnAround();
  }
   
  else pidMaze();
  return;
}

#endif

#ifdef RIGHT_HAND_RULE

void rightHandWall()
{
  qtra.readLine(s);
  
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^TURN^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^RIGHT^^^^^^^^^^^^^^^^^^^^^^^^^^  
   
  /*
   *        |-------
   *        |
   *        |
   */

  if(ON_LINE(s[0]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(150);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(100);
    #endif
    
    brake();
    
    /*
     *          |------
     *          |
     *          |
     */
     
     /*
     *          |
     *          |------
     *          |
     */
    turnRight();
  }

  /*
   *      -------|
   *             |
   *             |
   */

//^^^^^^^^^^^^^^^^^^^^^^^^^^TURN^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^LEFT^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  if(ON_LINE(s[7]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(160);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(100);
    #endif
    
    brake();
    qtra.readLine(s);

    /*
     *      -------|
     *             |
     *             |
     */

     if(OFF_LINE(s[3])&&OFF_LINE(s[4]))
     {
       turnLeft();
     } 
     
    /*
     *             |
     *             |
     *      -------|
     *             |
     *             |
     */
     
     else
     {
       path[pathLength] = 'S';
       #ifdef DEBUG
        Serial.println("S");
       #endif
       pathLength++;
       if(path[pathLength-2] == 'B')
         shortPath();
       straight();
     } 
  }
//^^^^^^^^^^^^^^^^^^^^^^^^^^CROSS^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^INTERSECTION^^^^^^^^^^^^^^^^^^^^^^^^
  /*
   *              |
   *        ______|______
   *              |   
   *              |
   */
  //all sensors on black surface
  if(ON_LINE(s[0]) && ON_LINE([7]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(150);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(100);
    #endif
    
    brake();
    qtra.readLine(s);

    /*
     *      --------------
     *      **************
     *      **************
     *      
     */
     
    if(ON_LINE(s[0])&&ON_LINE(s[7])&&ON_LINE(s[3])&&ON_LINE(s[4]))
    {
      done();
    }
    
    /*     _______
     *        |
     *        |
     *        |
     */
     

    /*
     *        |
     *    ____|____
     *        |
     *        |
     */
     
    else
    {
      sharpRight(turnSpeed,turnSpeed/.8);
      turnRight();
    }
  }

//^^^^^^^^^^^^^^^^^^^^^^^^^^TURN^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^AROUND^^^^^^^^^^^^^^^^^^^^^^^^^^

  if(OFF_LINE(s[0])&&OFF_LINE(s[7])&&OFF_LINE(s[3])&&OFF_LINE(s[4]))
  {
    moveForward(maze_left_base_speed,maze_right_base_speed);
    #ifdef LOW_VOLTAGE
    _delay_ms(120);    //150
    #endif

    #ifdef HIGH_VOLTAGE
    _delay_ms(90);
    #endif
    
    brake();
    turnAround();
  }
   
  else pidMaze();
  return;
}

#endif

//##########################################################
void pidMaze()
{
  double Kp = .03;
  double Kd = .2;
  unsigned int position = qtra.readLine(s);
  int error  = position - 3500;
  int pid = Kp * error + Kd * (error - lastError);
  lastError = error;
  int leftMotorSpeed = 55 - pid;
  int rightMotorSpeed = 55 + pid;

  if (leftMotorSpeed > 90)  leftMotorSpeed = 90;
  if (rightMotorSpeed > 90) rightMotorSpeed = 90;
  if (leftMotorSpeed < -90) leftMotorSpeed = -90;
  if (rightMotorSpeed < -90) rightMotorSpeed = -90;

  moveForward(leftMotorSpeed, rightMotorSpeed);
  return;
}
//##########################################################

void turnRight()
{
  while(1)
  {
    sharpRight(turnSpeed,turnSpeed/.8);
    qtra.readLine(s);
    if(ON_LINE(s[1])) break;
  }
  if(replayStage == 0)
  {
    path[pathLength] = 'R';
    #ifdef DEBUG
      Serial.println("R");
    #endif
    pathLength++;
    if(path[pathLength-2] == 'B')
      shortPath();
  }
  pidMaze();
  return;
}

void turnLeft()
{
  while(1)
  {
    sharpLeft(turnSpeed,turnSpeed/.8);
    qtra.readLine(s);
    if(ON_LINE(s[6])) break;
  }
  if(replayStage == 0)
  {
    path[pathLength] = 'L';
    #ifdef DEBUG
      Serial.println("L");
    #endif
    pathLength++;
    if(path[pathLength-2] == 'B')
      shortPath();
  }
  pidMaze();
  return;
}

void straight()
{
  moveForward(maze_left_base_speed,maze_right_base_speed);
  pidMaze();
}

void turnAround()
{
 while(1)
  {
    sharpLeft(turnSpeed,turnSpeed/.8);
    qtra.readLine(s);
    if(ON_LINE(s[3])| ON_LINE(s[4])) break;
  }
  path[pathLength]='B';
  #ifdef DEBUG
    Serial.println("B");
  #endif
  pathLength++;
  
  pidMaze();
    
  return;
}

void shortPath()
{
  bool shortDone=0;
  #ifdef LEFT_HAND_RULE                 //left_hand_rule
    if(path[pathLength-3]=='L' && path[pathLength-1]=='R' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='B';
      shortDone=1;
    }   
    if(path[pathLength-3]=='L' && path[pathLength-1]=='S' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='R';
      shortDone=1;
    }   
    if(path[pathLength-3]=='R' && path[pathLength-1]=='L' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='B';
      shortDone=1;
    }  
    if(path[pathLength-3]=='S' && path[pathLength-1]=='L' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='R';
      shortDone=1;
    }   
    if(path[pathLength-3]=='S' && path[pathLength-1]=='S' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='B';
      shortDone=1;
    }
    if(path[pathLength-3]=='L' && path[pathLength-1]=='L' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='S';
      shortDone=1;
    }
  #endif                                                  //right_hand_rule
  
  #ifdef RIGHT_HAND_RULE
    if(path[pathLength-3]=='L' && path[pathLength-1]=='R' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='B';
      shortDone=1;
    }
    if(path[pathLength-3]=='R' && path[pathLength-1]=='L' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='B';
      shortDone=1;
    }
    if(path[pathLength-3]=='R' && path[pathLength-1]=='S' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='L';
      shortDone=1;
    }   
    if(path[pathLength-3]=='S' && path[pathLength-1]=='R' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='L';
      shortDone=1;
    }   
    if(path[pathLength-3]=='S' && path[pathLength-1]=='S' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='B';
      shortDone=1;
    }
    if(path[pathLength-3]=='R' && path[pathLength-1]=='R' && shortDone==0)
    {
      pathLength-=3;
      path[pathLength]='S';
      shortDone=1;
    }
  #endif
  
  pathLength++;
  
  return;
}

void done()
{
  brake();
  PORTB |= 1 <<PB5;
  _delay_ms(1000);
  replayStage = 1;
  for(int i=0;i<pathLength;i++)
  {
    if(path[i] == 'L')  path[i] = 'R';
    else if(path[i] == 'R') path[i] = 'L';
  }

  for(int i=0;i<pathLength;i++)
  {
    reverse_path[i] = path[pathLength-(i+1)];
  }
  reverse_path[strlen(reverse_path)] = 'D';
  
  #ifdef DEBUG
  for(int i=0;i<strlen(reverse_path);i++)
  {
    Serial.print(reverse_path[i]);
  }
  #endif
  
  PORTB &= ~(1 << PB5);
  while(1)
  {
    moveForward(-40,-40/.8);
    qtra.readLine(s);
    if(OFF_LINE(s[0]) && OFF_LINE(s[7]))  break;
  }
  
  _delay_ms(100);
  
  while(1)
  {
    sharpLeft(turnSpeed,turnSpeed/.8);
    qtra.readLine(s);
    if(ON_LINE(s[6]))  break;
  }
  
  pidMaze();
  _delay_ms(10);
  brake();
  _delay_ms(100);
  
  replay();    
}

void replay()
{
//  checkPattern();

//  if(irSensor == B00011111 | irSensor == B00111111 | irSensor == B01111111 | irSensor == B11111100 | irSensor == B11111000 | irSensor == B11111110 | irSensor == B11111111)
  if(ON_LINE(s[0]) | ON_LINE(s[7]))
  {
    if(reverse_path[readLength] == 'L')
    {
      moveForward(maze_left_base_speed,maze_right_base_speed);
      _delay_ms(100);
      brake();
      turnLeft();
      pidMaze();
    }
    if(reverse_path[readLength] == 'R')
    {
      moveForward(maze_left_base_speed,maze_right_base_speed);
      _delay_ms(100);
      turnRight();
      pidMaze();
    }
    if(reverse_path[readLength] == 'S')
    {
      moveForward(maze_left_base_speed,maze_right_base_speed);
      _delay_ms(200);
    }
    if(reverse_path[readLength] == 'D')
    {
      endMotion();
    }
    readLength++;
  }
  else
    pidMaze();
  
  replay();
}


void endMotion()
{
  brake();
  PORTB &= ~(1 << PB5);
  _delay_ms(500);
  PORTB |= 1 <<PB5;
  _delay_ms(200);
  PORTB &= ~(1 << PB5);
  _delay_ms(200);
  PORTB |= 1 << PB5;
  _delay_ms(500);
  endMotion();
}



#endif
































































//void EEPROM_READ()
//{
//  char c[30];
//  for(int i=0;i<8;i++)
//  {
//    c[i] = EEPROM.read(i);
//    #ifdef DEBUG
//      Serial.print(c[i]);
//    #endif
//  }
//
//  #ifdef DEBUG
//   Serial.println('\n');
//  #endif
//}

//void testSensor()
//{
//  unsigned int position = qtra.readLine(s);
//  for (unsigned char i = 0; i < NUM_SENSORS; i++)
//  {
//    #ifdef DEBUG
//        Serial.print(s[i]);
//        Serial.print('\t');
//    #endif
//  }
//#ifdef DEBUG
//  Serial.println(position);
//#endif
//  delay(250);
//}

//void showMotorSpeed()
//{
//  Serial.print("Left : ");
//  Serial.print("\t");
//  Serial.print(leftMotorSpeed);
//  Serial.print("Right : ");
//  Serial.print("\t");
//  Serial.println(rightMotorSpeed);
//}




