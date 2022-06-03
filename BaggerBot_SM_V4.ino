/* FILENAME: BaggerBot_SM_V4.ino
 * AUTHORS:  BaggerBot Team
 * DATE:     2022/5/11
 * VERSION:  v4
 * DETAIL:   
 *  Program for bag feeding system for Golden Gate Mechanical's BaggerBot to help 
 *  automate the sandbag filling process, for final prototype produced by the 
 *  2021-2022 team. 
 *  Controls 5 solenoid valves (2 sets of vacuum cups, 3 pneumatic cylinders), 
 *  takes input from 2 sensors (2 limit switches). 
 *  
 *  Based on code provided by Skyentific - https://www.youtube.com/watch?v=UWc_7-8gXeE
 *  CAN-BUS Shield Library by Seeed Studio - 
 *        https://github.com/Seeed-Studio/Seeed_Arduino_CAN/releases/tag/v1.3.0
 */

// Include libraries
#include <mcp_can.h> // CAN-Bus Shield v1.3.0 by Seeed Studio
#include <SPI.h> 

// Set to 1 for additional debug information (See serial monitor)
int DEBUG = 1;

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

// Define pins
// Arm Motor (SPI)
#define PIN_SCK  13       // 
#define PIN_MOSI 12       // 
#define PIN_MISO 11       // 
#define PIN_SS   10       // 
// Vacuums
#define PIN_TOP_VAC 4 // Relay in1
#define PIN_BOT_VAC 5 // Relay in2
// Actuators
#define PIN_HORIZ   8 // Relay in5
#define PIN_VERT    7 // Relay in4
#define PIN_FINGERS 6 // Relay in3
// Sensors
#define PIN_VERT_SWITCH 9 // Head
#define PIN_USER_SWITCH 3 // Electronics enclosure
//#define PIN_USER_SWITCH A1

// Joystick on CAN-Bus Shield
//#define UP     A1
#define DOWN   A3
#define LEFT   A2
#define RIGHT  A5
#define CLICK  A4

// Define arm parameters 
// Value Limits
#define P_MIN -12.5f //CCW
#define P_MAX 12.5f //CW
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Arm rotation positions
//    Relative to looking at back of motor...
//    (-) = CCW
//    (+) = CW
// 90*3.14/180 = 1.57
// 10*3.14/180 = 0.17444 = 0.17
#define P_MIN_DESIRED -1.80f //
#define P_MAX_DESIRED -0.01f // originally -0.01
float ACT_LO  = P_MAX_DESIRED; 
float ACT_HI  = ACT_LO - 1.719;  // ~98.5 degrees
float PASS_HI  = ACT_HI - 0.131; // ~7.5 degrees
float PASS_LO  = ACT_LO - 0.55;  // originally -0.43 ~24.6 degrees
// Set initial input motor values
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 50.0f;
float kd_in = 2.0f;
float t_in = 0.0f;
// output (measured) values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;
// Rotation step size
float p_step = 0.0015;

// Create CAN-Bus interface object
MCP_CAN CAN(PIN_SS);

// Define time variables for non-blocking code
const unsigned long PRINT_INTERVAL = 150;
  // PRINT_INTERVAL defines how often debug (via serial) text will print
  //   Tiny interval may affect speed of program (especially the motor)
unsigned long previous_print_time = 0;        

const unsigned long PICK_UP_DELAY = 100;    
  // timer before state DRAGGING = delay once head's limit switch reaches bag, allows adequate suction
const unsigned long HORIZONTAL_TIME = 500;
  // timer before state PRESSING = time for full horizontal actuation
const unsigned long PRESS_TIME = 200;       
  // timer before state OPENING = time for vertical actuator to reach bottom vacuum cups
// const unsigned long OPEN_DELAY = 200;
  // timer within state OPENING = let bottom vacuum cups suction for a moment        
const unsigned long ARM_GRAB_TIME = 500;
  // timer before state GRABBING = keep both sets of vacuum cups on before inserting fingers    
const unsigned long ARM_TENSION_TIME = 400; // 
const unsigned long ARM_PULL_TIME = 350;    // 
const unsigned long ARM_RELEASE_TIME = 500; // 
const unsigned long REPEAT_TIME = 1000;     // 

unsigned long vertical_time;                // 
unsigned long vertical_initial_time;        // 
unsigned long vertical_finish_time;         // 
//float VERTICAL_RETRACT_FACTOR = 1.1;


//unsigned long previous_toggle_time = 0;
//unsigned long TOGGLE_TIME = 1000;

unsigned long previous_time = 0;
unsigned long current_time;
unsigned long running_time = 0;


// Baggerbot state machine variables
const int STATE_SIZE = 10;
int state_current = 0;
int state_previous = 0;
String state[STATE_SIZE] = {"LOADING","LOOKING","PICKING UP","DRAGGING",
                            "PRESSING","OPENING","GRABBING","TENSIONING",
                            "PULLING","RELEASING",};
int state_holder = 0;

// Define prototypes
int getBaggerState();
void RotateRight();
void RotateLeft();

void setup() {
  if (DEBUG)
  {
    Serial.begin(115200);
    Serial.println("Debugging: ON");
  }
  while (CAN_OK != CAN.begin(CAN_1000KBPS)) // can bus : baudrate = 1000kbps (AKA 1Mbps)
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
  }
  SERIAL.println("CAN BUS Shield init ok!");

  //// OPTIONAL: Joystick set up
  // pinMode(UP,INPUT);
  // pinMode(DOWN,INPUT);
  // pinMode(LEFT,INPUT);
  // pinMode(RIGHT,INPUT);
  // pinMode(CLICK,INPUT);
  //// Pull analog pins high to enable reading of joystick movements
  // digitalWrite(UP, HIGH);
  // digitalWrite(DOWN, HIGH);
  // digitalWrite(LEFT, HIGH);
  // digitalWrite(RIGHT, HIGH);
  // digitalWrite(CLICK, HIGH);
  
  // Begin vacuum system set up here
  pinMode(PIN_TOP_VAC,OUTPUT);
  pinMode(PIN_BOT_VAC,OUTPUT);

  // Begin actuator set up here
  pinMode(PIN_HORIZ, OUTPUT);
  pinMode(PIN_VERT, OUTPUT);
  pinMode(PIN_FINGERS,OUTPUT);

  // Begin sensor set up here
  pinMode(PIN_VERT_SWITCH,INPUT_PULLUP);
  pinMode(PIN_USER_SWITCH,INPUT);
  
  // Write Solenoid pins HIGH to turn them off by default (NC)
  digitalWrite(PIN_TOP_VAC,HIGH);
  digitalWrite(PIN_BOT_VAC,HIGH);
  digitalWrite(PIN_HORIZ,  HIGH);
  digitalWrite(PIN_VERT,   HIGH);
  digitalWrite(PIN_FINGERS,HIGH);
  
  // Limit switch on head is unpressed when LOW (NO)
  digitalWrite(PIN_VERT_SWITCH, LOW);
  
  // Turn off motor on start up
  ExitMotorMode();
  
}

void loop() {
  current_time = millis();
  state_holder = getBaggerState();
  
  if (DEBUG && (current_time - previous_print_time >= PRINT_INTERVAL))
  {
//    running_time += PRINT_INTERVAL;
//    Serial.print("Time elapsed: ");
//    Serial.print(running_time);
    Serial.print(" Bagger State: ");
    Serial.print(state_holder);
    Serial.print(" (");
    Serial.print(state[state_holder]); 
    Serial.print(") || Limit Switch: ");
    Serial.print(digitalRead(PIN_VERT_SWITCH));
    Serial.print(" || User Toggle: ");    
    Serial.println(digitalRead(PIN_USER_SWITCH));
    previous_print_time = current_time;
//    if(running_time >= 999999) {
//      running_time = 0;
//    }
  }

//  if ( (state_current != 0) && digitalRead(PIN_USER_SWITCH) == HIGH)
//  if ( (state_current != 0) && digitalRead(PIN_USER_SWITCH) == LOW)
//    {
//    //    delay(5000);
//    //    Serial.println("SOMETHING WENT WRONG");
//    //    digitalWrite(PIN_BOT_VAC,HIGH);
//    //    delay(1000);
//    //    digitalWrite(PIN_BOT_VAC,LOW);
//    //    delay(1000);
//    //    digitalWrite(PIN_BOT_VAC,HIGH);
//    //    delay(1000);
//    //    digitalWrite(PIN_BOT_VAC,LOW);
//    //    delay(1000);
//    //    digitalWrite(PIN_BOT_VAC,HIGH);
//    //    delay(1000);
//    //    digitalWrite(PIN_BOT_VAC,LOW);
//      delay(1000);
//      
//     state_current = 0; // Possible to immediately go to state 0 again after going from 0 to 1
//     }

}

/* 
 * Function: getBaggerState()
 * Inputs: None
 * Returns: state_HE - Integer representation of the state the 
 *                     state machine is currently in. 
 * Description: Considers and returns the current state the machine 
 *              is in. As follows:
 *              0:  LOADING
 *              1:  LOOKING
 *              2:  PICKING UP
 *              3:  DRAGGING
 *              4:  PRESSING
 *              5:  OPENING
 *              6:  GRABBING
 *              7:  TENSIONING
 *              8:  PULLING
 *              9:  RELEASING
*/
int getBaggerState()
{
  state_previous = state_current;
  switch (state_current)
  {
    /* State: LOADING: 
     * Purpose: Await user input while cassette is loaded
     *          Retract all pneumatic cylinders, arm to passive low
     * Transition: from 9 - timer REPEAT_TIME finish
     *               to 1 - PIN_USER_SWITCH is ON (LOW)
     */
    case 0:
    {
      // Turn off all solenoids
      digitalWrite(PIN_TOP_VAC,HIGH);
      digitalWrite(PIN_BOT_VAC,HIGH);
      digitalWrite(PIN_HORIZ,  HIGH);
      digitalWrite(PIN_VERT,   HIGH);
      digitalWrite(PIN_FINGERS,HIGH);
      
      if (digitalRead(PIN_USER_SWITCH) == LOW)
        {
          state_current = 1;
          vertical_initial_time = millis();
          // Make arm passive low
          EnterMotorMode();
        }
      
      return state_current;
    }
    
    /* State: LOOKING: Vertical actuation - looking/moving on top of bag stack
     * Purpose: 
     * Transition: 
     */
    case 1:
    {
      
      digitalWrite(PIN_VERT, LOW);
      digitalWrite(PIN_FINGERS, HIGH);

      // Make arm passive low
      if(p_out < PASS_LO) {
        RotateRight(PASS_LO);
      }
      else if(p_out > PASS_LO) {
        RotateLeft(PASS_LO);
      }
      
      if (digitalRead(PIN_VERT_SWITCH) == HIGH)
        {
          Serial.println("PRESSED");
          state_current = 2;
          vertical_finish_time = millis();
          vertical_time = vertical_finish_time - vertical_initial_time; 
          // Variable "vertical_time" defines the duration required 
          // to actuate to the top of the bag stack. This is 
          // stored to determine how long the vertical actuator will
          // take to fully retract (pick the bag up).
        }
      
      return state_current;
    }
    
    /* State: PICKING UP: Top vacuum enabling - picking up top bag
     * Purpose: 
     * Transition: 
     */
    case 2:
    {
      digitalWrite(PIN_TOP_VAC, LOW);
      
      if(millis() - vertical_finish_time >= PICK_UP_DELAY)  //MIGHT NOT NEED ANYMORE (due to better cups)
        {
          digitalWrite(PIN_VERT, HIGH);  
        }
      if(millis() - vertical_finish_time >= vertical_time + PICK_UP_DELAY)
        {
          state_current = 3;
          previous_time = millis();
        }
      return state_current;
    }
    
    /* State: DRAGGING: Horizontal actuation - dragging bag towards bottom vacuum
     * Purpose: 
     * Transition: 
     */
    case 3:
    {
      digitalWrite(PIN_HORIZ, LOW);
      
      if(millis() - previous_time >= HORIZONTAL_TIME)
      {
        state_current = 4;
        previous_time = millis();
      }
      return state_current;
    }
    
    /* State: PRESSING: Vertical actuation - pressing bag onto bottom vacuums
     * Purpose: 
     * Transition: 
     */
    case 4:
    {
      digitalWrite(PIN_VERT, LOW);
      
//      if (digitalRead(PIN_VERT_SWITCH) == HIGH)   LIMIT SWITCH NO LONGER REACHES BOTTOM VAC
//        {
//          state_current = 5;
//          previous_time = millis();
//        }

      if(millis() - previous_time >= PRESS_TIME)
      {
        digitalWrite(PIN_BOT_VAC, LOW);
        digitalWrite(PIN_VERT, HIGH);
        state_current = 5;
        previous_time = millis();
      }
      
      return state_current;
    }

    /* State: OPENING: Bottom vacuum enabling - opening bag
     * Purpose: 
     * Transition: 
     */
    case 5:
    { 
//      if(millis() - previous_time >= OPEN_DELAY)    LIMIT SWITCH NO LONGER REACHES BOT VAC
//          {
//            digitalWrite(PIN_VERT, HIGH);
//          }
      
      if(millis() - previous_time >= ARM_GRAB_TIME)
        {
          state_current = 6;
          previous_time = millis();
          p_step = 0.0005; // SLOW DOWN SPEED OF ARM
          RotateRight(ACT_LO);
        }
        
      return state_current;
    }

    /* State: GRABBING: Arms active low - initiate grabbing bag
     * Purpose: 
     * Transition: 
     */
    case 6:
    {
      // ARM ACTIVE LOW HERE
//      RotateRight(ACT_LO);            MOVED TO CASE 5 EVENT 

      if(millis() - previous_time >= ARM_TENSION_TIME)
      {
        state_current = 7;
        previous_time = millis();
        p_step = 0.0015;
        digitalWrite(PIN_FINGERS, LOW);
        digitalWrite(PIN_TOP_VAC, HIGH);
        digitalWrite(PIN_BOT_VAC, HIGH);
      }    
      
      return state_current;
    }

    /* State: TENSIONING: Arm actuation - tensioning bag onto fingers
     * Purpose: 
     * Transition: 
     */   
    case 7:
    {
      if(millis() - previous_time >= ARM_PULL_TIME)
        {
          state_current = 8;
          previous_time = millis();
          RotateLeft(ACT_HI);
        }    
      
      return state_current;
    }

    /* State: PULLING: Arm active high - pulling bag out
     * Purpose: 
     * Transition: 
     */
    case 8:
    {
      digitalWrite(PIN_HORIZ, HIGH);
      
      // ARM ACTIVE HIGH HERE
//      RotateLeft(ACT_HI);               MOVED TO CASE 7 EVENT

      if(millis() - previous_time >= ARM_RELEASE_TIME)
        {
          state_current = 9;
          previous_time = millis();
          RotateLeft(PASS_HI);
        }   
      
      return state_current;
    }

    /* State: RELEASING: Arm passive high - releasing bag
     * Purpose: 
     * Transition: 
     */
    case 9:
    {
      // ARM PASSIVE HIGH HERE
//      RotateLeft(PASS_HI);

//      if(analogRead(
      if(millis() - previous_time >= REPEAT_TIME)
        {
          state_current = 1;
          vertical_initial_time = millis();
        }   

      return state_current;
    }
    
  }
  
}

void printPosVel() {
  SERIAL.print(" ");
  SERIAL.print(p_in);
  SERIAL.print(" ");
  SERIAL.print(p_out); 
  SERIAL.print(" "); 
  SERIAL.println(v_out);
}

/* Function:  RotateRight
 * Description: Rotates arm motor CW (clockwise), relative to facing back of motor, until 
 *              the desired input position 'p_destination' is sent to the motor controller
 *              & any response from the motor controller is received.
 *              Rotation speed depends on 'p_step'. Blocking.
 */
void RotateRight(float p_destination){
  p_destination = constrain(p_destination, P_MIN, P_MAX);
  
  while (p_in < p_destination) {
    p_in = p_in + p_step;
    // send CAN
    pack_cmd();
    // receive CAN
    if(CAN_MSGAVAIL == CAN.checkReceive()) // check if data coming
    {
      unpack_reply();
    }
  }
  
  if(DEBUG) {
    //printPosVel();
  }
}

/* Function:  RotateRight
 * Description: Same as 'RotateLeft()' but CCW (counter-clockwise).
 *              Rotation speed depends on 'p_step'. Blocking.
 */
void RotateLeft(float p_destination){
  p_destination = constrain(p_destination, P_MIN, P_MAX);
  while (p_in > p_destination){
    p_in = p_in - p_step;
    // send CAN
    pack_cmd();
    // receive CAN
    if(CAN_MSGAVAIL == CAN.checkReceive()) // check if data coming
    {
      unpack_reply();
    }
  }

  if(DEBUG) {
    //printPosVel();
  }
}

void EnterMotorMode(){
  // Enter Motor Mode (enable)
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFC;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

void ExitMotorMode(){
  // Exit Motor Mode (disable)
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFD;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

void Zero(){
  // Set the current position as the new zero position
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFE;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

void pack_cmd(){
  byte buf[8];

  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, between -30 and + 30 rad/s
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]]
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  /// limit data to be within bounds ///
  float p_des = constrain(p_in, P_MIN, P_MAX); //fminf(fmaxf(P_MIN, p_in), P_MAX);                    
  float v_des = constrain(v_in, V_MIN, V_MAX); //fminf(fmaxf(V_MIN, v_in), V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX); //fminf(fmaxf(KP_MIN, kp_in), KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX); //fminf(fmaxf(KD_MIN, kd_in), KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX); //fminf(fmaxf(T_MIN, t_in), T_MAX);
  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  /// pack ints into the can buffer ///

  buf[0] = p_int >> 8;                                       
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}
 
void unpack_reply(){

  /// CAN Reply Packet Structure ///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and + 30 rad/s
  /// 12 bit current, between -40 and 40;
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]

  byte len = 0;
  byte buf[8];
  CAN.readMsgBuf(&len, buf);

  unsigned long canId = CAN.getCanId();

  /// unpack ints from can buffer ///
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];
  /// convert uints to floats ///
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
} 

unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if (bits == 16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}
