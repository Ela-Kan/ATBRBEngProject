/* ATBR Control for 3D robot - i.e. two ground wires and two live wires per SMA turn. Section 1 refers to the base of the robot, 
 * and Section 10 to the tip. Refer to dissertation document for the appropriate wiring diagram/how activating specific sections 
 * results in specific sectional bending, as well as the specific labelling for the wires (i.e. G1, G2, V1...V10, V11..V20).
 * REMEMBER: sections are activated in the opposite direction/towards the inside of the coil, rather than away from the centre
 */

 // By Ela Kanani 

 /* Relay Pin Arrangement:
  *  PIN 22: Ground 1 (G1), lies in +x,+y quadrant
  *  PIN 23: Ground 2 (G2), lies in -x,-y quadrant
  *  PIN 24-33: V1...V10 live wires, lie in +x,-y quadrant
  *  PIN 34-43: V11...V20 live wires, lie in -x,+y quadrant
  */ 

// Storing all relay pins
int relay[22] = {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43};

// Relays to activate if user wants to move in given directions
// To move in the negative X direction:
int negX_live_wires[10] = {24,25,26,27,28,29,30,31,32,33}; //activate V1...V10 dependent on section
int negX_gnd_wire = 22; //activate G1

// To move in positive X direction:
int posX_live_wires[10] = {34,35,36,37,38,39,40,41,42,43}; //activate V11...V20 dependent on section
int posX_gnd_wire = 23; //activate G2

// To move in negative Y direction:
int negY_live_wires[10] = {34,35,36,37,38,39,40,41,42,43}; //activate V11...V20 dependent on section
int negY_gnd_wire = 22; //activate G1

// To move in positive Y direction:
int posY_live_wires[10] = {24,25,26,27,28,29,30,31,32,33}; //activate V1...V10 dependent on section;
int posY_gnd_wire = 23; //activate G2

/* assign pins for BTS7960 H-Bridge
 *  LPWM,RPWM, LEN and REN (latter control power values)
 *  PIN 5: LPWM
 *  PIN 6: RPWM
 *  PIN 9: LEN
 *  PIN 10: REN
*/

int LPWM = 5; 
int RPWM = 6; 
int LEN = 9; 
int REN = 10;

/* declare the user inputs: 
 * active_direction = do we want to move in the positive (+) direction or negative (-) direction in reference to the axes
 * axis = to activate the robot in the x or y axis
 * section = section to activate (value between 1-10), 1 is the first base section and 10 is the last
 * activation_time= (in ms)  how long to send pwm signal for (i.e. how long to activate the section for).
 * pwm_power = the power sent to the section
*/
char active_direction, axis;
int section, activation_time, pwm_power;


// Functions ----------------------------------------------------------------------------------------------------------------------
/* Function to turn off all relays in the system.
 * Input: the array of ints containing the relay pins
 */
void TurnOffRelays(int* relay){
  for(int i = 0; i < 22; i++){
    digitalWrite(relay[i], HIGH); // Relay commands are 'reversed!'
  }
}

/* Function to turn ON PWM (BTS7960)
 * Input: PWM power value and relevant pins
 */
void TurnOnPWM(int pwm_power,int LPWM, int RPWM, int LEN, int REN){
  digitalWrite(RPWM,HIGH);   
  digitalWrite(LPWM, LOW);
  analogWrite(REN, pwm_power);
  analogWrite(LEN, pwm_power);
  }


/* Function to turn OFF PWM (BTS7960) 
* Input: relevant pins
*/
void TurnOffPWM(int LPWM, int RPWM, int LEN, int REN){
  digitalWrite(RPWM, LOW);   
  digitalWrite(LPWM, LOW);
  analogWrite(REN, 0);
  analogWrite(LEN, 0);
}

// Main Script ---------------------------------------------------------------------------------------------------------------------

void setup() {
  // start the serial monitor
  Serial.begin(9600);
  
  // RELAY SETUP 
  // initialise the relays as outputs and ensure they are inactive (off)
  for(int i = 0; i < 22; i++)
  {
    pinMode(relay[i], OUTPUT);
  }
  TurnOffRelays(relay);
  
  // BTS7960 H-BRIDGE SETUP 
  // initialise output pins
  pinMode(LPWM, OUTPUT); 
  pinMode(RPWM, OUTPUT); 
  pinMode(LEN, OUTPUT);
  pinMode(REN, OUTPUT);
  // ensure that PWM BTS7960 is off
  TurnOffPWM(LPWM,RPWM,LEN,REN);
  
  // explain inputs to user
  Serial.println("Enter direction(+/-), axis(x/y), section(1-10), activation time (ms) and");
  Serial.println("PWM power separated by commas, followed by the enter key: (e.g. +,x,5,3000,20)");
  //e.g. +,x,5,3000,20 means activate section 5 in the positive(+) x direction for 3 seconds at a power of 20.
} //void setup()

void loop() {
      while (Serial.available()==0){} // Do nothing unless there is a user input
      // read input string in the serial monitor until enter key is pressed
      String userInput = Serial.readStringUntil('\n');
      // parse userInput into desired variables
      sscanf(userInput.c_str(), "%c,%c,%i,%i,%i", &active_direction, &axis, &section, &activation_time, &pwm_power);
      // TO CONFIRM INPUTS 
      Serial.println("Received Instructions:");
      Serial.println((String)"Direction: "+active_direction); //display same received Data back in serial monitor.
      Serial.println((String)"Axis: "+axis); //display same received Data back in serial monitor.
      Serial.println((String)"Activation section: "+section); //display same received Data back in serial monitor.
      Serial.println((String)"Activation time (ms): "+activation_time); //display same received Data back in serial monitor.
      Serial.println((String)"PWM value: "+pwm_power); //display same received Data back in serial monitor.
      Serial.println();
   
      /* Perform desired activation. The section corresponds to the x/y live wire relay array index. 
       *  E.g. -x section 5 would need to activate negX_live_wires[5-1] and negX_gnd_wire */
      // If user wants to move in -x direction
      if ((active_direction =='-')&&(axis == 'x')){
         // Turn on corresponding ground wire relay
         digitalWrite(negX_gnd_wire, LOW);
         // Turn on corresponding live wire relay for the desired section 
         digitalWrite(negX_live_wires[section-1],LOW);
         // turn on PWM power
         TurnOnPWM(pwm_power,LPWM,RPWM,LEN,REN);
         //TURN EVERYTHING OFF AFTER ACTIVATION
         //only keep relay on for activation time
         delay(activation_time);
         // switch off all relays
         TurnOffRelays(relay);
         // switch off PWM power (BTS7960)
         TurnOffPWM(LPWM,RPWM, LEN, REN);
      } //-x
      
      // If user wants to move in +x direction
      if ((active_direction =='+')&&(axis == 'x')){
         // Turn on corresponding ground wire relay
         digitalWrite(posX_gnd_wire, LOW);
         // Turn on corresponding live wire relay for the desired section
         digitalWrite(posX_live_wires[section-1],LOW);
         // turn on PWM power
         TurnOnPWM(pwm_power,LPWM,RPWM,LEN,REN);
         //TURN EVERYTHING OFF AFTER ACTIVATION
         //only keep relay on for activation time
         delay(activation_time);
         // switch off all relays
         TurnOffRelays(relay);
         // switch off PWM power (BTS7960)
         TurnOffPWM(LPWM,RPWM, LEN, REN);
      } //+x

      // If user wants to move in -y direction
      if ((active_direction =='-')&&(axis == 'y')){
         // Turn on corresponding ground wire relay
         digitalWrite(negY_gnd_wire, LOW);
         // Turn on corresponding live wire relay for the desired section
         digitalWrite(negY_live_wires[section-1],LOW);
         // turn on PWM power
         TurnOnPWM(pwm_power,LPWM,RPWM,LEN,REN);
         //TURN EVERYTHING OFF AFTER ACTIVATION
         //only keep relay on for activation time
         delay(activation_time);
         // switch off all relays
         TurnOffRelays(relay);
         // switch off PWM power (BTS7960)
         TurnOffPWM(LPWM,RPWM, LEN, REN);
      } //-y
    
      // If user wants to move in +y direction
      if ((active_direction =='+')&&(axis == 'y')){
         // Turn on corresponding ground wire relay
         digitalWrite(posY_gnd_wire, LOW);
         // Turn on corresponding live wire relay for the desired section
         digitalWrite(posY_live_wires[section-1],LOW);
         // turn on PWM power
         TurnOnPWM(pwm_power,LPWM,RPWM,LEN,REN);
         //TURN EVERYTHING OFF AFTER ACTIVATION
         //only keep relay on for activation time
         delay(activation_time);
         // switch off all relays
         TurnOffRelays(relay);
         // switch off PWM power (BTS7960)
         TurnOffPWM(LPWM,RPWM, LEN, REN);
      } //+y

      //Check if user wishes to switch everything off and exit the code
      //TerminationCheck(LPWM,RPWM,LEN,REN,relay);
    
} // void loop()
