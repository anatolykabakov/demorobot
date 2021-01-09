int JoysticLeftXAxis, JoysticLeftYAxis;
int JoysticRightXAxis, JoysticRightYAxis;
unsigned int  JoysticLeftXAxisRaw = 0;
unsigned int  JoysticLeftYAxisRaw = 0;
unsigned int  JoysticRightXAxisRaw = 0;
unsigned int  JoysticRightYAxisRaw = 0;
int button1State = 0;
int button2State = 0;
int button3State = 0;
int button1PrevState = 0;
int button2PrevState = 0;
int button3PrevState = 0;
int button1Raw = 0;
int button2Raw = 0;
int button3Raw = 0;
int HYDRO_UP = 0;
int HYDRO_DOWN = 0;
int DEBUG = 0;

void setup() {
  Serial.begin(9600); // Default communication rate of the Bluetooth module
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(6, OUTPUT);
  analogWrite(6, 255);
}

void loop() {

  JoysticLeftXAxisRaw = analogRead(A2); // Read Joysticks X-axis
  JoysticLeftYAxisRaw = analogRead(A3); // Read Joysticks Y-axis
  JoysticRightXAxisRaw = analogRead(A0); // Read Joysticks X-axis
  JoysticRightYAxisRaw = analogRead(A1); // Read Joysticks Y-axis
  button1Raw = digitalRead(2);
  button2Raw = digitalRead(3);
  button3Raw = digitalRead(4);
  
  JoysticLeftXAxis = JoysticLeftXAxisRaw/4;
  JoysticLeftYAxis = JoysticLeftYAxisRaw/4;
  JoysticRightXAxis = JoysticRightXAxisRaw/4;
  JoysticRightYAxis = JoysticRightYAxisRaw/4;
  if ((button1Raw == 0) && (button1PrevState == 1)) {
    
    button1State = (1 - button1State);
  }
  button1PrevState = button1Raw;

  if ((button2Raw == 0) && (button2PrevState == 1)) {
    
    button2State = (1 - button2State);
  }
  button2PrevState = button2Raw;

  if ((button3Raw == 0) && (button3PrevState == 1)) {
    
    button3State = (1 - button3State);
  }
  button3PrevState = button3Raw;
  if (DEBUG) {
    Serial.print(JoysticLeftXAxis); // Dividing by 4 for converting from 0 - 1023 to 0 - 256, (1 byte) range
    Serial.print(" ");
    Serial.print(JoysticLeftYAxis);
    Serial.print(" ");
    Serial.print(JoysticRightXAxis); // Dividing by 4 for converting from 0 - 1023 to 0 - 256, (1 byte) range
    Serial.print(" ");
    Serial.print(JoysticRightYAxis);
    Serial.print(" ");
    Serial.print(button1State);
    Serial.print(" ");
    Serial.print(button2State);
    Serial.print(" ");
    Serial.print(button3State);
    Serial.println("");
  } else {
    byte message[]= {0x41, 0x54, 0x2B, 0x4D, 0x45, 0x53, 0x48, 0x00, 0xFF, 0xFF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x13, 0x0D, 0x0A};
    message[10] = JoysticLeftXAxis;
    message[11] = JoysticLeftYAxis;
    message[12] = JoysticRightXAxis;
    message[13] = JoysticRightYAxis;
    message[14] = button1State;
    message[15] = button2State;
    message[16] = button3State;
    Serial.write(message, sizeof(message));
    //Serial.write('\r\n'); //  Carriage Return
//    Serial.println('AT+MESH0FFFF11223344', HEX);
//    for (int i=0;i<sizeof(message);i++) {
//      Serial.print(message[i], HEX);
//    }
//    Serial.println("41542B4D45534800FFFF112233440D0A", HEX);
//    Serial.print(message, HEX);
//    Serial.write(JoysticLeftXAxis); // Dividing by 4 for converting from 0 - 1023 to 0 - 256, (1 byte) range
//    Serial.write(JoysticLeftYAxis);
//    Serial.write(JoysticRightXAxis); // Dividing by 4 for converting from 0 - 1023 to 0 - 256, (1 byte) range
//    Serial.write(JoysticRightYAxis);
//    Serial.write(button1State);
//    Serial.write(button2State);
//    Serial.write(button3State);
//    Serial.write(13);
  }
  delay(50);
}
