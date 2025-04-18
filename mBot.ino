#include "MeMCore.h"
#define RIGHT_TURNING_TIME_MS 350  // The time duration (ms) for left turning
#define LEFT_TURNING_TIME_MS 310   // The time duration (ms) for right turning
#define PAUSE 50                   // The time duration (ms) for mBot to pause after turning such that it gets time to settle down and not overturn
#define TIMEOUT 2000               // Max microseconds to wait for ultrasonic sensor to timeout; choose according to max distance of wall
#define SPEED_OF_SOUND 340
#define ONE_BLOCK_MOVING_TIME 700  // The time duration (ms) for moving when doing a double turn
#define RGBWait 150                // The time duration (ms) for LED to stabilise for accurate reading
#define LDRWait 10                 // The time duration (ms) for sensor to stabilise for accurate reading

#define IRREADER A3
#define LDRREADER A2
#define ULTRASONIC 12
#define CONTROL1 A0
#define CONTROL2 A1

MeRGBLed led(0, 30);  // Based on hardware connections on mCore; cannot change
MeBuzzer buzzer;

MeLineFollower lineFinder(PORT_2);  // Assigning lineFinder to RJ25 port 2
int status = 0;                     // Global status; 0 = do nothing, 1 = mBot runs

MeDCMotor leftMotor(M1);   // Assigning leftMotor to port M1
MeDCMotor rightMotor(M2);  // Assigning RightMotor to port M2

uint8_t motorSpeed_Left = 255;   // Left wheel speed tune to ensure both wheel spin at the same speed
uint8_t motorSpeed_Right = 220;  // Right wheel speed tune to ensure both wheel spin at the same speed


float whiteArray[] = { 955, 989.00, 975.00 };
float blackArray[] = { 686, 906.00, 813.00 };
float greyDiff[] = { whiteArray[0] - blackArray[0],
                     whiteArray[1] - blackArray[1],
                     whiteArray[2] - blackArray[2] };
float colourArray[] = { 0, 0, 0 };
float colourStorage[6][3] = { { 255.95, 273.93, 256.57 },    // White
                              { 255, 165, 118.63 },          // Red
                              { 178.22, 212.00, 187.94 },    // Green
                              { 180, 216.75, 250.91 },       // Blue
                              { 256.90, 244.00, 234.73 },    // Pink
                              { 255.00, 202.77, 110.19 } };  // Orange

char colourList[6][7] = { "White", "Red", "Green", "Blue", "Pink", "Orange" };

void set_decoder(int n) {
  if (n == 0) {  //Shine only Red LED
    analogWrite(A0, 255);
    analogWrite(A1, 255);
  } else if (n == 1) {  //Shine only Green LED
    analogWrite(A0, 0);
    analogWrite(A1, 255);
  } else if (n == 2) {  //Shine only Blue LED
    analogWrite(A0, 255);
    analogWrite(A1, 0);
  } else {  //Set IR emiiter to ON
    analogWrite(A0, 0);
    analogWrite(A1, 0);
  }
}

void setBalance() {
  // Set white Array
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);                   // Delay for 5 seconds to get ready
  for (int i = 0; i < 3; i++) {  // Shine Red, Green, Blue one at a time to get each reading
    set_decoder(i);
    delay(RGBWait);
    whiteArray[i] = getAvgReading(10);
    set_decoder(3);
    Serial.println(whiteArray[i]);
    delay(RGBWait);
  }

  // Set black Array
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);                   // Delay for 5 seconds to get ready
  for (int i = 0; i < 3; i++) {  // Shine Red, Green, Blue one at a time to get each reading
    set_decoder(i);
    delay(RGBWait);
    blackArray[i] = getAvgReading(10);
    set_decoder(3);
    Serial.println(blackArray[i]);
    delay(RGBWait);
    greyDiff[i] = whiteArray[i] - blackArray[i];  // The difference between the maximum and the minimum gives the range
  }

  Serial.println("Colour Sensor Is Ready.");
  delay(5000);

  // Get reading for each colour sample
  for (int j = 0; j < 6; j += 1) {
    Serial.print("Put colour ");
    Serial.println(colourList[j]);
    delay(5000);
    for (int i = 0; i < 3; i += 1) {  // Shine Red, Green, Blue one at a time to get each reading
      set_decoder(i);
      delay(RGBWait);
      colourStorage[j][i] = getAvgReading(10);
      colourStorage[j][i] = ((colourStorage[j][i] - blackArray[i]) / (greyDiff[i])) * 255;
      set_decoder(3);
      delay(RGBWait);
    }
    Serial.println("colour Array: ");
    for (int i = 0; i < 3; i += 1) {
      Serial.println(colourStorage[j][i]);  // Print out the colour for recording
    }
  }
}

int getAvgReading(int times)  // Find the average reading for the requested number of times of scanning LDR
{
  int reading;
  int total = 0;
  for (int i = 0; i < times; i++) {  // Take the reading as many times as requested and add them up
    reading = analogRead(LDRREADER);
    total = reading + total;
    delay(LDRWait);
  }
  return total / times;  //calculate the average and return it
}

void MOVE_FORWARD()  // Code for moving forward
{
  leftMotor.run(-motorSpeed_Left);   // Negative: wheel turns anti-clockwise
  rightMotor.run(motorSpeed_Right);  // Positive: wheel turns clockwise
}

void STOP_MOTOR()  // Code to stop moving
{
  leftMotor.stop();   // Stop left motor
  rightMotor.stop();  // Stop right motor
}


void TURN_LEFT()  // Code for left turn -- colour Red
{
  leftMotor.run(motorSpeed_Left);    // Positive: wheel turns clockwise
  rightMotor.run(motorSpeed_Right);  // Positive: wheel turns clockwise
  delay(LEFT_TURNING_TIME_MS);       // Keep turning left for this time duration
  STOP_MOTOR();                      // Stop motor
  delay(PAUSE);                      // Pause for mBot to stabilise
}

void TURN_RIGHT()  // Code for right turn -- colour Green
{
  leftMotor.run(-motorSpeed_Left);    // Positive: wheel turns clockwise
  rightMotor.run(-motorSpeed_Right);  // Positive: wheel turns clockwise
  delay(RIGHT_TURNING_TIME_MS);       // Keep turning right for this time duration
  STOP_MOTOR();                       // Stop motor
  delay(PAUSE);                       // Pause for mBot to stabilise
}

void TURN_RIGHT_RIGHT()  // Code for double turn right -- colour blue
{
  TURN_RIGHT();    // Turn right
  MOVE_FORWARD();  // Move forward
  delay(750);      // Keep going straight for a specific time (one block length)
  STOP_MOTOR();    // Stop motor
  delay(PAUSE);    // Pause for mBot to stabilise
  TURN_RIGHT();    // Turn right
}

void TURN_LEFT_LEFT()  // Code for double turn left -- colour pink
{
  TURN_LEFT();                   // Turn left
  MOVE_FORWARD();                // Move forward
  delay(ONE_BLOCK_MOVING_TIME);  // Keep going straight for a specific time (one block length)
  STOP_MOTOR();                  // Stop motor
  delay(PAUSE);                  // Pause for mBot to stabilise
  TURN_LEFT();
}

void TURN_BACK_LEFT()  // Code for turning back in the left direction -- colour Orange
{
  TURN_LEFT();   // Turn left
  delay(PAUSE);  // Pause for mBot to stabilise
  TURN_LEFT();   // Turn left
}

void TURN_BACK_RIGHT()  // Code for turning back in the right direction -- colour Orange
{
  TURN_RIGHT();  // Turn right
  delay(PAUSE);  // Pause for mBot to stabilise
  TURN_RIGHT();  // Turn right
}

void ADJUST_RIGHT() {
  leftMotor.run(-motorSpeed_Left);       // Negative: wheel turns anti-clockwise
  rightMotor.run(motorSpeed_Right / 2);  // Positive: wheel turns clockwise at half speed
}

void ADJUST_LEFT() {
  leftMotor.run(-motorSpeed_Left / 2);  // Negative: wheel turns anti-clockwise at halfspeed
  rightMotor.run(motorSpeed_Right);     // Positive: wheel turns clockwise
}

void celebrate() {  // Code for playing celebratory music
  buzzer.tone(392, 200);
  buzzer.tone(523, 200);
  buzzer.tone(659, 200);
  buzzer.tone(784, 200);
  buzzer.tone(659, 150);
  buzzer.tone(784, 400);
  buzzer.noTone();
}

void identify_colour(int n) {     // Function to dictate the mBot movement based on colour detected
  if (n == 0) {                   // If colour white is detected
    led.setColor(255, 255, 255);  // Set colour to white
    led.show();
    leftMotor.run(0);         // Stop motor
    rightMotor.run(0);        // Stop motor
    celebrate();              // Play music
    status = 0;               // Turn off mBot
  } else if (n == 1) {        // If colour red is detected
    led.setColor(255, 0, 0);  // Set colour to red
    led.show();
    TURN_LEFT();              // Turn left
  } else if (n == 2) {        // If colour green is detected
    led.setColor(0, 255, 0);  // Set colour to green
    led.show();
    TURN_RIGHT();             // Turn right
  } else if (n == 3) {        // If colour blue is detected
    led.setColor(0, 0, 255);  // Set colour to blue
    led.show();
    TURN_RIGHT_RIGHT();           // Double right turn
  } else if (n == 4) {            // If colour pink is detected
    led.setColor(201, 169, 166);  // Set colour to pink
    led.show();
    TURN_LEFT_LEFT();           // Double left turn
  } else {                      // If colour orange is detected
    led.setColor(255, 165, 0);  // Set colour to orange
    led.show();

    /*
    Detected if left wall or right wall is too close
    Turn left 180 if its near right wall
    Turn right 180 if its near left wall
    */

    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC, LOW);
    pinMode(ULTRASONIC, INPUT);
    long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
    float left_distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;


    set_decoder(3);  // Turn on IR emitter
    delay(10);
    float reading = analogRead(IRREADER);  // Take reading from the IR senor with IR emiter ON

    set_decoder(1);  // Turn off IR emiter
    delay(10);
    float value = analogRead(IRREADER) - reading;  // Take account of the environment IR

    if ((left_distance < 8 && left_distance > 0)) {
      TURN_BACK_RIGHT();
    } else if (left_distance == 0 && reading < 45) {  // No left wall detected, use IR sensor to check if right wall is close
      TURN_BACK_RIGHT();                              // Turn back in right direction if right wall is far
    } else {
      TURN_BACK_LEFT();  // Turn back in right direction if right wall is close
    }
  }
}

float square(float n) {  // Return the square of the input value
  return n * n;
}

void setup() {
  pinMode(A7, INPUT);  // Setup A7 as input for the push button
  Serial.begin(9600);  // To initialize the serial monitor
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(IRREADER, INPUT);
  pinMode(LDRREADER, INPUT);
  pinMode(CONTROL1, OUTPUT);
  pinMode(CONTROL2, OUTPUT);
  led.setpin(13);
  //setBalance();
}

void loop() {
  if (analogRead(A7) < 100) {  // If push button is pushed, the value will be very low
    status = 1 - status;       // Toggle status
    delay(500);                // Delay 500ms so that a button push won't be counted multiple times.
  }

  if (status == 1) {                             // Run mBot only if status is 1
    int sensorState = lineFinder.readSensors();  // Read the line sensor's state

    if (sensorState == S1_IN_S2_IN) {  // Detects black line indicating coloured paper
      STOP_MOTOR();                    // Stop motor

      for (int i = 0; i < 3; i += 1) {                                              // Detects the colour below, loop through each Red, Green, Blue LED to get each colour Array value
        set_decoder(i);                                                             // Turn ON the LED (Red, Green, Blue) one at a time
        delay(RGBWait);                                                             // Allow the LED to stabalise
        colourArray[i] = getAvgReading(10);                                         // Get the average reading for each colour
        colourArray[i] = ((colourArray[i] - blackArray[i]) / (greyDiff[i])) * 255;  // Reading minus the lowest value, divided by the maximun range, multiplied by 255 will give the supposed RGB value
        set_decoder(3);                                                             // Turn OFF the LED
        delay(RGBWait);                                                             // Allow the LED to fully turn off before turning on next LED
      }


      float diff = 0;
      int colour = 0;
      float min = square(colourStorage[0][0] - colourArray[0]) + square(colourStorage[0][1] - colourArray[1]) + square(colourStorage[0][2] - colourArray[2]);
      for (int j = 1; j < 6; j += 1) {
        // Calculate the distance to find the nearest possible colour. Store the nearest colour in the integer colour
        float diff = square(colourStorage[j][0] - colourArray[0]) + square(colourStorage[j][1] - colourArray[1]) + square(colourStorage[j][2] - colourArray[2]);
        if (diff < min) {
          min = diff;
          colour = j;
        }
      }

      identify_colour(colour);  // Call function for required the action after the colour is identified

    } else if (sensorState == S1_OUT_S2_OUT) {  // Did not detect black line

      // Left Distance Detection
      digitalWrite(ULTRASONIC, LOW);
      delayMicroseconds(2);
      digitalWrite(ULTRASONIC, HIGH);
      delayMicroseconds(10);
      digitalWrite(ULTRASONIC, LOW);
      pinMode(ULTRASONIC, INPUT);
      long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
      float left_distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;  // Calculate the distance from mBot to the left wall

      if ((left_distance < 8 && left_distance > 0)) {  // Check if too close to left wall
        ADJUST_RIGHT();
      } 
      else if (left_distance > 12.5) {  // If left_distance to too far right, turn ON IR to check for right distance
        // Right Distance Detection
        set_decoder(3);                        // Turn on IR emitter
        delay(10);                             // Allow IR to stabilise
        float reading = analogRead(IRREADER);  // Take reading from the IR senor with IR emiter ON

        set_decoder(1);                                // Turn OFF IR emitter
        delay(10);                                     // Allow IR to stabalise
        float value = analogRead(IRREADER) - reading;  // Take account of the environment IR

        if (value > 52) {  // If mBot is too close to right wall
          ADJUST_LEFT();   // Adjust to the left to prevent mBot from colliding the right wall
        } else {           // mBot is at a safe distance
          MOVE_FORWARD();  // Continue moving forward
        }
      } else {           // mBot is neither too close to left or right wall
        MOVE_FORWARD();  // Continue moving forward
      }
    }
  }
  delay(10);
}
