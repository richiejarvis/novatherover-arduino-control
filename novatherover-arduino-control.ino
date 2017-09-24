#include <PS2X_lib.h>

// PS2 Controller pins
#define PS2_DAT        2
#define PS2_CMD        4
#define PS2_ATT        3
#define PS2_CLK        5

// Magnetic Brake
#define RELAY1_PIN     6

// Steering Power
#define RELAY2_PIN     7

// Steering Relay pins
#define RELAY3_PIN     8
#define RELAY4_PIN     9

// Motor direction pin
#define DIR_PIN        10

// Throttle pin
#define PWM_PIN        11

#define PS2_PRESSURES  false
#define PS2_RUMBLE     false

PS2X ps2x; // create PS2 Controller Class

// Initial Startup Variables
int error = 0;
byte type = 0;
byte vibrate = 0;
int pwm_value = 0;
int throttle_pos = 127;
int steering_pos = 127;

int max_speed = 20;
int counter = 0;
int buttonBounceCounter = 0;

// Set pin vars to off/high
int motor_direction = HIGH;
int engine_power = HIGH;
int steering_power = HIGH;
int steering_relay_1 = HIGH;
int steering_relay_2 = HIGH;
int increment = 0;
int target_speed = 0;

boolean cruise_control = false;
boolean beast_mode = false;
boolean first_run = false;

// Storage array to record previous condition
int storage_array[];

// Card reset function
void(* resetFunc) (void) = 0;

void setup()
{
  Serial.begin(57600);
  delay(300);
  // GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, PS2_PRESSURES, PS2_RUMBLE);

  if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  // Something up with the controller - abort and reboot!
  // Delay in place to stop it checking too frequently....
  if (error != 0) {
    delay(2000);
    resetFunc();
  }

  // Set pin 11 PWM frequency to 62500 Hz (62500/1 = 62500)
  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  //Initialise outputs
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  // Set initial states
  digitalWrite(RELAY1_PIN, engine_power);
  digitalWrite(RELAY2_PIN, steering_power);
  digitalWrite(RELAY3_PIN, steering_relay_1);
  digitalWrite(RELAY4_PIN, steering_relay_2);
}

// Controls
// PSAB_L1 - Max Speed Down
// PSAB_R1 - Max Speed Up
// PSAB_CIRCLE - Cruise On/Off
// PSS_LY/PSS_RY - Left Stick (Up/Down) - Throttle
// PSS_LX/PSS_RX - Right Stick (Left/Right) - Steering
// PSB_TRIANGLE - Speed 20 (Lowest working speed)
// PSB_SQUARE - Emergency Stop

void speedChange() {
  // Check for beast_mode first....
  if (!beast_mode && target_speed != pwm_value) {
    // Speed up more gently
    if (first_run) {
      // Only set the increment once
      increment = target_speed / 20;
      first_run = false;
    }
    if (target_speed > pwm_value) {
      pwm_value = pwm_value + increment;
      if (pwm_value > target_speed) {
        pwm_value = target_speed;
      }
    }
    else if (target_speed < pwm_value) {
      // Slow down in a less jolty-way
      if (pwm_value > 0 && pwm_value < 100) {
        // speed is less than 50 - use 10 step slowdown
        pwm_value = pwm_value - 10;
      } else if (pwm_value >= 100 & pwm_value < 200) {
        // speed is greater than 100 && less than 200 - use 30 step slowdown
        pwm_value = pwm_value - 30;
      } else if (pwm_value > 200) {
        // speed is greater than 200 use 60 step slowdown
        pwm_value = pwm_value - 60;
      }
    }
  }
}

// Write current values to the EEPROM for permanent storage for later retrieval.
// Might not need to store too much....
boolean dumpToEeprom() {
  //
}

// Make numbers appear in
String threeDigitFormat(int value) {
  if (value < 10) {
    return "00" + String(value);
  } else if (value < 100) {
    return "0" + String(value);
  } else {
    return String(value);
  }
}

// This function will report the newValue only when it does not match the old value
int reportOnRelease(int newValue, int storagePos) {
  if (newValue == storage_array[storagePos]) {
    return storage_array[storagePos];
  } else {
    return storage_array[newValue];
  }
}

void loop()
{
  ps2x.read_gamepad();

  // Check the stick positions.
  // If they are all at 0,128 or 0, then there is a controller problem, and we should not do anything
  if (
    // All axis cannot be 0
    //    (ps2x.Analog(PSS_LY) == 0 && ps2x.Analog(PSS_LX) == 0 && ps2x.Analog(PSS_RY) == 0 && ps2x.Analog(PSS_RX) == 0) ||
    // All axis cannot be 128 at the same time (127 is middle)
    (ps2x.Analog(PSS_LY) == 128 && ps2x.Analog(PSS_LX) == 128 && ps2x.Analog(PSS_RY) == 128 && ps2x.Analog(PSS_RX) == 128) ||
    // All axis cannot be at max at the same time
    //       (ps2x.Analog(PSS_LY) == 255 && ps2x.Analog(PSS_LX) == 255 && ps2x.Analog(PSS_RY) == 255 && ps2x.Analog(PSS_RX) == 255)
  ) {

    //    Serial.println("Controller Failure - Abort");
    // Stop this happening too frequently
    delay(500);
    // Might want to store stuff in EEPROM here before restart....
    resetFunc();
  } else {

    // Read the Y position (Throttle)
    // 0-126 == Up
    // 127 == At rest
    // 128 == disconnected
    // 129-255 == Down

    // Read the Left stick position in preference to Right
    // If LX is reporting something, then use that
    // Otherwise use RX
    throttle_pos = ps2x.Analog(PSS_LY);
    if (throttle_pos == 127 || throttle_pos == 128)) {
      throttle_pos = ps2x.Analog(PSS_RY);
    }


    // Read the X position (Steering)
    // 0-126 == Left
    // 127 == At rest
    // 128 == disconnected
    // 129-255 == Right

    // Read the Left stick position in preference to Right
    // If LX is reporting something, then use that
    // Otherwise use RX
    steering_pos = ps2x.Analog(PSS_LX);
    if (steering_pos == 127 || steering_pos == 128)) {
    steering_pos = ps2x.Analog(PSS_RX);
    }


    // Set the Throttle Position.
    if (throttle_pos < 127) {
    // Go Forward
    target_speed = map(throttle_pos, 128, 0, 0, max_speed);
      motor_direction = LOW;
      engine_power = LOW;
      first_run = true;
      speedChange();
    } else if (throttle_pos > 128) {
    // Go Backwards
    target_speed = map(throttle_pos, 128, 255, 0, max_speed);
      motor_direction = HIGH;
      engine_power = LOW;
      first_run = true;
      speedChange();
    } else if (cruise_control && pwm_value > 0) {
    // Cruise Mode On
    engine_power = LOW;
  } else if (throttle_pos == 127 || ) {
    target_speed = 0;
    engine_power = HIGH;
    first_run = true;
    speedChange();
    }
  }

  // Adjust the steering via relays
  if (steering_pos < 100 ) {
    // Steer Left
    steering_power = LOW;
    steering_relay_1 = LOW;
    steering_relay_2 = LOW;
  } else if (steering_pos > 200) {
    // Steer Right
    steering_power = LOW;
    steering_relay_1 = HIGH;
    steering_relay_2 = HIGH;
  } else  {
    // Don't Move
    steering_power = HIGH;
    // These 2 aren't needed, except to turn the Relay LED's off.....
    steering_relay_1 = HIGH;
    steering_relay_2 = HIGH;
  }
  // Modify Max Speed
  if (ps2x.Button(PSB_L1)) {
    max_speed = max_speed - 2;
  }
  if (ps2x.Button(PSB_R1)) {
    max_speed = max_speed + 2;
  }


  // Enable/Disable Beast Mode
  if (ps2x.Button(PSB_SELECT)) {
    if (reportOnRelease(true, beast_mode)) {
      if (beast_mode) {
        beast_mode = false;
        max_speed = 40;
      } else {
        // Someone enabled Beast Mode!
        beast_mode = true;
        // Speed to Max
        max_speed = 250;
      }
    }
  }
  // Limit max speed
  if (max_speed > 250) {
    max_speed = 250;
  }
  if (max_speed < 15) {
    max_speed = 15;
  }
  // Cruise Control
  if (ps2x.Button(PSB_R2)) {
    cruise_control = true;
  }
  // Cruise Off
  if (ps2x.Button(PSB_L2)) {
    cruise_control = false;
  }

  // *********************************
  // Output Current Status - Do Stuff!
  // *********************************
  Serial.print( "max_speed: " + threeDigitFormat(max_speed));
  Serial.print(" pwm:" + threeDigitFormat(pwm_value));
  Serial.print(" target_speed: " + threeDigitFormat(target_speed));
  Serial.print(" dir: " + String(motor_direction) );
  Serial.print(" cruise: " + String(cruise_control));
  Serial.print(" beast: " + String(beast_mode));
  Serial.print(" brake: " + String(engine_power));
  Serial.print(" steering_power: " + String(steering_power));
  Serial.print(" Throttle:" + threeDigitFormat(steering_pos));
  Serial.print(" Steering:" + threeDigitFormat(throttle_pos));
  Serial.println(" relays:" + String(steering_relay_1) + String(steering_relay_2));
  // Make sure values are between 0 and 250
  if (pwm_value < 0) {
    pwm_value = 0;
  }
  if (pwm_value > 250) {
    pwm_value = 250;
  }

  // Store values to see if they are consistent
  // Use case:
  // Only activate/deactivate cruise control when button is released
  // To stop on/off/on/off scenario whilst button is down
  storage_array[] = {max_speed,
                     pwm_value,
                     target_speed,
                     motor_direction,
                     cruise_control,
                     beast_mode,
                     engine_power,
                     steering_power,
                     steering_pos,
                     throttle_pos,
                     steering_relay_1,
                     steering_relay_2
                    };
  // Do Stuff!
  // Turn brake off
  digitalWrite(RELAY1_PIN, engine_power);
  // Set the throttle
  analogWrite(PWM_PIN, pwm_value);
  digitalWrite(DIR_PIN, motor_direction);
  // Setup steering
  digitalWrite(RELAY3_PIN, steering_relay_1);
  digitalWrite(RELAY4_PIN, steering_relay_2);
  // Now turn the power on for the steering
  digitalWrite(RELAY2_PIN, steering_power);
  delay (10);
}



