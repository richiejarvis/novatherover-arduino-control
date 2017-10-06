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
int brake_power = HIGH;
int steering_power = HIGH;
int steering_relay_1 = HIGH;
int steering_relay_2 = HIGH;
int increment = 0;
int target_speed = 0;
int pss_lx = 0;
int pss_ly = 0;
int pss_rx = 0;
int pss_ry = 0;

boolean cruise_control = false;
boolean beast_mode = false;
boolean first_run = false;
boolean skip_initial = true;
boolean controller_present = false;
boolean accept_change = true;

// Storage array to record previous condition
int storage_array[20];

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
    Serial.println("Init Restart");
    delay(2000);
    resetFunc();
  } else {
    Serial.println("Startup Successful");
    controller_present = true;
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
  digitalWrite(RELAY1_PIN, brake_power);
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
  first_run = true;

  // Check for beast_mode first....
  if ((target_speed != pwm_value)) {
    // Speed up more gently

    if (first_run) {
      // Only set the increment once
      increment = target_speed / 50;
      if (increment < 1) {
        increment = 1;
      }
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
        pwm_value = pwm_value - 2;
      } else if (pwm_value >= 100 & pwm_value < 200) {
        // speed is greater than 100 && less than 200 - use 30 step slowdown
        pwm_value = pwm_value - 5;
      } else if (pwm_value >= 200) {
        // speed is greater than 200 use 60 step slowdown
        pwm_value = pwm_value - 10;
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



void loop()
{
  ps2x.read_gamepad();
  // If there is no controller connected on startup, the init part seems to report working
  // Then we get strange values set in the pwm and speed fields
  // Workaround to skip the first loop on restart of Arduino
  if (!skip_initial) {
    pss_lx = ps2x.Analog(PSS_LX);
    pss_ly = ps2x.Analog(PSS_LY);
    pss_rx = ps2x.Analog(PSS_RX);
    pss_ry = ps2x.Analog(PSS_RY);
    // Check the stick positions.
    // If they are all at 0,128 or 0, then there is a controller problem, and we should not do anything
    if (pss_lx == 128 && pss_ly == 128 && pss_rx == 128 && pss_ry == 128
        || (pss_lx == 255 && pss_ly == 255 && pss_rx == 255 && pss_ry == 255)
       ) {
      controller_present = false;
    } else {
      controller_present = true;
      // Read the Y position (Throttle)
      // 0-126 == Up
      // 127 == At rest
      // 128 == disconnected
      // 129-255 == Down
      // Read the Left stick position in preference to Right
      // RX and LX report 128 in middle position
      // If LX is reporting something, then use that
      // Otherwise use RX
      throttle_pos = pss_ly;
      if ((throttle_pos == 127 || throttle_pos == 128)) {
        throttle_pos = pss_ry;
      }
      // Read the X position (Steering)
      // 0-126 == Left
      // 127 == At rest
      // 128 == disconnected
      // 129-255 == Right

      // Read the Left stick position in preference to Right
      // If LX is reporting something, then use that
      // Otherwise use RX
      steering_pos = pss_lx;
      if (steering_pos == 127 || steering_pos == 128) {
        steering_pos = pss_rx;
      }


      // Set the Throttle Position.
      // Ignore if Cruise is on.
      if (!cruise_control) {
        if (throttle_pos < 90) {
          // Go Forward
          target_speed = map(throttle_pos, 128, 0, 0, max_speed);
          motor_direction = LOW;
          brake_power = LOW;
        } else if (throttle_pos > 190) {
          // Go Backwards
          target_speed = map(throttle_pos, 128, 250, 0, max_speed);
          motor_direction = HIGH;
          brake_power = LOW;
        } else if (cruise_control) {
          // Cruise Mode On
          brake_power = LOW;
        } else if (throttle_pos == 127 ) {
          target_speed = 0;
          brake_power = HIGH;
        }
      } else {
        target_speed = max_speed;
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
        first_run = true;
      }
      if (ps2x.Button(PSB_R1)) {
        max_speed = max_speed + 2;
        first_run = true;
      }
      // Emergency Stop
      if (ps2x.Button(PSB_CROSS)) {
        pwm_value = 0;
        brake_power = HIGH;
        target_speed = 0;
        cruise_control = false;
      }
      if (accept_change) {
        // Beast Mode
        // If disabled, turn it on
        // If enabled, turn it off
        if (!beast_mode) {
          // Enable Beast Mode
          if (ps2x.Button(PSB_SELECT)) {
            beast_mode = true;
            max_speed = 255;
            target_speed = 255;
            pwm_value = 230;
            accept_change = false;
          }
        } else {
          // Disable Beast Mode
          if (ps2x.Button(PSB_SELECT)) {
            beast_mode = false;
            max_speed = 20;
            target_speed = 20;
            accept_change = false;
          }
        }
        // Cruise Control
        // If disabled, turn it on
        // If enabled, turn it off
        if (!cruise_control) {
          if ((ps2x.Button(PSB_R2) || ps2x.Button(PSB_L2))) {
            cruise_control = true;
            pwm_value = target_speed;
            accept_change = false;
          }
        } else {
          // Cruise Off
          if ((ps2x.Button(PSB_R2) || ps2x.Button(PSB_L2))) {
            cruise_control = false;
            target_speed = 0;
            first_run = true;
            accept_change = false;
          }
        }
      }
      // Reset accept_change if buttons are not pushed
      if (!(ps2x.Button(PSB_SELECT) || ps2x.Button(PSB_R2) || ps2x.Button(PSB_L2))) {
        accept_change = true;
      }
    }


    // Limit max speed
    if (max_speed > 250) {
      max_speed = 250;
    }
    if (max_speed < 15) {
      max_speed = 15;
    }

    // Make sure values are between 0 and 250
    if (pwm_value < 0) {
      pwm_value = 0;
    }
    if (pwm_value > 250) {
      pwm_value = 250;
    }

    if (controller_present) {
      // Store values to see if they are consistent
      // Use case:
      // Only activate/deactivate cruise control when button is released
      // To stop on/off/on/off scenario whilst button is down
      storage_array[0] = max_speed;
      storage_array[1] = pwm_value;
      storage_array[2] = target_speed;
      storage_array[3] = motor_direction;
      storage_array[4] = cruise_control;
      storage_array[5] = beast_mode;
      storage_array[6] = brake_power;
      storage_array[7] = steering_power;
      storage_array[8] = steering_pos;
      storage_array[9] = throttle_pos;
      storage_array[10] = steering_relay_1;
      storage_array[11] = steering_relay_2;
      if (pwm_value != target_speed) {
        speedChange();
      }
      // Do Stuff!
      // Turn brake off
      digitalWrite(RELAY1_PIN, brake_power);
      // Set the throttle
      analogWrite(PWM_PIN, pwm_value);
      digitalWrite(DIR_PIN, motor_direction);
      // Setup steering
      digitalWrite(RELAY3_PIN, steering_relay_1);
      digitalWrite(RELAY4_PIN, steering_relay_2);
      // Now turn the power on for the steering
      digitalWrite(RELAY2_PIN, steering_power);
    } else {
      target_speed = 0;
    }
  } else {
    // First loop over - go to normal mode
    skip_initial = false;
  }
  // *********************
  // Output Current Status
  // *********************
  Serial.print("LX:" + threeDigitFormat(pss_lx));
  Serial.print(" LY:" + threeDigitFormat(pss_ly));
  Serial.print(" RX:" + threeDigitFormat(pss_rx));
  Serial.print(" RY:" + threeDigitFormat(pss_ry));
  Serial.print(" max_spd:" + threeDigitFormat(max_speed));
  Serial.print(" pwm:" + threeDigitFormat(pwm_value));
  Serial.print(" trgt_spd:" + threeDigitFormat(target_speed));
  Serial.print(" fwd:" + String(motor_direction) );
  Serial.print(" cruise:" + String(cruise_control));
  Serial.print(" beast:" + String(beast_mode));
  Serial.print(" stop:" + String(brake_power));
  Serial.print(" spd:" + threeDigitFormat(throttle_pos));
  Serial.print(" dir:" + threeDigitFormat(steering_pos));
  Serial.print(" pad:" + String(controller_present));
  Serial.println(" dir:" + String(steering_power) + String(steering_relay_1) + String(steering_relay_2));
  delay (10);
}



