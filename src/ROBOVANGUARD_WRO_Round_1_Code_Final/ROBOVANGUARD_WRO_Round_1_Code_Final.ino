/*
  ROBOVANGUARD – WRO Future Engineers 2026
  World Robot Olympiad – Future Engineers Division

  Team ID: 1129 | Team Name: ROBOVANGUARD
  Mentor: Mr. S. Valai Ganesh (Mech, AP SG)
  Team Leader: M. Manojkumar (CSBS) – Reg. No: 953623244024
  Hardware Lead: V. Rakshit (EEE) – Reg. No: 953623105044
  Mechanical: P. Chandru (Mech) – Reg. No: 953623114009

  Hybrid Architecture:
  - ESP32 Hardware Driver & Telemetry Engine.
  - Idle state on boot waiting for Pi 5 USB Serial commands.
  - Periodic USB Serial Telemetry for All Ultrasonic Sensors (US:F:..,F1:..,F2:..,L:..,R:..,B:..).
*/

int line_chk_count = 12;  // Lap check threshold (3 laps x 4 turns = 12)
int line_count = 0;

//#---Bot Speeds & Timings---#############################################################
int normal_speed = 245; // PWM (0-255) - Increased for faster straightaway performance
int turn_speed = 255;   // PWM (0-255) - Full 100% power during corner turns
int turn_delay = 2000;  // ms (corner turn arc duration - 2.0s for full 90 deg turn)
int fus_slow_speed = 220; // PWM (slowdown speed when approaching front wall)
int fus_slow_dist = 80;   // cm (front wall distance threshold for slowdown)

//#---Servo Angles (+-40 deg steering range)---###########################################
int servo_center = 100;                  // 100 deg (Straight center)
int left_turn_angle = servo_center - 40; // 60 deg (Left turn)
int right_turn_angle = servo_center + 40;// 140 deg (Right turn)
int target_wall_dist = 25;               // cm target distance from side wall
//#######################################################################################

bool lt_st_count = 0;
bool rt_st_count = 0;
bool left_right_arc_turn = 1;
bool left_right_r_turn = 0;

int f_us, f1_us, f2_us, b_us, l_us, r_us, fusa, far;

bool LOGIC_LOCK = 1; // 1 True state.

// ########### USB Serial Command & Failsafe Definitions #################################//
String serialCommandBuffer = "";
unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long COMMAND_TIMEOUT = 500; // 500ms failsafe timeout
bool serialControlActive = false;
bool useSideUltrasonic = true;             // Enables/disables ESP32 ultrasonic centering

// Timed Arc Turn State Machine
bool isTurning = false;
bool last_cmd_was_left = false;
unsigned long turnStartTime = 0;

// Forward declarations for unified movement execution functions (in Lib_Declarations_Setup.ino)
void execute_forward();
void execute_backward();
void execute_left();
void execute_right();
void execute_stop();
void execute_steer(int angle);
void execute_drive(int speed, int angle);
void side_us_logic_fun();
void US_Values(int &f, int &f1, int &f2, int &b, int &l, int &r);
void bot_shutdown();

// Process incoming command from Raspberry Pi 5 over USB Serial
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  lastCommandTime = millis();
  serialControlActive = true;

  if (cmd == "FORWARD") {
    isTurning = false;
    execute_forward();
    Serial.println("ACK:FORWARD");
  } else if (cmd == "BACKWARD") {
    isTurning = false;
    execute_backward();
    Serial.println("ACK:BACKWARD");
  } else if (cmd == "LEFT" || cmd == "TURN_LEFT") {
    last_cmd_was_left = true;
    if (!isTurning) {
      isTurning = true;
      turnStartTime = millis();
      line_count++;
      Serial.print("ACK:TURN_LEFT:COUNT:");
      Serial.println(line_count);
    }
    moveServoTo(left_turn_angle);
    motor_forward(turn_speed);
  } else if (cmd == "RIGHT" || cmd == "TURN_RIGHT") {
    last_cmd_was_left = false;
    if (!isTurning) {
      isTurning = true;
      turnStartTime = millis();
      line_count++;
      Serial.print("ACK:TURN_RIGHT:COUNT:");
      Serial.println(line_count);
    }
    moveServoTo(right_turn_angle);
    motor_forward(turn_speed);
  } else if (cmd == "STOP") {
    isTurning = false;
    execute_stop();
    Serial.println("ACK:STOP");
  } else if (cmd == "AUTO_US_ON") {
    useSideUltrasonic = true;
    Serial.println("ACK:AUTO_US_ON");
  } else if (cmd == "AUTO_US_OFF") {
    useSideUltrasonic = false;
    Serial.println("ACK:AUTO_US_OFF");
  } else if (cmd.startsWith("SET_TURN_DELAY:")) {
    turn_delay = cmd.substring(15).toInt();
    Serial.print("ACK:SET_TURN_DELAY:");
    Serial.println(turn_delay);
  } else if (cmd.startsWith("SET_SPEED:")) {
    normal_speed = constrain(cmd.substring(10).toInt(), 100, 255);
    Serial.print("ACK:SET_SPEED:");
    Serial.println(normal_speed);
  } else if (cmd.startsWith("STEER:")) {
    isTurning = false;
    useSideUltrasonic = false;
    int angle = cmd.substring(6).toInt();
    execute_steer(angle);
    Serial.print("ACK:STEER:");
    Serial.println(angle);
  } else if (cmd.startsWith("DRIVE:")) {
    isTurning = false;
    useSideUltrasonic = false;
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    if (secondColon != -1) {
      int speed = cmd.substring(firstColon + 1, secondColon).toInt();
      int angle = cmd.substring(secondColon + 1).toInt();
      execute_drive(speed, angle);
      Serial.print("ACK:DRIVE:");
      Serial.print(speed);
      Serial.print(":");
      Serial.println(angle);
    } else {
      Serial.println("ERROR:INVALID_DRIVE_FORMAT");
    }
  } else {
    Serial.print("ERROR:UNKNOWN_COMMAND:");
    Serial.println(cmd);
  }
}

// Non-blocking serial character receiver
void checkSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialCommandBuffer.length() > 0) {
        serialCommandBuffer.trim();
        if (serialCommandBuffer.length() > 0) {
          processCommand(serialCommandBuffer);
        }
        serialCommandBuffer = "";
      }
    } else {
      if (serialCommandBuffer.length() < 64) {
        serialCommandBuffer += c;
      }
    }
  }
}

// Communication Failsafe Watchdog: automatically stops motors if no command received within timeout
void checkFailsafe() {
  if (serialControlActive && !isTurning) {
    if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
      execute_stop();
      serialControlActive = false;
    }
  }
}

void sendUltrasonicTelemetry() {
  if (millis() - lastTelemetryTime >= 100) {
    lastTelemetryTime = millis();
    Serial.print("US:F:");
    Serial.print(f_us);
    Serial.print(",F1:");
    Serial.print(f1_us);
    Serial.print(",F2:");
    Serial.print(f2_us);
    Serial.print(",L:");
    Serial.print(l_us);
    Serial.print(",R:");
    Serial.print(r_us);
    Serial.print(",B:");
    Serial.println(b_us);
  }
}

void loop() {
  // Read ultrasonic sensors
  US_Values(f_us, f1_us, f2_us, b_us, l_us, r_us);

  // 1. Process USB Serial commands from Raspberry Pi 5
  checkSerialInput();
  checkFailsafe();
  sendUltrasonicTelemetry();

  // 2. Timed Arc Turn Non-Blocking Update
  if (isTurning) {
    if (millis() - turnStartTime >= (unsigned long)turn_delay) {
      isTurning = false;
      moveServoTo(servo_center);
      motor_forward(normal_speed);
      Serial.println("ACK:TURN_COMPLETE");
    } else {
      // KEEP SERVO LOCKED AT TURN ANGLE THROUGHOUT TURN DURATION
      if (last_cmd_was_left) {
        moveServoTo(left_turn_angle);
      } else {
        moveServoTo(right_turn_angle);
      }
    }
  } 
  // 3. Side Ultrasonic Centering (Active when driving forward and not executing a turn)
  else if (serialControlActive && useSideUltrasonic) {
    side_us_logic_fun();
  }
  // 4. Standby / Idle Mode (Waiting for Pi 5 Serial Commands)
  else if (!serialControlActive) {
    bot_shutdown();
  }
}

// Robust Side Ultrasonic Steering Logic with Outlier Filtering & Dual/Single Wall Fallback
void side_us_logic_fun() {              
  // Emergency front collision slow-down
  if (f_us > 0 && f_us < fus_slow_dist) {
    motor_forward(fus_slow_speed);
  } else {
    motor_forward(normal_speed);
  } 

  bool valid_left = (l_us > 5 && l_us < 120);
  bool valid_right = (r_us > 5 && r_us < 120);

  // CASE 1: Both Left and Right ultrasonic sensors see valid walls -> Centering
  if (valid_left && valid_right) {
    rgb_led(0, 255, 0); // Green LED
    int diff = r_us - l_us; // Positive when closer to Left wall
    int target_angle = servo_center + (diff * 2); // Steers RIGHT (+) away from Left wall
    moveServoTo(target_angle);
  } 
  // CASE 2: Only Left ultrasonic sees valid wall -> Maintain target left distance
  else if (valid_left) {
    rgb_led(0, 255, 255); // Cyan LED
    int err = target_wall_dist - l_us; // Positive when too close to Left wall
    int target_angle = servo_center + (err * 2); // Steers RIGHT (+) away from Left wall
    moveServoTo(target_angle);
  } 
  // CASE 3: Only Right ultrasonic sees valid wall -> Maintain target right distance
  else if (valid_right) {
    rgb_led(255, 255, 0); // Yellow LED
    int err = target_wall_dist - r_us; // Positive when too close to Right wall
    int target_angle = servo_center - (err * 2); // Steers LEFT (-) away from Right wall
    moveServoTo(target_angle);
  } 
  // CASE 4: No valid side wall readings -> Maintain straight center
  else {
    rgb_led(255, 0, 0); // Red LED
    moveServoTo(servo_center);
  }
}

void bot_shutdown() {
  motor_stop();
  moveServoTo(servo_center);
  rgb_led(0, 0, 0);
}