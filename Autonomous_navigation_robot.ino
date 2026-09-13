// ENGR 122 Final Project 
 
#include "MQTT.h" 
#include <Wire.h> 
#include <SSD1306Wire.h> 
#include <Ultrasonic.h> 
#include <Servo.h> 
 
// Motor pins 
#define motor1pin D3 
#define motor2pin D6 
 
Servo motor1; 
Servo motor2; 
 
// Ultrasonic sensors 
Ultrasonic ultrasonic_front(D5, D8); 
Ultrasonic ultrasonic_left(TX, D0); 
Ultrasonic ultrasonic_right(D4, D7); 
 
// OLED display 
SSD1306Wire display(0x3C, D2, D1); 
 
// Motor speed values 
#define STOP_MOTORS 90 
#define FORWARD_SPEED_RIGHT 120 
#define FORWARD_SPEED_LEFT 120 
#define SLOW_SPEED_RIGHT 100 
#define SLOW_SPEED_LEFT 100 
#define TURN_1 115 
#define TURN_2 65 
 
// Distance cutoffs 
#define FRONT 12.5 
#define SIDE 12.5 
#define ARRIVE_MM 100.0 
#define SLOWDOWN_MM 300.0 
 
// Timing 
#define STOP_MS 2000 
#define STUCK_MS 1000 
 
// Target coordinates in millimeters 
const int x_targets[] = {700, 1560, 2140, 150}; 
const int y_targets[] = {650, 120, 650, 150}; 
int target_index = 0; 
const int total_targets = sizeof(x_targets) / sizeof(x_targets[0]); 
 
// MQTT settings 
const char* mqtt_server = "YOUR_MQTT_SERVER"; 
const char* mqtt_username = "YOUR_MQTT_USERNAME"; 
const char* mqtt_password = "YOUR_MQTT_PASSWORD"; 
const int mqtt_port = 1883; 
 
const String topic = "EAS011_South"; 
 
// WiFi settings 
const char* ssid = "YOUR_WIFI_NAME"; 
const char* password = "YOUR_WIFI_PASSWORD"; 
 
// Position values come from MQTT 
float x_robot, y_robot, z_ang_robot; 
 
bool reached = false; 
unsigned long reach_time = 0; 
 
// Used to check whether the robot is actually moving 
float last_x = 0, last_y = 0; 
unsigned long last_move_time = 0; 
 
// Helps stop the robot from turning back and forth too long 
int oscillation_count = 0; 
int last_turn_direction = 0; 
 
void STOP() { 
  motor1.write(STOP_MOTORS); 
  motor2.write(STOP_MOTORS); 
} 
 
void FORWARD() { 
  motor1.write(FORWARD_SPEED_RIGHT); 
  motor2.write(FORWARD_SPEED_LEFT); 
} 
 
void SLOW() { 
  motor1.write(SLOW_SPEED_RIGHT); 
  motor2.write(SLOW_SPEED_LEFT); 
} 
 
void LEFT() { 
  motor1.write(TURN_2); 
  motor2.write(TURN_1); 
} 
 
void RIGHT() { 
  motor1.write(TURN_1); 
  motor2.write(TURN_2); 
} 
 
void SLIGHT_RIGHT() { 
  motor1.write(FORWARD_SPEED_RIGHT); 
  motor2.write(SLOW_SPEED_LEFT); 
} 
 
void SLIGHT_LEFT() { 
  motor1.write(SLOW_SPEED_RIGHT); 
  motor2.write(FORWARD_SPEED_LEFT); 
} 
 
bool arrived() { 
  float dist = sqrt(pow(x_robot - x_targets[target_index], 2) + pow(y_robot - y_targets[target_index], 2)); 
  return dist <= ARRIVE_MM; 
} 
 
float dist_to_target() { 
  float dx = x_targets[target_index] - x_robot; 
  float dy = y_targets[target_index] - y_robot; 
  return sqrt(dx * dx + dy * dy); 
} 
 
float angle_error() { 
  float dx = x_targets[target_index] - x_robot; 
  float dy = y_targets[target_index] - y_robot; 
 
  // atan2 gives the angle toward the target. Subtracting the robot angle gives the turn error. 
  float ang_err = atan2(dy, dx) * 180.0 / PI - z_ang_robot; 
 
  // Keeps the angle error inside the normal -180 to 180 degree range. 
  if (ang_err > 180) ang_err -= 360; 
  if (ang_err < -180) ang_err += 360; 
 
  return ang_err; 
} 
 
float angle_threshold(float dist) { 
  // The robot can be less exact when it is far away, then gets stricter near the target. 
  if (dist > 1000) return 30.0; 
  if (dist > 500) return 20.0; 
  return 10.0; 
} 
 
void update_display() { 
  float ang_err = angle_error(); 
 
  display.clear(); 
  display.drawString(0, 0, "X-coordinate: " + String(x_robot, 2)); 
  display.drawString(0, 15, "Y-coordinate: " + String(y_robot, 2)); 
  display.drawString(0, 30, "Target Angle: " + String(ang_err, 2)); 
  display.drawString(0, 45, "Target: " + String(target_index + 1) + " of " + String(total_targets)); 
  display.display(); 
} 
 
void refresh() { 
  mqtt_rebound(); 
  client.loop(); 
  update_display(); 
} 
 
bool obstacle() { 
  int front = ultrasonic_front.read(); 
  int left = ultrasonic_left.read(); 
  int right = ultrasonic_right.read(); 
 
  return (front < FRONT || left < SIDE || right < SIDE); 
} 
 
void avoid() { 
  while (true) { 
    refresh(); 
 
    if (arrived()) { 
      STOP(); 
      return; 
    } 
 
    int front = ultrasonic_front.read(); 
    int left = ultrasonic_left.read(); 
    int right = ultrasonic_right.read(); 
 
    // Once all sensors are clear, return to the normal target-following code. 
    if (front >= FRONT && left >= SIDE && right >= SIDE) { 
      return; 
    } 
 
    if (front < FRONT) { 
      if (left >= right) LEFT(); 
      else RIGHT(); 
    } else if (left < SIDE) { 
      SLIGHT_RIGHT(); 
    } else { 
      SLIGHT_LEFT(); 
    } 
 
    delay(50); 
  } 
} 
 
void align() { 
  float dx = x_targets[target_index] - x_robot; 
  float dy = y_targets[target_index] - y_robot; 
  float distance = sqrt(dx * dx + dy * dy); 
  float ang_err = angle_error(); 
 
  while (abs(ang_err) > angle_threshold(distance)) { 
    refresh(); 
 
    if (arrived()) { 
      STOP(); 
      return; 
    } 
 
    if (obstacle()) { 
      avoid(); 
      return; 
    } 
 
    int turn_direction = (ang_err > 0) ? 1 : -1; 
 
    // If the turn direction keeps flipping, the robot is probably overcorrecting. 
    if (turn_direction != last_turn_direction && last_turn_direction != 0) { 
      oscillation_count++; 
    } else { 
      oscillation_count = 0; 
    } 
 
    last_turn_direction = turn_direction; 
 
    if (oscillation_count >= 3) { 
      STOP(); 
 
      unsigned long start = millis(); 
 
      // Push forward briefly to break out of the back-and-forth turning pattern. 
      while (millis() - start < 2000) { 
        refresh(); 
 
        if (arrived()) { 
          STOP(); 
          return; 
        } 
 
        if (obstacle()) { 
          avoid(); 
          return; 
        } 
 
        FORWARD(); 
        delay(50); 
      } 
 
      STOP(); 
      oscillation_count = 0; 
      last_turn_direction = 0; 
      return; 
    } 
 
    if (ang_err > 0) LEFT(); 
    else RIGHT(); 
 
    delay(50); 
 
    ang_err = angle_error(); 
    dx = x_targets[target_index] - x_robot; 
    dy = y_targets[target_index] - y_robot; 
    distance = sqrt(dx * dx + dy * dy); 
  } 
 
  STOP(); 
} 
 
void setup() { 
  motor1.attach(motor1pin); 
  motor2.attach(motor2pin); 
 
  Wire.begin(); 
  delay(25); 
 
  wifi_mqtt_init(); 
  mqtt_clean(); 
  delay(500); 
 
  display.init(); 
  display.flipScreenVertically(); 
  display.setFont(ArialMT_Plain_10); 
  display.clear(); 
  display.display(); 
 
  last_move_time = millis(); 
} 
 
void loop() { 
  refresh(); 
 
  // Obstacle avoidance always has priority over target movement. 
  if (obstacle()) { 
    avoid(); 
    return; 
  } 
 
  if (target_index >= total_targets) { 
    STOP(); 
    return; 
  } 
 
  if (arrived()) { 
    if (!reached) { 
      reached = true; 
      reach_time = millis(); 
      STOP(); 
    } 
 
    if (millis() - reach_time >= STOP_MS) { 
      reached = false; 
      target_index++; 
      oscillation_count = 0; 
      last_turn_direction = 0; 
    } 
 
    refresh(); 
    delay(50); 
    return; 
  } 
 
  // If the position barely changes for too long, try a short forward push. 
  float moved = sqrt(pow(x_robot - last_x, 2) + pow(y_robot - last_y, 2)); 
 
  if (moved > 30) { 
    last_x = x_robot; 
    last_y = y_robot; 
    last_move_time = millis(); 
  } 
 
  if (millis() - last_move_time > STUCK_MS) { 
    unsigned long start = millis(); 
 
    while (millis() - start < 1500) { 
      refresh(); 
 
      if (arrived()) { 
        STOP(); 
        return; 
      } 
 
      if (obstacle()) { 
        avoid(); 
        return; 
      } 
 
      FORWARD(); 
      delay(50); 
    } 
 
    STOP(); 
    last_move_time = millis(); 
    return; 
  } 
 
  float dx = x_targets[target_index] - x_robot; 
  float dy = y_targets[target_index] - y_robot; 
  float dist = sqrt(dx * dx + dy * dy); 
  float ang_err = angle_error(); 
 
  if (abs(ang_err) > angle_threshold(dist)) { 
    align(); 
    return; 
  } 
 
  if (dist <= SLOWDOWN_MM) { 
    SLOW(); 
  } else { 
    FORWARD(); 
  } 
 
  while (true) { 
    refresh(); 
 
    if (arrived()) { 
      STOP(); 
      return; 
    } 
 
    if (obstacle()) { 
      STOP(); 
      avoid(); 
      return; 
    } 
 
    dx = x_targets[target_index] - x_robot; 
    dy = y_targets[target_index] - y_robot; 
    dist = sqrt(dx * dx + dy * dy); 
    ang_err = angle_error(); 
 
    if (dist <= SLOWDOWN_MM) { 
      SLOW(); 
    } else { 
      FORWARD(); 
    } 
 
    // Re-align if the robot starts drifting too far away from the target angle. 
    if (abs(ang_err) > angle_threshold(dist)) { 
      STOP(); 
      return; 
    } 
 
    delay(50); 
  } 
}