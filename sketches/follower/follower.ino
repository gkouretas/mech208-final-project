#include <HCSR04.h>
#include <pid.h>
#include <Timer.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

/* Helper structs */

// PID pin inputs
typedef struct {
  const int kp_in;
  const int ki_in;
  const int kd_in;
} pid_inputs_t;

typedef struct {
  const int kp_max;
  const int ki_max;
  const int kd_max;
} pid_gains_max_t;

// Distance sensor
typedef struct {
  const int echo;
  const int trigger;
} ultrasonic_pins_t;

constexpr int moving_average_window_size = 10;

// Moving average container
typedef struct {
  uint8_t index;
  float buffer[moving_average_window_size];
  float raw;
  float filtered;
} filtered_data_t;

// Command interface
typedef struct {
  const pid_inputs_t gains;
  const pid_gains_max_t max_gains;
  const ultrasonic_pins_t position_input;
  const int command_input;
} input_command_interface_t;

typedef enum {
  FAN,
  BEAM
} system_type_t;

typedef struct {
  const system_type_t sys;                        // enum: system type
  UltraSonicDistanceSensor position_sensor;       // position sensor object
  PID controller;                                 // PID controller
  filtered_data_t target_position;                // target position
  filtered_data_t actual_position;                // sensed position
  int input_limits[2];                            // input min/max
  int output_limits[2];                           // output min/max
  void (*command_handler)(int command);           // callback for command
  input_command_interface_t *command_interface;   // command interface containing IO pins
} system_interface_t;

/* Macros */
// Constants
#define NUMBER_OF_SYSTEMS 2 

// Struct initializers
#define INIT_COMMAND_INTERFACE(SYSTEM)                                                                                \
  { .gains = { kp_##SYSTEM##_input, ki_##SYSTEM##_input, kd_##SYSTEM##_input },                                       \
    .max_gains = { kp_##SYSTEM##_max, ki_##SYSTEM##_max, kd_##SYSTEM##_max },                                         \
    .position_input = { .echo = SYSTEM##_ultrasonic_sensor_echo, .trigger = SYSTEM##_ultrasonic_sensor_trigger },     \
    .command_input = pot_##SYSTEM##_command_input,                                                                    \
  }

#define INIT_FILTERED_DATA { .index = -1 }

// Logging
#define NEWLINE()     \
  Serial.println()

#define LOG(buf) {    \
  Serial.print(buf);  \
  Serial.print(" ");  \
}

#define LOGEOL(buf)   \
  Serial.println(buf);

/* Pinouts */

// System-related
constexpr int kp_fan_input                       = A1;
constexpr int ki_fan_input                       = A2;
constexpr int kd_fan_input                       = A3;
constexpr int pot_fan_command_input              = A0;
constexpr int fan_ultrasonic_sensor_trigger      = 12;
constexpr int fan_ultrasonic_sensor_echo         = 13;
constexpr int kp_beam_input                      = A4;
constexpr int ki_beam_input                      = A5;
constexpr int kd_beam_input                      = A6;
constexpr int pot_beam_command_input             = A0;
constexpr int beam_ultrasonic_sensor_trigger     = 14;
constexpr int beam_ultrasonic_sensor_echo        = 15;

constexpr int gain_input_toggle           = 18;
constexpr int enable_input_toggle         = 19;
constexpr int primary_input_toggle        = 2;

// Max kp, ki, kd values
constexpr double kp_fan_max = 25.0;
constexpr double ki_fan_max = 1.0;
constexpr double kd_fan_max = 35.0;
constexpr double kp_beam_max = 10.0;
constexpr double ki_beam_max = 1.0;
constexpr double kd_beam_max = 35.0;

// Additional pins
constexpr int fan_pwm_out                        = 10;
constexpr int bean_servo_pin                     = 9;
constexpr int led_gains                          = 51;
constexpr int led_enable                         = 43;
constexpr int led_fan_primary                    = 49;
constexpr int led_beam_primary                   = 45;

constexpr int lcd_decimation = 5;

// Button debounce
constexpr float button_debounce_duration_ms      = 1000.0;
Timer gain_input_timer(button_debounce_duration_ms, MILLISECONDS);
Timer enable_input_timer(button_debounce_duration_ms, MILLISECONDS);
Timer primary_input_timer(button_debounce_duration_ms, MILLISECONDS);

/* Configuration params */
// Fan
#define fan_pid_limits        {0, 255}
#define servo_limits          {-90, 90}
#define fan_position_limits   {5, 30}
#define beam_position_limits  {5, 30}

// Beam
Servo beam_servo;
constexpr int beam_home_position                = 90;
constexpr float feed_forward_slope              = -2.0;
constexpr float feed_forward_offset             = 175.0;

unsigned long ts;
constexpr int loop_rate_ms = 50.0;

static input_command_interface_t fan_command_interface = INIT_COMMAND_INTERFACE(fan);
static input_command_interface_t beam_command_interface = INIT_COMMAND_INTERFACE(beam);

void fan_command_callback(int command) { 
  analogWrite(fan_pwm_out, command);
}

void beam_command_callback(int command) { 
  beam_servo.write(beam_home_position - command);
}

/* Volatile booleans for interrupt. Need to be defined external from struct to work... */
volatile bool gain_state = false;
volatile bool enable_state = false;
volatile bool primary_state = true;

system_interface_t fan_system_interface = {
  .sys = FAN,
  .position_sensor = UltraSonicDistanceSensor(
    fan_command_interface.position_input.trigger,
    fan_command_interface.position_input.echo
  ),
  .controller = PID(
    0, 
    0, 
    0,
    NEGATIVE,
    &compute_fan_feed_forward
  ),
  .target_position = { .index = -1 },
  .actual_position = { .index = -1 }, // TODO: why does macro init fail?
  .input_limits = fan_position_limits,
  .output_limits = fan_pid_limits,
  .command_handler = &fan_command_callback,
  .command_interface = &fan_command_interface,
};

system_interface_t beam_system_interface = {
  .sys = BEAM,
  .position_sensor = UltraSonicDistanceSensor(
    beam_command_interface.position_input.trigger,
    beam_command_interface.position_input.echo
  ),
  .controller = PID(
    0, 
    0, 
    0,
    NEGATIVE
  ),
  .target_position = { .index = -1 },
  .actual_position = { .index = -1 }, // TODO: why does macro init fail?
  .input_limits = beam_position_limits,
  .output_limits = servo_limits,
  .command_handler = &beam_command_callback,
  .command_interface = &beam_command_interface,
};

system_interface_t interfaces[NUMBER_OF_SYSTEMS] = {fan_system_interface, beam_system_interface};

system_interface_t* get_interface(system_type_t system_type) {
  for (system_interface_t &interface : interfaces) {
    if (interface.sys == system_type) return &interface;
  }

  return NULL;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // LCD setup
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Device Addr = 0x27");
  
  // Initialize interfaces
  pinMode(gain_input_toggle, INPUT_PULLUP);
  gain_input_timer.Start();

  pinMode(enable_input_toggle, INPUT_PULLUP);
  enable_input_timer.Start();

  pinMode(primary_input_toggle, INPUT_PULLUP);
  primary_input_timer.Start();

  pinMode(led_gains, OUTPUT);
  pinMode(led_enable, OUTPUT);
  pinMode(led_fan_primary, OUTPUT);
  pinMode(led_beam_primary, OUTPUT);

  // Attach interrupts. Has to be done individually w/ current setup since debounce timers are decoupled...
  attachInterrupt(digitalPinToInterrupt(gain_input_toggle), toggle_edit_gains, FALLING);
  attachInterrupt(digitalPinToInterrupt(enable_input_toggle), toggle_edit_enable, FALLING);
  attachInterrupt(digitalPinToInterrupt(primary_input_toggle), toggle_edit_primary, FALLING);

  // Set servo to home position
  beam_servo.attach(bean_servo_pin);
  beam_servo.write(beam_home_position);
}

void loop() {
  static unsigned long iter = 0;
  ts = millis();

  update_sensed_and_targets();

  for (system_interface_t &interface : interfaces) {
    /* Update gains if state allows it */
    if (gain_state) {
      update_gains(&interface);
    }
    
    if (enable_state) {
      /* Compute PID */
      interface.controller.Step(
        loop_rate_ms * 0.001, // [ms] -> [s]
        interface.actual_position.filtered, 
        interface.target_position.filtered
      );

      /* Send clamped output to command handler */
      interface.command_handler(
        interface.controller.GetClampedOutput(
          interface.output_limits[0], 
          interface.output_limits[1]
        )
      );
    } else {
      interface.command_handler(0);
    }

    /* Log data */
    log_data(&interface);
  } 

  /* Log timestamp, start newline in prep for next packet */
  LOGEOL(ts);

  digitalWrite(led_gains, !gain_state);
  digitalWrite(led_enable, enable_state);
  digitalWrite(led_fan_primary, primary_state);
  digitalWrite(led_beam_primary, !primary_state);

  /* Display gains */
  if (iter % lcd_decimation == 0) {
    lcdPrint();
  }

  ++iter;

  unsigned long duration = (millis() - ts);

  /* Enforce loop frequency */
  if (duration < loop_rate_ms)
    delay(loop_rate_ms - duration);
}

void lcdPrint() {  
  lcd.setCursor(0, 0);
  lcd.print("   Kp/   Ki/   Kd   ");

  for (int i = 0; i < NUMBER_OF_SYSTEMS; i++) {
    lcd.setCursor(0, i+1);

    auto gains = interfaces[i].controller.GetGains();
    lcd.print(String(gains.kp, 3));
    lcd.print("/");
    lcd.print(String(gains.ki, 3));
    lcd.print("/");
    lcd.print(String(gains.kd, 3));
  }
}

void update_sensed_and_targets() {
  #if NUMBER_OF_SYSTEMS == 1
  LOG(1); /* priamry state always == true with one system */
  filter_data(
    &interfaces[0].target_position, 
    map(
      analogRead(interface.command_interface->command_input), 
      0, 
      1023, 
      interface.input_limits[0], 
      interface.input_limits[1]
    )
  );

  filter_data(
    &interfaces[0].actual_position, 
    interfaces[0].position_sensor.measureDistanceCm()
  );
  #else
  system_interface_t *primary_interface;
  system_interface_t *secondary_interface;
  LOG(primary_state);
  if (primary_state) {
    primary_interface = get_interface(FAN);
    secondary_interface = get_interface(BEAM);
  } else {
    primary_interface = get_interface(BEAM);
    secondary_interface = get_interface(FAN);
  }

  /* This should never happen, but in case it does... */
  if (primary_interface == NULL || secondary_interface == NULL) {
    return;
  }

  filter_data(
    &primary_interface->target_position, 
    map(
      analogRead(primary_interface->command_interface->command_input), 
      0, 
      1023, 
      primary_interface->input_limits[0], 
      primary_interface->input_limits[1]
    )
  );

  filter_data(
    &primary_interface->actual_position, 
    primary_interface->position_sensor.measureDistanceCm()
  );

  filter_data(
    &secondary_interface->actual_position, 
    secondary_interface->position_sensor.measureDistanceCm()
  );

  secondary_interface->target_position.filtered = primary_interface->actual_position.filtered;
  #endif
}

void log_data(system_interface_t *interface) {
  auto gains = interface->controller.GetGains();
  auto contributions = interface->controller.GetContributions();  
  LOG(interface->sys);
  LOG(interface->target_position.filtered);
  LOG(interface->actual_position.filtered);
  LOG(gains.kp);
  LOG(gains.ki);
  LOG(gains.kd);
  LOG(contributions.kp);
  LOG(contributions.ki);
  LOG(contributions.kd);
  LOG(contributions.ff);
}

void update_gains(system_interface_t *interface) {
  float kp_ = analogRead(interface->command_interface->gains.kp_in) / 1023.0 * interface->command_interface->max_gains.kp_max;
  float ki_ = analogRead(interface->command_interface->gains.ki_in) / 1023.0 * interface->command_interface->max_gains.ki_max;
  float kd_ = analogRead(interface->command_interface->gains.kd_in) / 1023.0 * interface->command_interface->max_gains.kd_max;
  interface->controller.SetGains(
    kp_, 
    ki_, 
    kd_,
    false
  );
}

double compute_fan_feed_forward(double dist) {
  /* Linear feed-forward. Enforce output is >= 0, since fan is always trying to push ball. */
  return enable_state ? max(0.0, dist*feed_forward_slope + feed_forward_offset) : 0.0;
}

void filter_data(filtered_data_t *data, float new_val) {
  /* Pass input to "raw" field */
  data->raw = new_val;

  if (data->index != moving_average_window_size-1) {
    /* Case: buffer is not full */
    /* Reset filtered output to zero */
    data->filtered = 0.0;                                                 

    /* Emplace current sample to next "empty" index */
    data->buffer[++data->index] = data->raw;

    /* Sum values in buffer, divide by # of samples (index+1) */         
    for (int i = 0; i <= data->index; ++i) {
      data->filtered += data->buffer[i];                  
    }
    data->filtered /= (data->index+1);                           
  } else {
    /* Case: buffer is full */
    /* Assign filter to the current value, since that will not be included in loop below */
    data->filtered = data->raw;          

    /* Shift buffer and sum N most recent values. Divide by window size to compute average */                     
    for (int i = 1; i < moving_average_window_size; ++i) {
      data->buffer[i-1] = data->buffer[i];                
      data->filtered += data->buffer[i-1];
    }
    
    data->filtered /= moving_average_window_size;

    /* Append buffer with current value */
    data->buffer[moving_average_window_size-1] = data->raw;   
  }
}


void toggle_edit_gains() {
  if (gain_input_timer.IsComplete()) {
    gain_state = !gain_state;
    gain_input_timer.Start();
  }
}

void toggle_edit_enable() {
  if (enable_input_timer.IsComplete()) {
    enable_state = !enable_state;
    enable_input_timer.Start();
  }
}

void toggle_edit_primary() {
  if (primary_input_timer.IsComplete()) {
    primary_state = !primary_state;
    primary_input_timer.Start();
  }
}