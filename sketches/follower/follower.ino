#include <HCSR04.h>
#include <pid.h>
#include <Timer.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

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
  const int gain_edit_input;
} input_command_interface_t;

typedef enum {
  FAN,
  BEAM
} system_type_t;

typedef struct {
  const system_type_t sys;
  UltraSonicDistanceSensor position_sensor;
  PID controller;
  Timer button_debounce_timer;
  filtered_data_t target_position; // TODO: filter?
  filtered_data_t actual_position;
  int input_limits[2];
  int output_limits[2];
  volatile bool *gain_set_state;
  void (*command_handler)(int command);
  input_command_interface_t *command_interface;
} system_interface_t;

/* Macros */

// Struct initializers
#define INIT_COMMAND_INTERFACE(SYSTEM)                                                                                \
  { .gains = { kp_##SYSTEM##_input, ki_##SYSTEM##_input, kd_##SYSTEM##_input },                                       \
    .max_gains = { kp_##SYSTEM##_max, ki_##SYSTEM##_max, kd_##SYSTEM##_max },                                         \
    .position_input = { .echo = SYSTEM##_ultrasonic_sensor_echo, .trigger = SYSTEM##_ultrasonic_sensor_trigger },     \
    .command_input = pot_##SYSTEM##_command_input,                                                                    \
    .gain_edit_input = SYSTEM##_gain_state_input_toggle                                                               \
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
constexpr int kp_fan_input                       = A0;
constexpr int ki_fan_input                       = A1;
constexpr int kd_fan_input                       = A2;
constexpr int pot_fan_command_input              = A3;
constexpr int fan_gain_state_input_toggle        = 18;
constexpr int fan_ultrasonic_sensor_trigger      = 12;
constexpr int fan_ultrasonic_sensor_echo         = 13;
constexpr int kp_beam_input                      = A4;
constexpr int ki_beam_input                      = A5;
constexpr int kd_beam_input                      = A6;
constexpr int pot_beam_command_input             = A7;
constexpr int beam_gain_state_input_toggle       = 19;
constexpr int beam_ultrasonic_sensor_trigger     = 14;
constexpr int beam_ultrasonic_sensor_echo        = 15;

// Max kp, ki, kd values
constexpr double kp_fan_max = 25.0;
constexpr double ki_fan_max = 1.0;
constexpr double kd_fan_max = 100.0;
constexpr double kp_beam_max = 10.0;
constexpr double ki_beam_max = 1.0;
constexpr double kd_beam_max = 100.0;

// Additional pins
constexpr int fan_pwm_out                        = 8;
constexpr int bean_servo_pin                     = 9;
constexpr int led_disable_mods                   = 53;
constexpr int led_enable_mods                    = 51;

// Button debounce
constexpr float button_debounce_duration_ms      = 1000.0;

/* Configuration params */
// Fan
#define fan_pid_limits        {0, 255}
#define servo_limits          {-59, 59}
#define fan_position_limits   {5, 30}
#define beam_position_limits  {5, 30}

// Beam
Servo beam_servo;
constexpr int beam_home_position                = 94;
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
volatile bool fan_state = false;
volatile bool beam_state = false;

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
  .button_debounce_timer = Timer(button_debounce_duration_ms, MILLISECONDS),
  .target_position = { .index = -1 },
  .actual_position = { .index = -1 }, // TODO: why does macro init fail?
  .input_limits = fan_position_limits,
  .output_limits = fan_pid_limits,
  .gain_set_state = &fan_state,
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
  .button_debounce_timer = Timer(button_debounce_duration_ms, MILLISECONDS),
  .target_position = { .index = -1 },
  .actual_position = { .index = -1 }, // TODO: why does macro init fail?
  .input_limits = beam_position_limits,
  .output_limits = servo_limits,
  .gain_set_state = &beam_state,
  .command_handler = &beam_command_callback,
  .command_interface = &beam_command_interface,
};

system_interface_t interfaces[2] = {fan_system_interface, beam_system_interface};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // LCD setup
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Device Addr = 0x27");
  
  // Initialize interfaces
  for (system_interface_t &interface : interfaces) {
    pinMode(interface.command_interface->gain_edit_input, INPUT_PULLUP);
    interface.button_debounce_timer.Start();
  }

  // Attach interrupts. Has to be done individually w/ current setup since debounce timers are decoupled...
  attachInterrupt(digitalPinToInterrupt(fan_system_interface.command_interface->gain_edit_input), toggle_fan_edit_gains, FALLING);
  attachInterrupt(digitalPinToInterrupt(beam_system_interface.command_interface->gain_edit_input), toggle_beam_edit_gains, FALLING);

  // Set servo to home position
  beam_servo.attach(bean_servo_pin);
  beam_servo.write(beam_home_position);
}

void loop() {
  ts = millis();
  double target_position;
  bool first_run = true;

  for (system_interface_t &interface : interfaces) {
    /* Update gains if state allows it */
    if (*interface.gain_set_state) {
      update_gains(&interface);
    }
    
    /* Get target position, either from user input or from previous system */
    if (first_run) {
      /* Read target position from input, filter to avoid discontinuities */
      filter_data(
        &interface.target_position, 
        map(
          analogRead(interface.command_interface->command_input), 
          0, 
          1023, 
          interface.input_limits[0], 
          interface.input_limits[1]
        )
      );
      first_run = false;
    } else {
      /* Pass previous system's position */
      interface.target_position.filtered = target_position;
    }

    /* Get actual position, put through moving average filter */
    filter_data(
      &interface.actual_position, 
      interface.position_sensor.measureDistanceCm()
    );

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

    /* Set target position for next system to the actual position of this one */
    target_position = interface.actual_position.filtered;

    /* Log data */
    log_data(&interface);
  }

  LOGEOL(ts);

  /* Log timestamp, start newline in prep for next packet */
  unsigned long duration = (millis() - ts);
  if (duration < loop_rate_ms)
    delay(loop_rate_ms - duration);

  /* Display gains */
  lcdPrint();
}

void lcdPrint() {
  auto gains = interface->controller.GetGains();
  
  lcd.setCursor(0, 0);
  lcd.print("   Kp/   Ki/   Kd");

  system_interface_t *interface;
  for (int i=0; i<2; i++) {
    lcd.setCursor(0, i+1);

    interface = &(interfaces[i]);
    auto gains = interface->controller.GetGains();
    lcd.print(String(gains.kp, 5));
    lcd.print("/");
    lcd.print(String(gains.ki, 5));
    lcd.print("/");
    lcd.print(String(gains.kd, 5));
  }
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
  return max(0.0, dist*feed_forward_slope + feed_forward_offset);
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


void _handle_gain_interrupt(system_interface_t *interface) {
  if (interface->button_debounce_timer.IsComplete()) {
    *interface->gain_set_state = !(*interface->gain_set_state);
    interface->button_debounce_timer.Start();
  }
}

void toggle_fan_edit_gains() {
  _handle_gain_interrupt(&fan_system_interface);
}

void toggle_beam_edit_gains() {
  _handle_gain_interrupt(&beam_system_interface);
}
