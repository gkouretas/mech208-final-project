#include <HCSR04.h>
#include <pid.h>
#include <Timer.h>
#include <Servo.h>

/* Helper structs */

// PID pin inputs
typedef struct {
  const int kp_in;
  const int ki_in;
  const int kd_in;
} pid_inputs_t;

typedef struct {
  const int echo;
  const int trigger;
} ultrasonic_pins_t;

// Distance sensor
constexpr int moving_average_window_size        = 30;

// Moving average container
typedef struct {
  uint8_t index;
  float buffer[moving_average_window_size];
  float val;
} filtered_data_t;

// Command interface
typedef struct {
  const pid_inputs_t gains;
  const ultrasonic_pins_t position_input;
  const int command_input;
  const int gain_edit_input;
} input_command_interface_t;

typedef struct {
  UltraSonicDistanceSensor position_sensor;
  PID controller;
  Timer button_debounce_timer;
  float target_position;
  filtered_data_t actual_position;
  int limits[2];
  volatile bool gain_set_state;
  void (*command_handler)(float command);
  input_command_interface_t *command_interface;
} system_interface_t;

/* Macros */

// Struct initializers
#define INIT_COMMAND_INTERFACE(SYSTEM)                                                                                \
  { .gains = { kp_##SYSTEM##_input, ki_##SYSTEM##_input, kd_##SYSTEM##_input },                                       \
    .position_input = { .echo = SYSTEM##_ultrasonic_sensor_echo, .trigger = SYSTEM##_ultrasonic_sensor_trigger }, \
    .command_input = pot_##SYSTEM##_command_input,                                                                    \
    .gain_edit_input = SYSTEM##_gain_state_input_toggle                                                             \
  }

#define INIT_FILTERED_DATA \
  { .index = -1 }

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

// Additional pins
constexpr int fan_pwm_out                        = 8;
constexpr int bean_servo_pin                     = 9;
constexpr int led_disable_mods                   = 53;
constexpr int led_enable_mods                    = 51;

/* Configuration params */
// Fan
constexpr int fan_position_limits_mm[2]         = {50, 400};
#define fan_pid_limits {0, 255}
#define servo_limits   {-59, 59}
constexpr int beam_home_position                = 121;
constexpr float feed_forward_slope              = 0.0;
constexpr float feed_forward_offset             = 30.0;

// Max kp, ki, kd values
constexpr double kp_fan_max = 10.0;
constexpr double ki_fan_max = 0.01;
constexpr double kd_fan_max = 1000.0;

unsigned long ts;
constexpr int loop_rate_ms = 10.0;

static input_command_interface_t fan_command_interface = INIT_COMMAND_INTERFACE(fan);
static input_command_interface_t beam_command_interface = INIT_COMMAND_INTERFACE(beam);

void fan_command_callback(float command) { }
void beam_command_callback(float command) { }

static system_interface_t fan_system_interface = {
  .position_sensor = UltraSonicDistanceSensor(
    fan_command_interface.position_input.trigger,
    fan_command_interface.position_input.echo
  ),
  .controller = PID(
    0, 
    0, 
    0
  ),
  .button_debounce_timer = Timer(1000, MILLISECONDS),
  .target_position = 0,
  .actual_position = { .index = -1 },
  .limits = fan_pid_limits,
  .gain_set_state = true,
  .command_handler = &fan_command_callback,
  .command_interface = &fan_command_interface,
};

static system_interface_t beam_system_interface = {
  .position_sensor = UltraSonicDistanceSensor(
    beam_command_interface.position_input.trigger,
    beam_command_interface.position_input.echo
  ),
  .controller = PID(
    0, 
    0, 
    0
  ),
  .button_debounce_timer = Timer(1000, MILLISECONDS),
  .target_position = 0,
  .actual_position = { .index = -1 },
  .limits = servo_limits,
  .gain_set_state = true,
  .command_handler = &beam_command_callback,
  .command_interface = &beam_command_interface,
};

system_interface_t interfaces[2] = {fan_system_interface, beam_system_interface}; 
Servo beam_servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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

void update_gains(system_interface_t *interface) {
  float kp_ = analogRead(interface->command_interface->gains.kp_in) / 1023.0;
  float ki_ = analogRead(interface->command_interface->gains.ki_in) / 1023.0;
  float kd_ = analogRead(interface->command_interface->gains.kd_in) / 1023.0;
  interface->controller.SetGains(
    kp_, 
    ki_, 
    kd_,
    ki_ == interface->controller.GetGains().ki ? false : true
  );
}

float compute_feed_forward(float dist) {
  // range: 30-330
  return max(0.0, (330-dist)*feed_forward_slope + feed_forward_offset);
}

void filter_data(filtered_data_t *data, float new_val) {
  // Shoutout to Manoj for the suggestion
  if (data->index != moving_average_window_size-1) {
    data->val = 0.0;                                                 // reset filtered output to zero
    /* If moving average buffer is not full, average the values in the buffer */
    data->buffer[++data->index] = new_val;         // append buffer with current value
    for (int i = 0; i <= data->index; ++i) {
      data->val += data->buffer[i];                  // sum values in array
    }
    data->val /= (data->index+1);                           // divide by number of samples
  } else {
    /* Shift buffer and sum N most recent values */
    data->val = new_val;                               // reset filtered output to current value
    for (int i = 1; i < moving_average_window_size; ++i) {
      data->buffer[i-1] = data->buffer[i];                // shift buffer over
      data->val += data->buffer[i-1];
    }
    data->buffer[moving_average_window_size-1] = new_val;   // append buffer with current value
    data->val /= moving_average_window_size;                         // divide by number of samples
  }
}

void loop() {
  ts = millis();
  for (system_interface_t &interface : interfaces) {
    update_gains(&interface);
    interface.target_position = map(analogRead(interface.command_interface->command_input), 0, 1023, interface.limits[0], interface.limits[1]);
    filter_data(&interface.actual_position, interface.position_sensor.measureDistanceCm());
    interface.controller.Step(loop_rate_ms, interface.target_position, interface.actual_position.val);
  }

  delay(loop_rate_ms - (millis() - ts));
}

void toggle_fan_edit_gains()  {}

void toggle_beam_edit_gains() {}