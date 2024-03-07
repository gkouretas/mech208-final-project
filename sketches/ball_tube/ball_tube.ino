#include <HCSR04.h>
#include <pid.h>
#include <Timer.h>

constexpr int kp_in                         = A0;
constexpr int ki_in                         = A1;
constexpr int kd_in                         = A2;
constexpr int pot_target_position           = A3;
constexpr int fan_pwm_out                   = 9;
constexpr int fan_ultrasonic_sensor_trigger = 12;
constexpr int fan_ultrasonic_sensor_echo    = 13;
constexpr int button_enable_pid_mod         = 18;
constexpr int led_disable_mods              = 53;
constexpr int led_enable_mods               = 51;
constexpr int position_limits_mm[2]         = {50, 400};
constexpr int moving_average_window_size    = 30;
constexpr float feed_forward_slope          = 0.0;
constexpr float feed_forward_offset         = 30.0;

typedef struct {
  uint8_t index;
  float buffer[moving_average_window_size];
  float val;
} filtered_data_t;

filtered_data_t actual_dist = {.index = -1};

#define NEWLINE()                   Serial.println()
#define LOG(buf) {   \
  Serial.print(buf); \
  Serial.print(" "); \
}

#define LOGEOL(buf)                 Serial.println(buf);

constexpr double kp_max = 10.0;
constexpr double ki_max = 0.01;
constexpr double kd_max = 1000.0;

volatile bool edit_state = true;

unsigned long last_ts;
unsigned long ts;


UltraSonicDistanceSensor fan_distance_sensor(
  fan_ultrasonic_sensor_trigger, 
  fan_ultrasonic_sensor_echo,
  400
);  // Initialize sensor that uses digital pins 13 and 12.

PID controller(0, 0, 0);

Timer button_debounce(1000, MILLISECONDS);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(button_enable_pid_mod, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_enable_pid_mod), toggle_edit_gains, FALLING);

  pinMode(led_disable_mods, OUTPUT);
  pinMode(led_enable_mods, OUTPUT);
  button_debounce.Start();
}

void update_gains() {
  float kp_ = analogRead(kp_in) / 1023.0 * kp_max;
  float ki_ = analogRead(ki_in) / 1023.0 * ki_max;
  float kd_ = analogRead(kd_in) / 1023.0 * kd_max;
  controller.SetGains(
    kp_, 
    ki_, 
    0,
    ki_ == controller.GetGains().ki ? false : true
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
  /* Compute distance */
  filter_data(&actual_dist, fan_distance_sensor.measureDistanceCm() * 10.0);
  int target_dist = map(analogRead(pot_target_position), 0, 1023, position_limits_mm[1], position_limits_mm[0]);

  /* Update gains based on pot reading */
  if (edit_state) {
    update_gains();
  }

  controller.Step(10, target_dist, (int)(actual_dist.val));
  last_ts = millis();

  /* Get PWM */
  int ff = compute_feed_forward(actual_dist.val);
  int pwm = controller.GetClampedOutput(0.0, 255.0 - ff) + ff;

  /* Send PWM signal to fan */
  analogWrite(fan_pwm_out, map(analogRead(kp_in), 0, 1023, 0, 255));
  digitalWrite(led_enable_mods, !edit_state);
  digitalWrite(led_disable_mods, edit_state);

  /* Serial console */
  // LOG((int)(actual_dist.val));
  // LOG(target_dist);
  LOGEOL(map(analogRead(kp_in), 0, 1023, 0, 255));
  // LOG(controller.GetKiContribution());
  // LOG(controller.GetKdContribution());
  // LOGEOL(ff);

  delay(10); // 10 ms delay, hard coded for now...
}

void toggle_edit_gains() { 
  if (button_debounce.IsComplete()) {
    edit_state = !edit_state;
    button_debounce.Start();
  }
}