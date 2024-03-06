constexpr int pot_target_position = A0;
constexpr int left_fan_pwm_out    = 8;
constexpr int right_fan_pwm_out   = 9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int position = map(analogRead(pot_target_position), 0, 1023, 0, 255);
  analogWrite(left_fan_pwm_out, abs(255 - position));
  analogWrite(right_fan_pwm_out, abs(position - 255));
}
