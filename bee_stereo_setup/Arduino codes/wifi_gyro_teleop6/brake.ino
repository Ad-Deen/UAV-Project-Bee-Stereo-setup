float rampValue(float current, int input, float ramp_rate_per_sec ) {
  const float dt = 0.0025;  // Fixed 2.5ms
  float step = ramp_rate_per_sec * dt;

  if (input == 0) {
    // Ramp down toward 0
    if (current > step) current -= step;
    else if (current < -step) current += step;
    else current = 0.0;
  } else if (input == 1) {
    if (current < 1.0) current = min(2.0f, current + step);
  } else if (input == -1) {
    if (current > -1.0) current = max(-2.0f, current - step);
  }

  return current;
}