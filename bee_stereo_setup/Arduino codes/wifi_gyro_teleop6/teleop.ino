// Call this every control loop to update targets
void updateTargetTilt(float forward_cmd, float right_cmd,float heading) {
  // Inputs:
  // forward_cmd: +ve for forward, -ve for backward
  // right_cmd:   +ve for right,   -ve for left

  const float max_tilt_deg = 20.0;  // Max safe tilt angle in degrees
  // Serial.print(forward_cmd);
  //     Serial.print("     ");
  //     Serial.println(right_cmd);
  // Clamp commands between -1.0 and 1.0 for normalized input
  forward_cmd = constrain(forward_cmd, -1.0f, 1.0f);
  right_cmd   = -constrain(right_cmd,   -1.0f, 1.0f);
  // targetyawZ = heading;
  // if(forward_cmd == 0 && right_cmd == 0){
  //   targetrollX= constrain(asin((accelX / 9.81) * RAD_TO_DEG)*brake_scaleX, -5 , 5) ; //position hold pr brake to any drift
  //   targetpitchY = constrain(asin((accelY / 9.81) * RAD_TO_DEG)*brake_scaleY, -5, 5)  ;
  // }
  // Map normalized input to tilt angles
  targetpitchY = constrain(forward_cmd * max_tilt_deg, -max_tilt_deg, max_tilt_deg);  // Forward/backward
  targetrollX = constrain(right_cmd   * max_tilt_deg, -max_tilt_deg, max_tilt_deg);  // Left/right
}

// void updateAltitudeHold(float accelZ, float dt) {
//   if (!altitude_hold) return;  // Do nothing if hold is OFF

//   // 1. Compute error (Z acceleration deviation from gravity)
//   float alt_error = 9.81 - accelZ;  // positive if falling

//   // 2. Integrate error (optional)
//   alt_integral += alt_error * dt;
//   alt_integral = constrain(alt_integral, -1.0f, 1.0f);  // limit windup

//   // 3. Derivative term
//   float derivative = (alt_error - alt_prev_error) / dt;
//   alt_prev_error = alt_error;

//   // 4. Output adjustment (acts like throttle offset)
//   float throttle_adjust = alt_kp * alt_error + alt_ki * alt_integral + alt_kd * derivative;

//   // 5. Apply to throttle `r`
//   r = constrain(r + (int)throttle_adjust, 0, 600);  // Or whatever your safe max is
// }

// void applyAltitudeBraking(float accelZ) {
//   if (!altitude_hold) return;

//   float vertical_error = 9.81 - accelZ;  // +ve if falling, -ve if rising
//   throttle_adjust = (int)(alt_kd * vertical_error);

//   // r = constrain(r + throttle_adjust, 0, 600);  // apply adjustment to throttle
// }
