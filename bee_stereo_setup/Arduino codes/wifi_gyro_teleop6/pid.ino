void updatePID() {
  
  static unsigned long lastUpdate = 0;
  unsigned long now = micros();
  unsigned long dt = now - lastUpdate;  // in µs
  if (dt < 2500) return;   // skip invalid dt
  lastUpdate = now;
  
  // targetrollX = constrain(asin(accelX / 9.81) * RAD_TO_DEG * brake_scaleX, -5, 5); // right +ve ||  left -ve 
  // targetpitchY = -constrain(asin(accelY / 9.81) * RAD_TO_DEG * brake_scaleY, -5, 5);  // front move +ve accelY || back move -ve accelY
  // === GYRO X PID ===
  float errorX = targetrollX - rollX;

  if ((errorX > 0 && previous_errorX < 0) || (errorX < 0 && previous_errorX > 0)) {
    integralX = 0.0f;  // Reset on sign change
  }

  integralX += errorX * dt;
  integralX = constrain(integralX, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Use gyro rate directly for derivative term (gyroX is angular velocity)
  float derivativeX = -gyroX;  // Negative because we want to oppose the motion

  correctionX = (int)constrain(
    kpX * errorX +
    kiX * integralX * 1e-6f +      // µs to sec
    kdX * derivativeX,
    -OUTPUT_LIMIT, OUTPUT_LIMIT
  );
  rev_correctionX = correctionX;
  previous_errorX = errorX;

  // === GYRO Y PID ===
  float errorY = targetpitchY - pitchY;

  if ((errorY > 0 && previous_errorY < 0) || (errorY < 0 && previous_errorY > 0)) {
    integralY = 0.0f;
  }

  integralY += errorY * dt;
  integralY = constrain(integralY, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivativeY = -gyroY;

  correctionY = (int)constrain(
    kpY * errorY +
    kiY * integralY * 1e-6f +
    kdY * derivativeY,
    -OUTPUT_LIMIT, OUTPUT_LIMIT
  );
  rev_correctionY = correctionY;
  previous_errorY = errorY;

  // === YAW PID ===
// Based on yaw angle (yawZ) and gyro rate (gyroZ)
float errorZ = targetyawZ - yawZ;

// Reset integral if error crosses zero (optional)
if ((errorZ > 0 && previous_errorZ < 0) || (errorZ < 0 && previous_errorZ > 0)) {
  integralZ = 0.0f;
}

// Integrate yaw angle error over time
integralZ += errorZ * dt;
integralZ = constrain(integralZ, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

// Derivative term uses gyroZ directly (rate of change of yaw angle)
float derivativeZ = -gyroZ;

// Final control output with mixed strategy
yaw_offset = (int)constrain(
  kpZ * errorZ +
  kiZ * integralZ * 1e-6f +
  kdZ * derivativeZ,
  -OUTPUT_LIMIT, OUTPUT_LIMIT
);

previous_errorZ = errorZ;
  // Serial.println("target X Y & angle X Y ---");  
  
  // Serial.print(targetrollX);
  // Serial.print("   --  ");
  // Serial.print(targetpitchY);
  // Serial.print("  |  ");
  // Serial.print(rollX);
  // Serial.print("   --  ");
  // Serial.print(pitchY);
  // Serial.print("  |  ");
  // Serial.print(accelX);
  // Serial.print("   --  ");
  // Serial.println(accelY);

  // Serial.print(correctionX);
  // Serial.print("   --  ");
  // Serial.print(correctionY);
  // Serial.print("  |  ");
  // Serial.print(yaw_offset);
  // Serial.print("   --  ");
  // Serial.print(targetrollX);
  // Serial.print("  |  ");
  // Serial.print(targetpitchY);
  // Serial.print("   --  ");
  // Serial.println(targetyawZ);
}
