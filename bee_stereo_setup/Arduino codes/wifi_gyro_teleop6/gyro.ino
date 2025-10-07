void readGyro() {

    if (mpu.update()) {

            rollX = mpu.getRoll()-179.9+0.72+1.9;
            if(rollX < -60){
                rollX = rollX + 360;
            }
            // rollX = filterX.update(rollX);
            // pitchY = filterY.update(mpu.getPitch());
            // yawZ = filterZ.update(mpu.getYaw()-0.18);
            pitchY = mpu.getPitch()+0.31;
            yawZ = mpu.getYaw();

            // rollX  = alpha * raw_rollX  + (1.0 - alpha) * rollX;
            // pitchY = alpha * raw_pitchY + (1.0 - alpha) * pitchY;
            // yawZ   = alpha * raw_yawZ   + (1.0 - alpha) * yawZ;

            gyroX = mpu.getGyroX()+2.5;
            gyroY = mpu.getGyroY();
            gyroZ = mpu.getGyroZ()+.8;
            // gyroX  = alpha * raw_gyroX  + (1.0 - alpha) * gyroX;
            // gyroY = alpha * raw_gyroY + (1.0 - alpha) * gyroY;
            // Using quaternion q (w,x,y,z)
            // float qw = mpu.getQuaternionW();
            // float qx = mpu.getQuaternionX();
            // float qy = mpu.getQuaternionY();
            // float qz = mpu.getQuaternionZ();

            // gravity vector in sensor frame:
            // float gX = 2*(qx*qz - qw*qy);
            // float gY = 2*(qw*qx + qy*qz);
            // float gZ = qw*qw - qx*qx - qy*qy + qz*qz;

            // accelX = mpu.getLinearAccX();
            // accelY = mpu.getLinearAccY();
            // accelZ = mpu.getLinearAccZ();
            // accelY = mpu.getAccX()+gX;
            // accelX = mpu.getAccY()-gY;
            // accelZ = mpu.getAccZ();
            
            // Serial.print("  Vx: "); Serial.print(vel.x);
            // Serial.print("  Vy: "); Serial.println(vel.y);
            // Serial.print("Vz: "); Serial.println(vel.z);
            

            accelY = mpu.getAccX();
            accelX = mpu.getAccY()+0.01;
            // accelZ = mpu.getAccZ();
            //printing info
            // Serial.println("Roll, Pitch, Yaw ");
            // Serial.print(rollX);
            // Serial.print(", ");
            // Serial.print(pitchY);
            // Serial.print(", ");
            // Serial.println(yawZ);
            // Serial.println("Gyro X ,Y ,Z  ");
            // Serial.print(gyroX);
            // Serial.print(" ---- ");
            // Serial.print(gyroY);
            // Serial.print(" , ");
            // Serial.println(gyroZ);
            // Serial.println("Accel X ,Y ,Z  ");
            // Serial.print(accelX);
            // Serial.print("  ");
            // Serial.print(accelX + gX);
            // Serial.print(" , ");
            // Serial.print(accelY);
            // Serial.print("  ");
            // Serial.print(accelY - gY);
            // Serial.print(" , ");
            // Serial.println(accelZ);
            // Serial.print("  ");
            // Serial.println(accelZ - gZ);
            // prev_ms = millis();
        
    }

}