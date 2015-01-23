package org.usfirst.frc.team1660.robot;


import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.nav6.frc.BufferingSerialPort;
import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;





/**
 * This is a short sample program demonstrating how to use the basic throttle
 * mode of the new CAN Talon.
 *  Testing commit
 */
public class hkRobot extends SampleRobot {

	//Code forthe NavX Gyro
    BufferingSerialPort serial_port;
    IMUAdvanced imu;      //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
    //SmartDashboard dash;
    boolean first_iteration;

    //Assigning motor names to CANTalons
  CANTalon motor1;
  CANTalon motor2;
  CANTalon motor3;
  CANTalon motor4;

  public hkRobot() {
	  
	  //Code for the NavX Gyro

      try {
    	  serial_port = new SerialPort(57600,SerialPort.Port.kMXP);                       
    	  byte update_rate_hz = 50;
          //imu = new IMU(serial_port,update_rate_hz);
          imu = new IMUAdvanced(serial_port,update_rate_hz);
      } catch( Exception ex ) {       
      }
      if ( imu != null ) {
          LiveWindow.addSensor("IMU", "Gyro", imu);
      }
      first_iteration = true;
  
  
      	SmartDashboard.putNumber(   "IMU_Yaw", 			imu.getYaw());
  		SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
       	SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
	  
	  
      motor1 = new CANTalon(1); // Initialize the CanTalonSRX on device 1.
      motor2 = new CANTalon(2);
      motor3 = new CANTalon(3);
      motor4 = new CANTalon(4);
  }

  /**
    * Runs the motor.
    */
  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {
      // Set the motor's output to half power.
      // This takes a number from -1 (100% speed in reverse) to +1 (100% speed
      // going forward)
      motor1.set(0.5);
      motor2.set(0.5);
      motor3.set(0.5);
      motor4.set(0.5);

      Timer.delay(0.01);  // Note that the CANTalon only receives updates every
                          // 10ms, so updating more quickly would not gain you
                          // anything.
    }
    motor1.disable();
    motor2.disable();
    motor3.disable();
    motor4.disable();
  }
}

