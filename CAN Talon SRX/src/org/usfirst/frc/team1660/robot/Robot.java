package org.usfirst.frc.team1660.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.kauailabs.nav6.frc.IMU; 
//import com.kauailabs.nav6.frc.IMUAdvanced;

/**
 * This is a short sample program demonstrating how to use the basic throttle
 * mode of the new CAN Talon.
 *  Testing commit
 */
public class Robot extends SampleRobot {
	
	RobotDrive hkDrive;
	Joystick driverStick;
	
	public void robotInit() {
		hkDrive = new RobotDrive(frontleft, backleft, frontright, backright);
	    driverStick = new Joystick(1);
	}

	//Code forthe NavX Gyro
  //  SerialPort serial_port;
    //IMUAdvanced imu;      //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
    //SmartDashboard dash;
    boolean first_iteration;

    //Assigning motor names to CANTalons
  CANTalon frontleft;
  CANTalon frontright;
  CANTalon backleft;
  CANTalon backright;

  public Robot() {
	  
	  //Code for the NavX Gyro
/**
    
	   	try {
	    	serial_port = new SerialPort(57600,SerialPort.Port.kOnboard);
			
			// You can add a second parameter to modify the 
			// update rate (in hz) from 4 to 100.  The default is 100.
			// If you need to minimize CPU load, you can set it to a
			// lower value, as shown here, depending upon your needs.
			
			// You can also use the IMUAdvanced class for advanced
			// features.
			
			byte update_rate_hz = 50;
			imu = new IMU(serial_port,update_rate_hz);
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
	  
	**/  
      frontleft = new CANTalon(1); // Initialize the CanTalonSRX on device 1.
      backleft = new CANTalon(2);
      backright = new CANTalon(3);
      frontright = new CANTalon(4);
  }

  /**
    * Runs the motor.
    */
  
 public void checkJoystick()
 {
	 double threshold = 0.11;
	 
	 double x = driverStick.getRawAxis(4) ;
	 double moveValue = driverStick.getRawAxis(1);
	 double rotateValue = driverStick.getRawAxis(0);
	
	if(moveValue > threshold*-1 && moveValue < threshold) {
		moveValue = 0;
	}
	if(rotateValue > threshold*-1 && rotateValue < threshold) {
		rotateValue = 0;
	}
	
	if(x > threshold*-1 && x < threshold) {
		x = 0;
	}
	
	System.out.println("move: "+moveValue+" rotate: "+rotateValue);
	
	hkDrive.mecanumDrive_Cartesian(rotateValue, moveValue, x, 0);
	 
 }
 
  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {
    	checkJoystick();	
    	
      Timer.delay(0.01);  // Note that the CANTalon only receives updates every
                          // 10ms, so updating more quickly would not gain you
                          // anything.
    }
    frontleft.disable();
    frontright.disable();
    backleft.disable();
    backright.disable();
  }
}

