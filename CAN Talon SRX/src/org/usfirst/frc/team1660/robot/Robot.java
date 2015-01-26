package org.usfirst.frc.team1660.robot;

//IMPORTING USED CLASSES
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import org.usfirst.frc.team1660.robot.HKdriveClass;
//import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;


public class Robot extends SampleRobot {
	
  //ASSIGNING MOTORS TO THE CANTalons
  public static CANTalon frontleft;
  public static CANTalon frontright;
  public static CANTalon backleft;
  public static CANTalon backright;

  //NAVX GYRO CODE
  public static IMUAdvanced imu;      //IMUAdvanced imu;  // Alternatively, use IMUAdvanced for advanced features
  SerialPort serial_port;
  boolean first_iteration;

  public static RobotDrive hkDrive;
  public static Joystick driverStick;
  HKdriveClass HKdriveClassObject;
  
  
  public void robotInit() {
		hkDrive = new RobotDrive(frontleft, backleft, frontright, backright);
	    driverStick = new Joystick(1);
	}  
  
  
  public Robot() {
	  
	  //INITIALIZE CANTalonSRX's FOR DRIVETRAIN
      frontleft = new CANTalon(1);
      backleft = new CANTalon(2);
      backright = new CANTalon(3);
      frontright = new CANTalon(4);
      
      //INITIALIZE GYRO SENSOR
      IMUAdvanced imu = HKdriveClass.imu;
  }
  

  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {

    	checkJoystick();	
    	
    	Timer.delay(0.01);  // Note that the CANTalon only receives updates every
                            // 10ms, so updating more quickly would not gain you anything.
    
		//DISPLAY NAVX GYRO DATA ON SMARTDASHBOARD
	    SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
	    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
		SmartDashboard.putNumber(   "IMU_Yaw", 				imu.getYaw());
		//SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
		//SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
		//SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
		//SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
		//SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());	  
	    //Timer.delay(0.2);	
    
    }
    
    frontleft.disable();
    frontright.disable();
    backleft.disable();
    backright.disable();
  }



////////////////////////////////////
//CUSTOM METHODS CREATED BY HK1660//
////////////////////////////////////

//DRIVE WITH JOYSTICK -Matthew
public void checkJoystick()
{
	 double threshold = 0.11;
	 
	 double x = driverStick.getRawAxis(0) ;
	 double moveValue = driverStick.getRawAxis(1);
	 double rotateValue = driverStick.getRawAxis(4);
	
	 //KILL GHOST MOTORS
	if(moveValue > threshold*-1 && moveValue < threshold) {
		moveValue = 0;
	}
	if(rotateValue > threshold*-1 && rotateValue < threshold) {
		rotateValue = 0;
	}
	
	if(x > threshold*-1 && x < threshold) {
		x = 0;
	}
	
	//MECANUM
	System.out.println("move: "+moveValue+" rotate: "+rotateValue);
	hkDrive.mecanumDrive_Cartesian(rotateValue, moveValue, x, 0);
	HKdriveClassObject.zeroYaw();
	//HKdriveClassObject.doMecanum(x,moveValue,rotateValue); 
	
  }







}



