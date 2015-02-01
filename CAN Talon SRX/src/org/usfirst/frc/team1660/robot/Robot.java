package org.usfirst.frc.team1660.robot;

//IMPORTING USED CLASSES
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first. wpilibj.Relay; 

//import org.usfirst.frc.team1660.robot.HKdriveClass;
//import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;


public class Robot extends SampleRobot {
	
  //DECLARING MOTORS
  public static CANTalon frontleft;
  public static CANTalon frontright;
  public static CANTalon backleft;
  public static CANTalon backright;
  Talon eaterRight;
  Talon eaterLeft;
  CANTalon lifterRight;
  CANTalon lifterLeft;
  
//DECLARING RELAYS
  Relay leftArmRelay;
  Relay rightArmRelay;
  Relay airComprs;

  
  
  //NAVX GYRO CODE
  IMUAdvanced imu;      //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
  SerialPort serial_port;
  boolean first_iteration;

  public static RobotDrive hkDrive;
  public static Joystick driverStick;
  public static Joystick manipStick;
 
  
  //ROBOT VARIABLES
  double eatSpeed=0.75;
  double spitSpeed=0.50;
  double liftSpeed=0.40; 
  
  
  
  
  public Robot() {
	  try {
	        serial_port = new SerialPort(57600,SerialPort.Port.kMXP);
	                
	                // You can add a second parameter to modify the 
	                // update rate (in hz) from 4 to 100.  The default is 100.
	                // If you need to minimize CPU load, you can set it to a
	                // lower value, as shown here, depending upon your needs.
	                
	                // You can also use the IMUAdvanced class for advanced
	                // features.
	                
	                byte update_rate_hz = 50;
	                //imu = new IMU(serial_port,update_rate_hz);
	                imu = new IMUAdvanced(serial_port,update_rate_hz); 
	        } catch( Exception ex ) {
	        	SmartDashboard.putString("error message", "cant initialize gyro" + ex.getMessage());
	                
	        }
	        if ( imu != null ) {
	            LiveWindow.addSensor("IMU", "Gyro", imu);
	        }
	        first_iteration = true;
	  
	  
	  
	  
	  
	  //INITIALIZE CANTalonSRX
      frontleft  = new CANTalon(6);
      backleft   = new CANTalon(4);
      backright  = new CANTalon(3);
      frontright = new CANTalon(2);
      lifterRight= new CANTalon(5);
      lifterLeft= new CANTalon(1);
      eaterRight = new Talon(1);
      eaterLeft  = new Talon(2);
      
      
      //INITIALIZE RLAYS
      leftArmRelay  = new Relay(1);
      rightArmRelay = new Relay(2);
      airComprs     = new Relay(0);

      
  
}

//////////////////////////////////////////
//OFFICIAL FRC METHODS CALLED EACH MATCH//
/////////////////////////////////////////
  
  
  public void robotInit() {
		hkDrive     = new RobotDrive(frontleft, backleft, frontright, backright);
	    driverStick = new Joystick(1);
	    manipStick  = new Joystick(2);
	    //HKdriveClassObject.zeroYaw();  //calibrate robot gyro to zero when facing away from driver (may need 20 seconds)

	}  
  
  
  public void autonomous(){
	  //3-TOTE-STACK-AUTO
	  
	  
	  
	  
	  
	  
  }
  
  
  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {

    	checkComp();
    	checkJoystick();	
    	processGyro();
    	checkEatingButtons();
    	checkLiftingButtons();
    	checkBiting();
    	
       	Timer.delay(0.01);  // Note that the CANTalon only receives updates every
                            // 10ms, so updating more quickly would not gain you anything.
    
    
    }
  
    //DISABLE ALL ACTUATORS (Motors, Pistons, etc.) AT END OF MATCH
    frontleft.disable();
    frontright.disable();
    backleft.disable();
    backright.disable();
  }
  
  
  


////////////////////////////////////
//CUSTOM METHODS CREATED BY HK1660//
////////////////////////////////////

  
//MOVE DRIVETRAIN WITH XBOX360 JOYSTICKS -Matthew
public void checkJoystick()
{
	
	 double threshold = 0.11;
	 
	 double x = driverStick.getRawAxis(0) ; //up and down on the left thumb stick?
	 double moveValue = driverStick.getRawAxis(1);// right and left on left thumb stick?
	 double rotateValue = driverStick.getRawAxis(4);// right and left on right thumb stick
	
	 //KILL GHOST MOTORS -Matthew & Dianne
	if(moveValue > threshold*-1 && moveValue < threshold) {
		moveValue = 0;
	}
	if(rotateValue > threshold*-1 && rotateValue < threshold) {
		rotateValue = 0;
	}
	if(x > threshold*-1 && x < threshold) {
		x = 0;
	}
	
	//MECANUM -Matthew
	SmartDashboard.putNumber(  "move",        moveValue);
	SmartDashboard.putNumber(  "rotate",        rotateValue);
	//System.out.println("move: "+moveValue+" rotate: "+rotateValue);
	hkDrive.mecanumDrive_Cartesian(rotateValue, moveValue, x, imu.getYaw());
	//HKdriveClassObject.doMecanum(x,moveValue,rotateValue); 
}


//EAT and SPITING WITH XBOX360 -Adonis & Jatara

public void checkEatingButtons(){
	//EATING  &spiiting WITH JOYSTICKS
	
	if (manipStick.getRawButton(6)==true ){  //if holding the A button, 
		

		//then eater motor spin	
		eaterRight.set(eatSpeed);
		eaterLeft.set(-eatSpeed);
	}
	
	else if (manipStick.getRawButton(2)==true ){  //if holding the X button, 
		//then eater motor spin backwards	
		eaterRight.set(-spitSpeed);
		eaterLeft.set(spitSpeed);
	}
	
	
	
	else{
		eaterRight.set(0.0);
		eaterLeft.set(0.0);
	}
}
    



//BITING WITH XBOX360 jamesey 

public void checkBiting(){
	//Bit with LB button
	
	if (manipStick.getRawButton(4)==true ){  //if holding the LB button, 	        
	 leftArmRelay.set(Relay.Value.kForward);                 
		rightArmRelay.set(Relay.Value.kForward);
	}
	
	if (manipStick.getRawButton(5)==true ){  //if holding the RB button, 
		
	leftArmRelay.set(Relay.Value.kReverse);
	rightArmRelay.set(Relay.Value.kReverse);
	}
	
}



//COMPRESSOR ON & OFF WITH JOYSTICKS
public void checkComp(){
if (manipStick.getRawButton(1)==true ){  //if holding the B button	
	 airComprs.set(Relay.Value.kForward);
}                     

 if (manipStick.getRawButton(3)==true ){  //if holding the Y button, 
		airComprs.set(Relay.Value.kReverse);
 }
}


//LIFT WITH XBOX360 -Adonis & Jatara\

public void checkLiftingButtons(){

 double axisValue = manipStick.getRawAxis(1);
 lifterRight.set(axisValue);
 lifterLeft.set(-axisValue);
}
















//SENDS GYRO VALUES TO SMARTDASHBOARD
public void processGyro() {	  
	  
	  boolean is_calibrating = imu.isCalibrating();
    if ( first_iteration && !is_calibrating ) {
        Timer.delay( 0.3 );
        imu.zeroYaw();
        first_iteration = false;
    }
    
    // Update the dashboard with status and orientation
    // data from the nav6 IMU
    
    SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
    SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
    SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
    SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

    // If you are using the IMUAdvanced class, you can also access the following
    // additional functions, at the expense of some extra processing
    // that occurs on the CRio processor
    
    SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
    SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
    SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
    SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
}



//AUTO EAT METHOD
public void autoEat() {
eaterRight.set(eatSpeed);
eaterLeft.set(eatSpeed);
}

//AUTO LIFT METHOD
public void autoLift() {
lifterRight.set(liftSpeed);
lifterLeft.set(liftSpeed);
}

//AUTO DRIVE TO NEXT TOTE METHOD




//AUTO DROP OFF A STACK METHOD








}



