package org.usfirst.frc.team1660.robot;

//IMPORTING USED CLASSES
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Relay; 
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.CANTalon.ControlMode;

//import org.usfirst.frc.team1660.robot.HKdriveClass;
//import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;


public class Robot extends SampleRobot {
	
  //DECLARING JOYSTICK VARIABLES   -jamesey
	int FORWARDBACKWARD_AXIS = 1; //Left joystick up and down
	int TURNSIDEWAYS_AXIS = 4; //Right joystick side to side
	int LIFTDROP_AXIS = 1; //Left joystick up and down
	
	int EAT_BUTTON = 1; //A
	int SPIT_BUTTON = 3; //X
	int OPEN_BUTTON = 5; //LB
	int CLOSE_BUTTON = 6; //RB
	int STRAFE_AXIS = 0; //Left joystick side to side
	int COMPRESSER_ON_BUTTON = 8; //Start
	int COMPRESSER_OFF_BUTTON= 7; //Back
	int CHANGE_BUTTON = 4; //Y
	int DPAD_UP = 12;
	int DPAD_DOWN =13;
	
	
  //DECLARING MOTORS
  public static CANTalon frontleft;
  public static CANTalon frontright;
  public static CANTalon backleft;
  public static CANTalon backright;
  Talon eaterRight;
  Talon eaterLeft;
  CANTalon lifterFollower;
  CANTalon lifterLeft;
  
  //DECLARING RELAYS
  Relay tuskRelay;
  Relay airComprs;

  //DECLARING SENSORS
  DigitalInput pressureSwitch;
  DigitalInput limitTote;
  DigitalInput limitContainer;
  DigitalInput limitTopR;
  DigitalInput limitTopL;
  DigitalInput limitBottomR;
  DigitalInput limitBottomL;

  
  //DECLARING TIMERS
  Timer rumbleTimer;
  
    
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

  boolean rumbleToggle = false;
  public boolean SINGLE_CONTROLLER = false; //start by using only 1 xbox controller, touch button to add manipStick
  
  //SmartDashboard objects
  SendableChooser startingPosition;
  SendableChooser strategy;
  
  
  
  public Robot() {
	  
	  //INITIALIZE GYRO
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
	  	  
	  
	  //INITIALIZE SPEED CONTROLLERS
      frontleft  = new CANTalon(6);
      backleft   = new CANTalon(4);
      backright  = new CANTalon(3);
      frontright = new CANTalon(2);
      
      lifterLeft= new CANTalon(1);
      //lifterLeft.changeControlMode(ControlMode.PercentVbus);
      lifterFollower= new CANTalon(5);
      //lifterFollower.changeControlMode(ControlMode.Follower);
      //lifterFollower.set(1);
      //lifterFollower.reverseOutput(true);

      eaterRight = new Talon(1);
      eaterLeft  = new Talon(2);
      
      
      
      //INITIALIZE RELAYS   jamesey
      airComprs  = new Relay(0);
      tuskRelay  = new Relay(1);


      //INITIALIZE SENSORS    
      pressureSwitch = new DigitalInput(0);
      limitTote = new DigitalInput (1);
      limitContainer = new DigitalInput (2);
      limitBottomR = new DigitalInput (3);
      limitTopR = new DigitalInput (4);
      limitBottomL = new DigitalInput (5);
      limitTopL = new DigitalInput (6);
     
      
}

//////////////////////////////////////////
//OFFICIAL FRC METHODS CALLED EACH MATCH//
/////////////////////////////////////////
  
  public void robotInit() {
		
	  	SINGLE_CONTROLLER = true; //start by using only 1 xbox controller, touch button to add manipStick
	    
	  	hkDrive     = new RobotDrive(frontleft, backleft, frontright, backright);
	    driverStick = new Joystick(1);
	    manipStick  = new Joystick(2);
	    //HKdriveClassObject.zeroYaw();  //calibrate robot gyro to zero when facing away from driver (may need 20 seconds)
	    airComprs.set(Relay.Value.kForward);
	    
        startingPosition = new SendableChooser();
        startingPosition.addDefault("Left", new Integer(1));
        startingPosition.addObject("Middle", new Integer(2));
        startingPosition.addObject("Right", new Integer(3));
        SmartDashboard.putData("startingPosition", startingPosition);
        
        strategy = new SendableChooser();
        strategy.addDefault("Strategy 1", new Integer(1));
        strategy.addObject("Strategy 2", new Integer(2));
        strategy.addObject("Strategy 3", new Integer(3));
        SmartDashboard.putData("strategy", strategy);

        
	}  
  
  
  public void autonomous(){
	  
	  
	  
	  
	  
	  
	  
  }
  
  
  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {

    	checkSingle();
    	checkComp();
    	//checkCompPressureSwitch();
    	processGyro();
    	checkJoystick();	
    	checkEatingButtons();
       	checkTusks();
    	checkLiftingButtons();
    	checkRumble();
    	
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
	 
	 double x = driverStick.getRawAxis(STRAFE_AXIS) ; // right and left on the left thumb stick?
	 double moveValue = driverStick.getRawAxis(FORWARDBACKWARD_AXIS);// up and down on left thumb stick?
	 double rotateValue = driverStick.getRawAxis(TURNSIDEWAYS_AXIS);// right and left on right thumb stick
	
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
	SmartDashboard.putNumber(  "Strafe",        x);

	hkDrive.mecanumDrive_Cartesian(moveValue, rotateValue, x, 0);
	//HKdriveClassObject.doMecanum(x,moveValue,rotateValue); 
}


//EAT and SPITING WITH XBOX360 -Adonis & Jatara
public void checkEatingButtons(){

	boolean hitT = limitTote.get();
	boolean hitC = limitContainer.get();
	
	if(   SINGLE_CONTROLLER == false      )
	{
			//manipStick Code
			if (manipStick.getRawButton(EAT_BUTTON)==true && hitC == false && hitT == false )
			{  //if holding the A button, 
				//then eater motor spin	
				eaterRight.set(-eatSpeed);
				eaterLeft.set(eatSpeed);
				SmartDashboard.putString(  "Eater",        "Eating");
			}
			else if (manipStick.getRawButton(SPIT_BUTTON)==true ){  //if holding the X button, 
				//then eater motor spin backwards	
				eaterRight.set(spitSpeed);
				eaterLeft.set(-spitSpeed);
				SmartDashboard.putString(  "Eater",        "Spitting");

			}
			else{
				eaterRight.set(0.0);
				eaterLeft.set(0.0);
				SmartDashboard.putString(  "Eater",        "Hungry");
			}
	}
	
	else{
			//driverStick  jamesey
		if (driverStick.getRawButton(EAT_BUTTON)==true && hitC == false && hitT == false ){
				eaterRight.set(-eatSpeed);
				eaterLeft.set(eatSpeed);
				SmartDashboard.putString(  "Eater",        "Eating");
		}
		else if (driverStick.getRawButton(SPIT_BUTTON)==true ){  //if holding the X button, 
				//then eater motor spin backwards	
				eaterRight.set(spitSpeed);
				eaterLeft.set(-spitSpeed);
				SmartDashboard.putString(  "Eater",        "Spitting");
		}	
		else{
				eaterRight.set(0.0);
				eaterLeft.set(0.0);
				SmartDashboard.putString(  "Eater",        "Hungry");
			}
	}
	
}
    

//INTAKE WITH XBOX360 jamesey 
public void checkTusks(){
	//manipStick
	if(   SINGLE_CONTROLLER == false   ){
		if (manipStick.getRawButton(OPEN_BUTTON)==true ){  //if holding the LB button, 	        
		 tuskRelay.set(Relay.Value.kForward);                 
		}
		
		if (manipStick.getRawButton(CLOSE_BUTTON)==true ){  //if holding the RB button, 
		tuskRelay.set(Relay.Value.kReverse);
		}
	}
		
	//driveStick jamesey
	else{
 
		if (driverStick.getRawButton(OPEN_BUTTON)==true ){  //if holding the LB button, 	        
		 tuskRelay.set(Relay.Value.kForward);                 
		}
		
		if (driverStick.getRawButton(CLOSE_BUTTON)==true ){  //if holding the RB button, 	
		tuskRelay.set(Relay.Value.kReverse);
		}
	}
}

//COMPRESSOR ON & OFF WITH PRESSURE SWITCH
public void checkCompPressureSwitch(){

	if (pressureSwitch.get()) {
        airComprs.set(Relay.Value.kForward);
        SmartDashboard.putString("Compressor", "Switched ON");
	} 
   else {
        airComprs.set(Relay.Value.kOff);
        SmartDashboard.putString("Compressor", "Switched OFF");
   }      
}


//COMPRESSOR ON & OFF WITH JOYSTICKS -jamesey
public void checkComp(){
	//manipStick
	if(   SINGLE_CONTROLLER == false   ){
	
		if (manipStick.getRawButton(COMPRESSER_ON_BUTTON)==true ){  //if holding the start button	
			airComprs.set(Relay.Value.kForward);
			SmartDashboard.putString(  "Compressor",        "Button ON");

		}                     		
		 if (manipStick.getRawButton(COMPRESSER_OFF_BUTTON)==true ){  //if holding the back button, 
			airComprs.set(Relay.Value.kOff);
			SmartDashboard.putString(  "Compressor",        "Button OFF");
		 }
 
	// driverStick	 
	 else{  
		if (driverStick.getRawButton(COMPRESSER_ON_BUTTON)==true ){  //if holding the start button	
			 airComprs.set(Relay.Value.kForward);
			SmartDashboard.putString(  "Compressor",        "ON");

		}                    
		
		if (driverStick.getRawButton(COMPRESSER_OFF_BUTTON)==true ){  //if holding the back button, 
				airComprs.set(Relay.Value.kOff);}
				SmartDashboard.putString(  "Compressor",        "OFF");
	 	}
	}
}


//LIFT WITH XBOX360 -Adonis & Jatara\
public void liftingSettings(double liftingSpeed) {
	lifterLeft.set(liftingSpeed);
	lifterFollower.set(liftingSpeed);
}

public void checkLiftingButtons(){
	//Encoder values
	SmartDashboard.putNumber("encoderLValue", lifterLeft.getPosition());
	SmartDashboard.putNumber("encoderRValue", lifterFollower.getPosition());
	SmartDashboard.putNumber("encLposition", lifterLeft.getEncPosition());
	SmartDashboard.putNumber("encRposition", lifterFollower.getEncPosition());
	SmartDashboard.putNumber("encLspeed", lifterLeft.getEncVelocity());
	SmartDashboard.putNumber("encRspeed", lifterFollower.getEncVelocity());
	
	double axisValue = 0;
	
	//manipStick
	if(   SINGLE_CONTROLLER == false   ){
		axisValue = manipStick.getRawAxis(LIFTDROP_AXIS); // left joystick up and down
	}
	
	/**
	 //driversStick  jamesey
	else{
		if(driverStick.getPOV(DPAD_UP)==0) {
			axisValue = 1.0;
		}	
		if(driverStick.getPOV(DPAD_DOWN)==180) {
			axisValue = -1.0;
		}
	}
	**/
	
	boolean hitTL = limitTopL.get();
	boolean hitBL = limitBottomL.get();
	boolean hitTR = limitTopR.get();
	boolean hitBR = limitBottomR.get();
	
	//limit switch stops
	if(hitTR==false && axisValue < 0 ){
		//liftingSettings(0);		
		lifterLeft.set(0);
		lifterFollower.set(0);
	}
	else if(hitBR==false && axisValue > 0 ){
		//liftingSettings(0);		
		lifterLeft.set(0);
		lifterFollower.set(0);
		//RESET THE RIGHT ENCODER HERE
		
		
		
	}
	if(hitTL==false && axisValue > 0){
		//liftingSettings(0);		
		lifterLeft.set(0);
		lifterFollower.set(0);
	}
	else if(hitBL==false && axisValue < 0) {
		//liftingSettings(0);		
		lifterLeft.set(0);
		lifterFollower.set(0);
		//RESET THE LEFT ENCODER HERE
		
		
		
	}
	
	else{
		lifterLeft.set(axisValue*-.85);
		lifterFollower.set(axisValue);
	}
	
	SmartDashboard.putNumber(  	"LifterSpeed",      axisValue);
	SmartDashboard.putBoolean(  "limitBottomR",     hitBR);
	SmartDashboard.putBoolean(  "limitTopR",        hitTR);
	SmartDashboard.putBoolean(  "limitBottomL",     hitBL);
	SmartDashboard.putBoolean(  "limitTopL",        hitTL);	
	
}



//SWITCH OFF BETWEEN SINGLE OR DUAL CONTROLLERS
 public void checkSingle(){
	SmartDashboard.putBoolean(  "Single Controller Mode",     SINGLE_CONTROLLER);
	if (driverStick.getRawButton(CHANGE_BUTTON)==true ){  //if holding the Y button
		SINGLE_CONTROLLER = false;
	}                    	
 }
 
 
//RUMBLE WHEN CAPTURING A TOTE
 public void checkRumble(){

	 double rumbleLength = 2.0;  //seconds for rumble
	 boolean gotTote = limitTote.get(); //check if we have a tote
	 SmartDashboard.putBoolean(  "Got Tote?",        gotTote);
	 
	 //do this first time
	 if (gotTote == true && rumbleToggle == false){
		 rumbleToggle = true;
		 rumbleTimer.reset();
	 }
	 
	 //do this a while longer
	 else if(gotTote == true && rumbleToggle==true && rumbleTimer.get()<rumbleLength){
		 //manipStick.setRumble(Joystick.RumbleType.kRightRumble, 1);
	 }	 
	 
	 else if(rumbleTimer.get()>rumbleLength*2){
		 rumbleToggle = false;
	 }

 }



//SENDS GYRO VALUES TO SMARTDASHBOARD
public void processGyro() {	  
	  
	boolean is_calibrating = imu.isCalibrating();
    if ( first_iteration && !is_calibrating ) {
        Timer.delay( 0.3 );
        imu.zeroYaw();
        first_iteration = false;
    }
    
    // Update the dashboard with status and orientation data from the nav6 IMU
    SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
    SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
    SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
    SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

    // If you are using the IMUAdvanced class, you can also access the following additional functions, at the expense of some extra processing that occurs on the CRio processor    
    SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
    SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
    SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
    SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());   
}


//AUTO EAT METHOD -Adonis & Jatara
public void autoEat() {
eaterRight.set(eatSpeed);
eaterLeft.set(-eatSpeed);
}

//AUTO LIFT METHOD -Adonis & Jatara
public void autoLift() {
lifterLeft.set(-liftSpeed);
}

//AUTO DRIVE TO NEXT TOTE METHOD
public void autoDrive(double driveSpeed) {
frontleft.set(1);
backleft.set(1);
backright.set(1);
frontright.set(1);
}

//AUTO DROP OFF A STACK METHOD
public void autoDrop(double liftSpeed) {
 liftingSettings(liftSpeed);
}





}




