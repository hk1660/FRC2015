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
	int STRAFE_AXIS = 0; //Left joystick side to side
	
	int LIFTDROP_AXIS = 1; //Left joystick up and down
	int EAT_BUTTON = 1; //A
	int SPIT_BUTTON = 3; //X
	int OPEN_BUTTON = 5; //LB
	int CLOSE_BUTTON = 6; //RB
	
	int COMPRESSOR_ON_BUTTON = 8; //Start
	int COMPRESSOR_OFF_BUTTON= 7; //Back
	int CHANGE_BUTTON = 4; //Y
	int DPAD = 0; //POV
	
	
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
  Relay armRelay;
  Relay airC;
  Relay relayFlame;
  
  //DECLARING SENSORS
  DigitalInput pressureSwitch;
  DigitalInput limitTote;
  DigitalInput limitContainer;
  DigitalInput limitTopR;
  DigitalInput limitTopL;
  DigitalInput limitBottomR;
  DigitalInput limitBottomL;
  
  //DECLARING TIMERS
  public Timer rumbleTimer = new Timer();
  
    
  //NAVX GYRO CODE
  IMUAdvanced imu;      //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
  SerialPort serial_port;
  boolean first_iteration;

  public static RobotDrive hkDrive;
  public static Joystick driverStick;
  public static Joystick manipStick;
  
  //EATING VARIABLES
  double eatSpeed=0.75;
  double spitSpeed=0.50;
  double liftSpeed=0.30;
  
  //SET PID VALUES
  double LP = 0.8;
  double LI = 0.0;
  double LD = 0.0;
  double FP = 0.8;
  double FI = 0.0;
  double FD = 0.0;
  
  //LIFTING VARIABLES
  double P = 0.8;
  double I = 0.0;
  double D = 0.0;
  
  int bottom = 0;
  int low = 1480/2;
  int middle = 14*1481;
  int high = 26*1481;
  int top = 55690;  //experimental value of top of the Lift
  
  double manualLiftRate = 1481/4;
  int manualLiftCount = 0;
  int currentL = 0;
  int currentR = 0;

  //BOOLEANS
  public boolean rumbleFlag = false;
  public boolean CONTROLLER_TOGGLE = false; //variable to toggle controller settings
  public boolean autoStackEatFlag= false;
  public boolean autoStackPickUpFlag= false;
  public boolean autoStackSpitFlag = false;
  
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
	  	  
	  
	  //INITIALIZE OUTPUTS
      frontleft  = new CANTalon(6);
      backleft   = new CANTalon(4);
      backright  = new CANTalon(3);
      frontright = new CANTalon(2);      
      lifterLeft= new CANTalon(1);
      lifterFollower= new CANTalon(5);
      eaterRight = new Talon(1);
      eaterLeft  = new Talon(2);
      airC  = new Relay(0);
      armRelay  = new Relay(1);
      relayFlame = new Relay(2);

      //INITIALIZE SENSORS    
      pressureSwitch = new DigitalInput(0);
      limitTote = new DigitalInput (1);
      //limitContainer = new DigitalInput (2);
      limitBottomR = new DigitalInput (3);
      limitTopR = new DigitalInput (4);
      limitBottomL = new DigitalInput (5);
      limitTopL = new DigitalInput (6);     
      
}

  
//////////////////////////////////////////
//OFFICIAL FRC METHODS CALLED EACH MATCH//
/////////////////////////////////////////
  
  public void robotInit() {
	    rumbleTimer.start();
	    imu.zeroYaw();
	  	hkDrive     = new RobotDrive(frontleft, backleft, frontright, backright);
	    driverStick = new Joystick(1);
	    manipStick  = new Joystick(2);
	    //HKdriveClassObject.zeroYaw();  //calibrate robot gyro to zero when facing away from driver (may need 20 seconds)
	    airC.set(Relay.Value.kForward);
	    
		//LIFT ENCODERS SETUP
	    lifterLeft.changeControlMode(CANTalon.ControlMode.Position);
		lifterFollower.changeControlMode(CANTalon.ControlMode.Position);
		lifterLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lifterFollower.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
	
		//CHOOSING AUTO MODE
	    startingPosition = new SendableChooser();
        startingPosition.addDefault("Left", new Integer(1));
        startingPosition.addObject("Middle", new Integer(2));
        startingPosition.addObject("Right", new Integer(3));
        SmartDashboard.putData("startingPosition", startingPosition);
        
        strategy = new SendableChooser();
        strategy.addDefault("Move forward only", new Integer(1));
        strategy.addObject("Carry tote", new Integer(2));
        strategy.addObject("Push Container", new Integer(3));
        strategy.addObject("Carry tote and container", new Integer(4));
        strategy.addObject("Three tote stack", new Integer(5));
        SmartDashboard.putData("strategy selector", strategy);
     
	}  
    
  
  public void autonomous(){
	  	  
	 Timer timerAuto = new Timer();
	 timerAuto.start(); 
	 int currentStrategy = (int) strategy.getSelected(); 
	  
	 while(isAutonomous() && isEnabled()){ 
	  
		 lifterLimitPosition();
		 processGyro();
		 
		 double timerA = timerAuto.get();
		 SmartDashboard.putNumber("match time",timerA);
		  
	     if(currentStrategy == 1) {
	    	runAutoStrategy_GoForwardOnly(timerAuto); 
	     }  
	     if(currentStrategy == 2) {
	    	 onlyToteStrategy(timerAuto); 
		     } 
	     if(currentStrategy == 3) {
	    	 onlyContainerStrategy(timerAuto);
	     }
	     if(currentStrategy == 4) {
	    	 oneToteStrategy(timerAuto);
	     }
	     if(currentStrategy == 5) {
	    	 threeStackStrategyNoContainersInTheWay(timerAuto);
	     }
	     if(Timer.getMatchTime() > 15 ) {
    		 stopDrive();
    		 stopEatTote();
    	 }
	     
	  }
  }
  
  
  public void operatorControl() {
	int counter1 = 0;
    while (isOperatorControl() && isEnabled()) {
    	
    	SmartDashboard.putNumber(  "Op Control Counter",     counter1++);
       	checkFlame();
    	processGyro();
     	checkJoystick();	
    	checkEatingButtons();
       	checkArms();
       	checkLiftingButtons();
       	checkCompPressureSwitch();
       	//autoStackButtons();
       	//checkLiftingAxis();
    	checkRumble(rumbleTimer);
    	
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
	
	
	//MOVE DRIVETRAIN WITH XBOX360 JOYSTICKS -Matthew
	public void checkJoystick()
	{
		
		 double threshold = 0.11;
		 
		 double strafe = squareInput(driverStick.getRawAxis(STRAFE_AXIS)) ; // right and left on the left thumb stick?
		 double moveValue = squareInput(driverStick.getRawAxis(FORWARDBACKWARD_AXIS));// up and down on left thumb stick?
		 double rotateValue = squareInput(driverStick.getRawAxis(TURNSIDEWAYS_AXIS));// right and left on right thumb stick
		
		 //KILL GHOST MOTORS -Matthew & Dianne
		if(moveValue > threshold*-1 && moveValue < threshold) {
			moveValue = 0;
		}
		if(rotateValue > threshold*-1 && rotateValue < threshold) {
			rotateValue = 0;
		}
		if(strafe > threshold*-1 && strafe < threshold) {
			strafe = 0;
		}
		
		//MECANUM -Matthew
		SmartDashboard.putNumber(  "move",        moveValue);
		SmartDashboard.putNumber(  "rotate",        rotateValue);
		SmartDashboard.putNumber(  "Strafe",        strafe);
	    //Below is right order, Internet(wpi) wrong
		hkDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, imu.getYaw()); //imu.getYaw()
		//HKdriveClassObject.doMecanum(x,moveValue,rotateValue); 
	}

	public double squareInput(double x) {
        if(x > 0 ) {
      	return Math.pow(x, 4);
        }
        else{
      	  return -1*Math.pow(x, 4); 
        }
      }

	
	//EAT and SPITING WITH XBOX360 -Adonis & Jatara
	public void checkEatingButtons(){
	
		boolean hitT = limitTote.get();
		//boolean hitC = limitContainer.get();
		SmartDashboard.putBoolean("hit tote?", hitT);
		//SmartDashboard.putBoolean("hit container?", hitC);
			
		if (manipStick.getRawButton(EAT_BUTTON)==true )
		{  //if holding the A button, then eater motor spin	
	        eaterRight.set(eatSpeed);
	        eaterLeft.set(-eatSpeed);
	        closeGrab();
			SmartDashboard.putString(  "Eater",        "Eating");
		}
		else if (manipStick.getRawButton(SPIT_BUTTON)==true ){  //if holding the X button, 
			//then eater motor, spin backwards	
	
			eaterRight.set(-spitSpeed);
			eaterLeft.set(spitSpeed);
			closeGrab();
			SmartDashboard.putString(  "Eater",        "Spitting");
		}
		else{
			eaterRight.set(0.0);
			eaterLeft.set(0.0);
			SmartDashboard.putString(  "Eater",        "Hungry1");
		}
	}
	    
	
	//INTAKE WITH XBOX360 jamesey 
	public void checkArms(){
		if (manipStick.getRawButton(OPEN_BUTTON)==true ){  //if holding the LB button, 	        
			openGrab();               
		}
		
		if (manipStick.getRawButton(CLOSE_BUTTON)==true ){  //if holding the RB button, 
		    closeGrab();
		}
	}
	
	
	//COMPRESSOR ON & OFF WITH PRESSURE SWITCH OR JOYSTICKS -jamesey
	public void checkCompPressureSwitch(){
		
	    SmartDashboard.putBoolean("Pressure Switch", pressureSwitch.get());
	    checkToggle();
		
	    //based on Pressure Switch		    
	    if(   CONTROLLER_TOGGLE == false   ){
	    	
					if (pressureSwitch.get()==true) {
				        airC.set(Relay.Value.kOff);
				        SmartDashboard.putString("Compressor", "PSwitched OFF");
					} 
				   else {
				        airC.set(Relay.Value.kForward);
				        SmartDashboard.putString("Compressor", "PSwitched ON");
				   } 		
	    }
		//Manual override of compressor only on DriverStick
		if(   CONTROLLER_TOGGLE == true  ){
			
			SmartDashboard.putBoolean("Comp Button", manipStick.getRawButton(COMPRESSOR_ON_BUTTON));
			
			if (driverStick.getRawButton(COMPRESSOR_ON_BUTTON)==true ){  //if holding the start button	
				airC.set(Relay.Value.kForward);
				SmartDashboard.putString(  "Compressor",        "ButtonFWD ON");
			}                     		
			 if (driverStick.getRawButton(COMPRESSOR_OFF_BUTTON)==true ){  //if holding the back button, 
				airC.set(Relay.Value.kOff);
				SmartDashboard.putString(  "Compressor",        "Button OFF");
			}
		} 
	}	

	
    //DRAGON FLAME IF TOTE IS UPRIGHT
    public void checkFlame() {

    	int FLAME_ONBUTTON = 5;
    	int FLAME_OFFBUTTON = 6;
    	if(driverStick.getRawButton(FLAME_OFFBUTTON) == true) {
    		relayFlame.set(Relay.Value.kReverse);
    	}
    	if(driverStick.getRawButton(FLAME_ONBUTTON) == true) {
    		relayFlame.set(Relay.Value.kForward);                 
    	}	
    }

	//SWITCH OFF BETWEEN CONTROLLER SETTINGS
	public void checkToggle(){
		if (driverStick.getRawButton(CHANGE_BUTTON)){  //if holding the Y button
			CONTROLLER_TOGGLE = false;
		}
		
		if(manipStick.getRawButton(CHANGE_BUTTON)){ //if holding the Y button
			CONTROLLER_TOGGLE = true;
		}	
		SmartDashboard.putBoolean(  "Controller Toggle?",     CONTROLLER_TOGGLE);
	 }

	
	//LIFT WITH XBOX360 -Adonis & Jatara\ 
	public void checkLiftingButtons(){
	    
		lifterLimitPosition();
		
		//DPAD POSITION CONTROL COMMANDS
		SmartDashboard.putNumber("DPAD Value", manipStick.getPOV(DPAD));
		
		if(manipStick.getPOV(DPAD) == 180) {  //go to 0.5" from bottom
			lifterLeft.set(low);
			lifterFollower.set(-low);
			openGrab();
			SmartDashboard.putString("Lifter Status", "Down");
			manualLiftCount = 0;
		}
		if(manipStick.getPOV(DPAD) == 270) { //go to 14.5" from bottom
			lifterLeft.set(middle);
			lifterFollower.set(-middle);
			openGrab();
			SmartDashboard.putString("Lifter Status", "Middle");
			manualLiftCount = 0;
		}		
		if(manipStick.getPOV(DPAD) == 0) { //go to 26.5" from bottom
		    lifterLeft.set(high);
			lifterFollower.set(-high);
			openGrab();
			SmartDashboard.putString("Lifter Status", "High");
			manualLiftCount = 0;
		}
		if(manipStick.getRawAxis(5) > 0.2){ //go down until you hit limit switch
			lifterLeft.set(-1481*40);
			lifterFollower.set(1481*40);
			SmartDashboard.putString("Lifter Status", "LIMIT");
		}
		
		//DISABLE & ENABLE COMMANDS
		if(manipStick.getRawAxis(3)>0.1) { //right trigger disables the motors
			lifterLeft.disableControl();
			lifterFollower.disableControl();
			SmartDashboard.putString("Lifter Status", "Disabled");
		}
		if(manipStick.getRawAxis(4)>0.2){ //left trigger re-enables lift motors
			lifterLeft.enableControl();
			lifterFollower.enableControl();	
			SmartDashboard.putString("Lifter Status", "Enabled");
		}

		SmartDashboard.putNumber(  	"manualLiftCount",      manualLiftCount);
	}
			
	public void lifterLimitPosition(){

		SmartDashboard.putNumber("encLspeed", lifterLeft.getEncVelocity());
		SmartDashboard.putNumber("encRspeed", lifterFollower.getEncVelocity());
		SmartDashboard.putNumber("currentCommandL", lifterLeft.getPosition());
		SmartDashboard.putNumber("currentCommandR", lifterFollower.getPosition());
	
		//LIMIT SWITCH SETUP
		boolean hitTL = limitTopL.get();
		boolean hitBL = limitBottomL.get();
		boolean hitTR = limitTopR.get();
		boolean hitBR = limitBottomR.get();
		
		SmartDashboard.putBoolean(  "limitBottomR",     hitBR);
		SmartDashboard.putBoolean(  "limitTopR",        hitTR);
		SmartDashboard.putBoolean(  "limitBottomL",     hitBL);
		SmartDashboard.putBoolean(  "limitTopL",        hitTL);	
		
		//PID SETUP
		lifterLeft.setPID(LP, LI, LD);
		lifterFollower.setPID(FP, FI, FD);
		
		//CHANGE PID VALUES IF OUT OF SYNC -Matthew
		double syncPfactor = 0.6;
		double syncIfactor = 1.0;
		double syncDfactor = 1.0;
		int errorThresh = 700; //about a half inch
		
		int leftEncPosition = Math.abs( lifterLeft.getEncPosition() );
		int followerEncPosition = Math.abs( lifterFollower.getEncPosition() );
		
		SmartDashboard.putNumber("encLposition", leftEncPosition);
		SmartDashboard.putNumber("encRposition", followerEncPosition);
		SmartDashboard.putNumber("leftLifterError", lifterLeft.getClosedLoopError());
		SmartDashboard.putNumber("followerLifterError", lifterFollower.getClosedLoopError());
		SmartDashboard.putNumber("current Output LEFT", lifterLeft.getOutputCurrent());
		SmartDashboard.putNumber("current Output RIGHT", lifterFollower.getOutputCurrent());
		 
		if(leftEncPosition - followerEncPosition   > errorThresh) {
			lifterFollower.setPID(FP*syncPfactor, FI*syncIfactor, FD*syncDfactor);
		}
		else if(followerEncPosition -leftEncPosition  > errorThresh) {
			lifterLeft.setPID(LP*syncPfactor, LI*syncIfactor, LD*syncDfactor);
		}
		else {
			lifterFollower.setPID(FP, FI, FD);
			lifterLeft.setPID(LP, LI, LD);
		}
		
		//LIMIT SWITCH-BASED COMMMANDS
		if(hitBR==false){		//RESET THE RIGHT ENCODER when hit bottom
			lifterFollower.setPosition(bottom);
			lifterFollower.set(-low);
			SmartDashboard.putString("Lifter Status", "ResetRight");
		}
		if(hitBL==false) {		//RESET THE LEFT ENCODER when hit bottom
			lifterLeft.setPosition(bottom);
			lifterLeft.set(low);
			SmartDashboard.putString("Lifter Status", "ResetLeft");
		}
		if(hitTR==false){  //move Right side to 26.5" if hit the top
			lifterFollower.set(top);
			lifterFollower.setPosition(high);
			SmartDashboard.putString("Lifter Status", "TopRightReset");			
		}
		if(hitTL==false){  //move Left side to 26.5" if hit the top
			lifterLeft.set(top);
			lifterLeft.setPosition(high);
			SmartDashboard.putString("Lifter Status", "TopLeftReset");
		}
  }
	

	
	//MANUAL CONTROL OF POSITION
	public void checkLiftingAxis(){
		
		lifterLimitPosition();
	
		SmartDashboard.putNumber("LifterAxis", manipStick.getRawAxis(1));
		
		if(manipStick.getRawAxis(1)==0.0){
			currentL = lifterLeft.getEncPosition();
			currentR = lifterFollower.getEncPosition();
		}
		if(manipStick.getRawAxis(1)>0.1){
			manualLiftCount = manualLiftCount+1;
			manualLiftRate = manipStick.getRawAxis(1)*1481/4;
			lifterLeft.set(manualLiftRate*manualLiftCount + currentL);			
			lifterFollower.set(-manualLiftRate*manualLiftCount -currentR);
		}
		if(manipStick.getRawAxis(1)<-0.1){
			manualLiftCount = manualLiftCount+1;
			manualLiftRate = manipStick.getRawAxis(1)*1481/4;
			lifterLeft.set(currentL - manualLiftRate*manualLiftCount);			
			lifterFollower.set(-currentR + manualLiftRate*manualLiftCount);
		}
		
		SmartDashboard.putNumber(  	"manualLiftCount",      manualLiftCount);
	}
	
	
	//RUMBLE WHEN CAPTURING A TOTE
	 public void checkRumble(Timer rumbleTimer) {

		 double timerR = rumbleTimer.get();
		 double rumbleLength = 2.0;  //seconds for rumble	
		 boolean noTote = limitTote.get(); //check if we have a tote
		 SmartDashboard.putBoolean(  "No Tote?",        noTote);
		 SmartDashboard.putBoolean("rumbleFlag", rumbleFlag);
		 SmartDashboard.putNumber("rumbleTimer", timerR);
		 
		 
		 
		 //do this first time
		 if (noTote == false && rumbleFlag == false && timerR < 4){
			 rumbleFlag = true;
			 rumbleTimer.reset();
		 }
		 
		 //do this a while longer
		 if(rumbleFlag==true && timerR < rumbleLength){
		 manipStick.setRumble(Joystick.RumbleType.kLeftRumble,1);	 
		 manipStick.setRumble(Joystick.RumbleType.kRightRumble,1);
		 }	 
		 
		 if(timerR > rumbleLength*2){
			 rumbleFlag = true;
		 }
		 		 
	 }
	 
	
	//AUTO DRIVE METHODS
	public void goForwardAtSpeed(double speed) {
		hkDrive.mecanumDrive_Cartesian(0, 0, speed, 0);
	}
	
	public void strafeLeftAtSpeed(double speed) {
		hkDrive.mecanumDrive_Cartesian(-speed, 0, 0, 0);
	}
	
	public void strafeRightAtSpeed(double speed) {
		hkDrive.mecanumDrive_Cartesian(speed, 0, 0, 0);
	}
	
	public void turnLeftAtSpeed(double speed) {
		hkDrive.mecanumDrive_Cartesian(0, speed, 0, 0);
	}
	
	public void turnRightAtSpeed(double speed) {
		hkDrive.mecanumDrive_Cartesian(0, -speed, 0, 0);
	}
	
	public void stopDrive() {
		hkDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	public void turnAndStrafe(double strafeSpeed, double turnSpeed) {
		hkDrive.mecanumDrive_Cartesian(-strafeSpeed, -turnSpeed, 0, 0);
	}
	
	
	//AUTO EAT METHOD -Adonis & Jatara
	public void eatTote() {
		eaterRight.set(eatSpeed);
		eaterLeft.set(-eatSpeed);	
	}
	
	public  void stopEatTote() {
		eaterRight.set(0);
		eaterLeft.set(0);
	}
	
	//AUTO GRAB METHODS
	public void closeGrab(){
		armRelay.set(Relay.Value.kForward);		
	}
	
	public void openGrab(){
		armRelay.set(Relay.Value.kReverse);
	}
	
	
	//AUTO LIFT METHODS -Adonis & Jatara
	public void autoLift(double position) {
		lifterLeft.set(position);
		lifterFollower.set(-position);  //accounts for lifters acting in opposite directions
	}
	
	public void autoDrop() {
		autoLift(low);
	}
	
	
	//AUTO STRATEGIES
	public void runAutoStrategy_GoForwardOnly(Timer timerAuto) {
		double timerA = timerAuto.get();
		if(timerA < 2) {
			goForwardAtSpeed(0.3);
		}
		else{
			stopDrive();
		}
	}
	
	public void toteStrategy(Timer timerAuto) {
		double timerA = timerAuto.get();
		if(timerA < 2) {
			eatTote();
			closeGrab();
	    }
		if(timerA > 2 && timerA < 4) {
			stopEatTote();
			openGrab();
			autoLift(middle);
		}
		if(timerA > 4 && timerA < 6) {
			turnRightAtSpeed(0.3);
		}
		if(timerA > 6 && timerA < 8) {
			goForwardAtSpeed(0.3);
		}
		if(timerA > 8 && timerA < 10) {
			autoLift(low);
		}
	}
	
	public void oneToteStrategy(Timer timerAuto) {
		double timerA = timerAuto.get();
		
		if(timerA > 0 && timerA < 2.0){
			relayFlame.set(Relay.Value.kReverse);         
			autoLift(middle);
		}
		if(timerA > 2.0 && timerA < 4){
		    turnAndStrafe(0.6,0.3);
		}
		if(timerA > 7 && timerA < 10){
			goForwardAtSpeed(0.6);
		}
		if(timerA > 10 && timerA < 12){
			relayFlame.set(Relay.Value.kForward);
			stopDrive();
			
		}
	
	}
	
	public void testStrategy(Timer timerAuto) {
		double timerA = timerAuto.get();
	
		if(timerA < 2) {
			turnRightAtSpeed(0.3);
		}
		if(timerA > 2 && timerA < 4) {
			turnLeftAtSpeed(0.3);
		}
		if(timerA > 4 && timerA < 6) {
			strafeRightAtSpeed(0.3);
		}
		if(timerA > 6 && timerA < 8) {
			strafeLeftAtSpeed(0.3);
		}
		else{
			stopDrive();
		}
	}
	
    public void runOneToteStrategy(Timer timerAuto){
    	double timerA = timerAuto.get();
		 SmartDashboard.putNumber("match time",timerA);
	    	 
    	 if(timerA < 1.5) { //grab &eat a tote
    		 closeGrab();
    	 }
    	 if(timerA > 1.5 && timerA < 2.5) {  // move forward to next tote
    		 eatTote();
    	 }
         if(timerA > 2.5 && timerA < 3.0) { 
    		 stopEatTote();
         }
         if(timerA > 3.0 && timerA < 5.00) {
    		 lifterLeft.set(middle);
    		 lifterFollower.set(middle);
         }	
         if(timerA > 5.00 && timerA < 6.00) {
    		 hkDrive.mecanumDrive_Cartesian(0, 1, 0, 0);
    	 }	    	
    }
    
    //AUTO COMBO METHODS
    public void autoStackPrep() {
    	lifterLeft.set(middle);
    	lifterFollower.set(middle);
    	//openGrab();
    }
    
    public void autoStackEat() {  //A BUTT
    
    	Timer timerB = new Timer();
    	timerB.start();
    	
    	if(timerB.get() < 0.5 && autoStackEatFlag== true) {
    	eaterRight.set(eatSpeed);
    	eaterLeft.set(-eatSpeed);
    	}
    	if(timerB.get() > 0.5 && timerB.get() < 1 && autoStackEatFlag== true) {
    		closeGrab();
        }
    	if(limitTote.get()==true && autoStackEatFlag== true ){
    		stopEatTote();
    	}
    	if(timerB.get()>3 && autoStackEatFlag == true){
    		stopEatTote();
    		timerB.reset();
    		autoStackEatFlag = false;
    	}        	
    }
  
    public void autoStackPickUp(){
    	Timer timerC = new Timer();
    	timerC.start();

    	if(timerC.get() > 0 && timerC.get() < 1 && autoStackPickUpFlag== true) {
    		lifterLeft.set(low);
    		lifterFollower.set(low);
    		//openGrab();
    	}   	
    	if(timerC.get() > 2 && timerC.get() < 2.5 && autoStackPickUpFlag== true) {
    		lifterLeft.set(middle);
    		lifterFollower.set(middle);
    	}
    	if(timerC.get()>2.5 && autoStackPickUpFlag == true){
    		timerC.reset();
    		autoStackPickUpFlag = false;
    	}    	
    }
    
    public void autoStackSpit() {  //X
    	Timer timerD = new Timer();
    	timerD.start();
 /*
    	if(timerD.get() > 0 && timerD.get() < 1 && autoStackSpitFlag == true) {	    	
	    	
	    	lifterLeft.setPosition(low);
			lifterFollower.setPosition(low);
    	}*/
    	if(timerD.get()>0 && timerD.get()<2 && autoStackSpitFlag == true){
    		eaterRight.set(-spitSpeed);
        	eaterLeft.set(spitSpeed);	
        	closeGrab();
    	}
    	if(timerD.get()>2 && autoStackSpitFlag == true){
    		
    		//openGrab();
    		stopEatTote();
    		timerD.reset();
    		autoStackSpitFlag = false;
    	}    	
    }


  //AUTO STACK BUTTONS
  	public void autoStackButtons(){
  	
  		//prep on 
  		if(manipStick.getRawButton(1)==true){
  			autoStackPrep();
  			SmartDashboard.putBoolean("A button", manipStick.getRawButton(1));
  		}

  		//eat on A
  		if (manipStick.getRawButton(2)==true){
  			autoStackEatFlag=true;
  			SmartDashboard.putBoolean("B Button", manipStick.getRawButton(2));
  		}
  		autoStackEat();
  		
  		//lift on 
  		if (manipStick.getRawButton(3)==true){
  			autoStackPickUpFlag= true;
  			SmartDashboard.putBoolean("X Button", manipStick.getRawButton(3));
  		}
  		autoStackPickUp();
  		
  		//spit on X
  		if(manipStick.getRawButton(4)==true){
  			autoStackSpitFlag=true;
  			SmartDashboard.putBoolean("Y Button", manipStick.getRawButton(4));
  		}
  		autoStackSpit();



  	} 
   public void threeStackStrategyNoContainersInTheWay(Timer timerAuto){
	   double timerA = timerAuto.get();
	   timerAuto.start();
	   if(timerA < 0.5){
		   lifterLeft.set(low);
		   lifterFollower.set(low);
	   }
	   if(timerA > 0.5 && timerA < 2.5){
		  lifterLeft.set(high);
		  lifterFollower.set(high);
		  goForwardAtSpeed(0.3);
	   }
	   if(timerA > 2.5 && timerA < 3.5){
		   lifterLeft.set(low);
		   lifterFollower.set(low);
	   }
	   if(timerA > 3.5 && timerA < 4.5){
		   lifterLeft.set(high);
		   lifterFollower.set(high);
	   }
	   if(timerA > 4.5 && timerA < 6.5){
		   goForwardAtSpeed(03);	
	   }
	   if(timerA > 6.5 && timerA < 7.5){
		   lifterLeft.set(low);
		   lifterFollower.set(low);
	   }
	   if(timerA > 7.5 && timerA < 8.5){
		   lifterLeft.set(high);
		   lifterFollower.set(high);
	   }
	   if( timerA > 8.5 && timerA < 9){
		   turnLeftAtSpeed(0.4);
	   }
	   if(timerA > 9 && timerA < 11){
		   goForwardAtSpeed(0.5);
	   }
	   if(timerA > 11 && timerA < 12){
		   lifterLeft.set(low);
		   lifterFollower.set(low);
	   }
   }
  	
   public void threeStackStrategyTwo(Timer timerAuto) {
	   double timerA = timerAuto.get();
	   timerAuto.start();
	   if(timerA < 0.5 && timerA < 2 ) {
		   lifterLeft.setPosition(middle);
		   lifterFollower.setPosition(middle);
		   strafeLeftAtSpeed(0.4);
	   }
	   if(timerA > 2 && timerA < 3) {
		   goForwardAtSpeed(0.4);
	   }
	   
	   if(timerA > 3 && timerA < 4.5) {
		   strafeRightAtSpeed(0.4);
	   }
	   if(timerA > 4.5 && timerA < 5.5) {
		   goForwardAtSpeed(0.4);
	   }
	   if(timerA > 5.5 && timerA < 6.5) {
		  lifterLeft.setPosition(low);
		  lifterFollower.setPosition(low);
	   }
	   if(timerA > 6.5 && timerA < 8) {
		  lifterLeft.setPosition(high);
		  lifterFollower.setPosition(high);
		  strafeLeftAtSpeed(0.4);
	   }
	   if(timerA > 8 && timerA < 9) {
		   goForwardAtSpeed(0.4);
	   }
	   if(timerA > 9 && timerA < 10) {
		   strafeRightAtSpeed(0.4);   
	   }
	   if(timerA > 10 && timerA < 11) {
		   goForwardAtSpeed(0.4);
	   }
	   if(timerA > 11 && timerA < 12){
		   lifterLeft.setPosition(low);
		   lifterFollower.setPosition(low);
	   }
	   if(timerA > 12 && timerA < 13){
		   lifterLeft.setPosition(middle);
		   lifterFollower.setPosition(middle);
	   }
	   if(timerA > 13 && timerA < 15) {
		   strafeLeftAtSpeed(0.4);
	   }
	   if(timerA > 15 && timerA < 16) {
		  lifterLeft.setPosition(low);
		  lifterFollower.setPosition(low);
		  goForwardAtSpeed(-0.4);
	   }
	   
   }
   
   public void onlyToteStrategy(Timer timerAuto) {
	  double  timerA =  timerAuto.get();
	  if(timerA < 0.5  ) {
		  lifterLeft.setPosition(middle);
		  lifterFollower.setPosition(middle);
       }
	  if(timerA > 0.5 && timerA < 1.5) {
		  turnRightAtSpeed(0.5);
	  }
	  if(timerA > 1.5 && timerA < 5.5) {
		  goForwardAtSpeed(0.4);
	  }
	  
}
   
   public void onlyContainerStrategy(Timer timerAuto) {
	   double timerA = timerAuto.get();
	   if(timerA > 0 && timerA < 5) {
		   goForwardAtSpeed(0.4);
	   }
	   
   }
    		
    
}