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

  
  //EATING VARIABLES
  double eatSpeed=0.75;
  double spitSpeed=0.25;
  double liftSpeed=0.40;
  
  //SET PID VALUES
  double LP = 0.400;
  double LI = 0.000;
  double LD = 0.0;
  double FP = 0.400;
  double FI = 0.000;
  double FD = 0.0;
  
  //LIFTING VARIABLES
  double P = 0.60;
  double I = 0.00;
  double D = 0.00;
  
  int bottom = 0;
  int low = 1480/2;
  int middle = 14*1481;
  int high = 26*1481;
  int top = 55690;  //experimental value of top of the Lift
  
  double manualLiftRate = 1481/4;
  int manualLiftCount = 0;
  int currentL = 0;
  int currentR = 0;

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
      
      
      
      airC  = new Relay(0);
      armRelay  = new Relay(1);


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
	    airC.set(Relay.Value.kForward);
	   // SINGLE_CONTROLLER = true; //start by using only 1 xbox controller, touch button to add manipStick
	    
	  	hkDrive     = new RobotDrive(frontleft, backleft, frontright, backright);
	    driverStick = new Joystick(1);
	    manipStick  = new Joystick(2);
	    //HKdriveClassObject.zeroYaw();  //calibrate robot gyro to zero when facing away from driver (may need 20 seconds)
	    airC.set(Relay.Value.kForward);
	    
	    startingPosition = new SendableChooser();
        startingPosition.addDefault("Left", new Integer(1));
        startingPosition.addObject("Middle", new Integer(2));
        startingPosition.addObject("Right", new Integer(3));
        SmartDashboard.putData("startingPosition", startingPosition);
        
        strategy = new SendableChooser();
        strategy.addDefault("Move forward only", new Integer(1));
        strategy.addObject("Strategy 2", new Integer(2));
        strategy.addObject("Strategy 3", new Integer(3));
        SmartDashboard.putData("strategy selector", strategy);

        
	}  
  
  
  
  public void autonomous(){
	  	  
	 double autoLength = 15.0;
	 Timer timerAuto = new Timer();
	 timerAuto.start(); 
	 int currentStrategy = (int) strategy.getSelected(); 
	  
	 while(isAutonomous() && isEnabled()){ 
	  
		 lifterLimitPosition();
		 double timerA = timerAuto.get();
		 SmartDashboard.putNumber("match time",timerA);
		  
	     if(currentStrategy == 1) {
	    	runAutoStrategy_GoForwardOnly(timerAuto); 
	     }  
	     if(currentStrategy == 2) {
		    	testStrategy(timerAuto); 
		     } 
	     if(currentStrategy == 3) {
	    	 oneToteStrategy(timerAuto);
	     }
	    	 //lifterLeft.setPosition(middle);
	    	 //lifterFollower.setPosition(middle);
	    	// hkDrive.mecanumDrive_Cartesian(0, 1, 0, 0);
	    	
	    	 
//	    	 if(Timer.getMatchTime() > 5 ) {
	//    		 stopDrive();
	  //  	 }
	    	 
	    	 //Timer.delay(5);
	    	 //hkDrive.mecanumDrive_Cartesian(0, 0, 0, imu.getYaw());	 
	     
	    /* else if(currentStrategy == 2){	
	    	 eatTote();
	    	 Timer.delay(5);
	    	 lifterLeft.setPosition(middle);
	    	 lifterFollower.setPosition(middle);
	    	 hkDrive.mecanumDrive_Cartesian(1, 0, 0, imu.getYaw());
	    	 Timer.delay(5);
	    	 hkDrive.mecanumDrive_Cartesian(0, 1, 0, imu.getYaw());
	    	 Timer.delay(2);
	    	 hkDrive.mecanumDrive_Cartesian(-1, 0, 0, imu.getYaw());
	    	 Timer.delay(2);
	    	 eatTote();
	    	 Timer.delay(5);
	    	 hkDrive.mecanumDrive_Cartesian(-1, 0, 0, imu.getYaw());
	    	 Timer.delay(5);
	     } */
	    	
	    /* else {
	    	 eatTote();
	    	 Timer.delay(5);
	    	 lifterLeft.setPosition(middle);
	    	 lifterFollower.setPosition(middle);
	    	 hkDrive.mecanumDrive_Cartesian(1, 0, 0, imu.getYaw());
	    	 Timer.delay(5);
	    	 hkDrive.mecanumDrive_Cartesian(0, 1, 0, imu.getYaw());
	    	 Timer.delay(2);
	    	 hkDrive.mecanumDrive_Cartesian(-1, 0, 0, imu.getYaw());
	    	 Timer.delay(2);
	    	 eatTote();
	    	 Timer.delay(5);
	    	 hkDrive.mecanumDrive_Cartesian(1, 0, 0, imu.getYaw());
	    	 Timer.delay(5);
	    	 hkDrive.mecanumDrive_Cartesian(0, 1, 0, imu.getYaw());
	    	 Timer.delay(2);
	    	 hkDrive.mecanumDrive_Cartesian(-1, 0, 0, imu.getYaw());
	    	 Timer.delay(5);
	    	 lifterLeft.setPosition(high);
	    	 lifterFollower.setPosition(high);
	    	 eatTote();
	    	 Timer.delay(5);
	    	 hkDrive.mecanumDrive_Cartesian(-1, 0, 0, imu.getYaw());
	    	 Timer.delay(5);
	    	 
	    	 
  
	     }  */
	  }
  }
  
  
  public void operatorControl() {
	  int counter1 = 0;
    while (isOperatorControl() && isEnabled()) {
    	SmartDashboard.putNumber(  "Single Controller Mode",     counter1++);
    	//checkSingle();
    	//checkComp();

    	checkJoystick();	
    	processGyro();
    	
    	checkCompPressureSwitch();
    	checkEatingButtons();
       	checkArms();
       	
		//lifterLimitPosition();
       	checkLiftingButtons();
    	//adjustLiftingPID();
    	
    	//checkRumble();
    	
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

  
	//SWITCH OFF BETWEEN SINGLE OR DUAL CONTROLLERS
  static int counter = 0;
	/* public void checkSingle(){
		if (driverStick.getRawButton(CHANGE_BUTTON)){  //if holding the Y button
			SINGLE_CONTROLLER = false;
		}
		
		if(manipStick.getRawButton(CHANGE_BUTTON)){ //if holding the Y button
			SINGLE_CONTROLLER = true;
		}	
		SmartDashboard.putNumber(  "Single Controller Mode",     counter++);
	 }
	*/
	  
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
		hkDrive.mecanumDrive_Cartesian( strafe, -rotateValue, -moveValue, 0); //imu.getYaw()
		//HKdriveClassObject.doMecanum(x,moveValue,rotateValue); 
	}
	
	//EAT and SPITING WITH XBOX360 -Adonis & Jatara
	public void checkEatingButtons(){
	
		boolean hitT = limitTote.get();
		//boolean hitC = limitContainer.get();
		SmartDashboard.putBoolean("hit tote?", hitT);
		//SmartDashboard.putBoolean("hit container?", hitC);
		
		
		if(   SINGLE_CONTROLLER == false      )
		{
				//manipStick Code
				if (manipStick.getRawButton(EAT_BUTTON)==true && hitT == true )
				{  //if holding the A button, 
					//then eater motor spin	
					eatTote();
					SmartDashboard.putString(  "Eater",        "Eating");
				}
				else if (manipStick.getRawButton(SPIT_BUTTON)==true ){  //if holding the X button, 
					//then eater motor spin backwards	
					eaterRight.set(-spitSpeed);
					eaterLeft.set(spitSpeed);
					SmartDashboard.putString(  "Eater",        "Spitting");
	
				}
				else{
					eaterRight.set(0.0);
					eaterLeft.set(0.0);
					SmartDashboard.putString(  "Eater",        "Hungry1");
				}
		}
		
		else{
			SmartDashboard.putString(  "Eater",        "Disabled");
			/**
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
		
		
		**/
		}
	}
	    
	
	//INTAKE WITH XBOX360 jamesey 
	public void checkArms(){
		//manipStick
		if(   SINGLE_CONTROLLER == false   ){
			if (manipStick.getRawButton(OPEN_BUTTON)==true ){  //if holding the LB button, 	        
				armRelay.set(Relay.Value.kForward);                 
			}
			
			if (manipStick.getRawButton(CLOSE_BUTTON)==true ){  //if holding the RB button, 
			armRelay.set(Relay.Value.kReverse);
			}
		}
			
		//driveStick jamesey
		else{
	 
			if (driverStick.getRawButton(OPEN_BUTTON)==true ){  //if holding the LB button, 	        
			 armRelay.set(Relay.Value.kForward);                 
			}
			
			if (driverStick.getRawButton(CLOSE_BUTTON)==true ){  //if holding the RB button, 	
			armRelay.set(Relay.Value.kReverse);
			}
		}
	}
	
	
	//COMPRESSOR ON & OFF WITH PRESSURE SWITCH
	public void checkCompPressureSwitch(){
		
	    SmartDashboard.putBoolean("Pressure Switch", pressureSwitch.get());
		
	    //based on Pressure Switch	
	    
	    if(   SINGLE_CONTROLLER == false   ){
	    	
					if (pressureSwitch.get()==true) {
				        airC.set(Relay.Value.kOff);
				        SmartDashboard.putString("Compressor", "Switched OFF");
					} 
				   else {
				        airC.set(Relay.Value.kForward);
				        SmartDashboard.putString("Compressor", "Switched ON");
				   } 
					
	    }
		//Manual override of compressor only on Single Controller Mode
		if(   SINGLE_CONTROLLER == true  ){
			
			SmartDashboard.putBoolean("checking the comp On button", manipStick.getRawButton(COMPRESSOR_ON_BUTTON));
			if (driverStick.getRawButton(COMPRESSOR_ON_BUTTON)==true ){  //if holding the start button	
				airC.set(Relay.Value.kForward);
				SmartDashboard.putString(  "Compressor",        "Button fwd");
	        
			}                     		
			 if (driverStick.getRawButton(COMPRESSOR_OFF_BUTTON)==true ){  //if holding the back button, 
				airC.set(Relay.Value.kOff);
				SmartDashboard.putString(  "Compressor",        "Button rev");
			 }
		} 
		
		
	}
	
	
	//COMPRESSOR ON & OFF WITH JOYSTICKS -jamesey
	public void checkComp(){
		
		//manipStick
		if(   SINGLE_CONTROLLER == false   ){
				
				SmartDashboard.putBoolean("checking the comp On button", manipStick.getRawButton(COMPRESSOR_ON_BUTTON));
				if (manipStick.getRawButton(COMPRESSOR_ON_BUTTON)==true ){  //if holding the start button	
					airC.set(Relay.Value.kForward);
					SmartDashboard.putString(  "Compressor",        "Button fwd");
		        
				}                     		
				 if (manipStick.getRawButton(COMPRESSOR_OFF_BUTTON)==true ){  //if holding the back button, 
					airC.set(Relay.Value.kOff);
					SmartDashboard.putString(  "Compressor",        "Button rev");
				 }
		} 
		// driverStick	 
		 else{  
			 	if (driverStick.getRawButton(COMPRESSOR_ON_BUTTON)==true ){  //if holding the start button	
					 airC.set(Relay.Value.kForward);
					// airC.start();
					SmartDashboard.putString(  "Compressor",        "ON");
		
				}                    
				
				if (driverStick.getRawButton(COMPRESSOR_OFF_BUTTON)==true ){  //if holding the back button, 
						airC.set(Relay.Value.kOff);
						SmartDashboard.putString(  "Compressor",        "OFF");
			 	}
		}
	}
	
	
	//LIFT WITH XBOX360 -Adonis & Jatara\ 
	public void checkLiftingButtons(){
	    adjustLiftingPID();
		lifterLimitPosition();
		
		/*	//MANUAL CONTROL OF POSITION
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
			}*/
	
			//DPAD POSITION CONTROL COMMANDS
			SmartDashboard.putNumber("DPAD Value", manipStick.getPOV(DPAD));
			
			if(manipStick.getPOV(DPAD) == 180) {  //go to 0.5" from bottom
				lifterLeft.set(low);
				lifterFollower.set(-low);
				SmartDashboard.putString("Lifter Status", "Down");
				manualLiftCount = 0;
			}
			if(manipStick.getPOV(DPAD) == 270) { //go to 14.5" from bottom
				lifterLeft.set(middle);
				lifterFollower.set(-middle);
				SmartDashboard.putString("Lifter Status", "Middle");
				manualLiftCount = 0;
			}		
			if(manipStick.getPOV(DPAD) == 0) { //go to 26.5" from bottom
			    lifterLeft.set(high);
				lifterFollower.set(-high);
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
				lifterLeft.disable();
				lifterFollower.disable();
				SmartDashboard.putString("Lifter Status", "Disabled");
			}
			if(manipStick.getRawAxis(4)>0.1){ //left trigger renables lift motors
				lifterLeft.enableControl();
				lifterFollower.enableControl();	
				SmartDashboard.putString("Lifter Status", "Enabled");
			}
	
			SmartDashboard.putNumber(  	"manualLiftCount",      manualLiftCount);
	}
	
	
	public void lifterValues(){
		SmartDashboard.putNumber(  	"manualLiftCount",      manualLiftCount);
	
	}	
	
	
	//MANUALLY ADJUST VALUES OF P, I, D
	public void adjustLiftingPID(){
	
		if(manipStick.getRawButton(2)==true){
			LP = LP + 0.01;
			FP = FP + 0.01;
			SmartDashboard.putNumber("PValue", LP);
		}
	/*
		if(manipStick.getRawButton(8)==true){
			LI = LI + 0.01;
			FI = FI + 0.01;
			SmartDashboard.putNumber("IValue", LI);	
		}
	
		if(manipStick.getRawButton(7)==true){
			LD = LD + 0.01;
			FD = FD + 0.01;
			SmartDashboard.putNumber("DValue", LD);
		}
		
	*/
	} 
	
	
	//LINK LIFT MOTORS
	public void liftingSettings(double liftingSpeed) {
		lifterLeft.set(liftingSpeed);
		lifterFollower.set(liftingSpeed);
	}
	
	
	//RUMBLE WHEN CAPTURING A TOTE
	 public void checkRumble(){
		 
		 double rumbleLength = 2.0;  //seconds for rumble	
		 rumbleTimer.start();
		 boolean gotTote = limitTote.get(); //check if we have a tote
		 SmartDashboard.putBoolean(  "Got Tote?",        gotTote);
		 SmartDashboard.putBoolean("rumbleToggle", rumbleToggle);
		 SmartDashboard.putNumber("rumbleTimer", rumbleTimer.get());
		 
		 //do this first time
		 if (gotTote == true && rumbleToggle == false){
			 rumbleToggle = true;
			 rumbleTimer.reset();
		 }
		 //do this a while longer
		 else if(gotTote == true && rumbleToggle==true && rumbleTimer.get()<rumbleLength){
		 manipStick.setRumble(Joystick.RumbleType.kLeftRumble,1);	 
		 }	 
		 else if(rumbleTimer.get()>rumbleLength*2){
			 rumbleToggle = false;
		 }
		 else{
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
	
	
	//AUTO DRIVE METHODS
	public void autoDrive(double strafe, double rot, double fwd) {
		//frontleft.set(1);
		//backleft.set(1);
		//backright.set(1);
		//frontright.set(1);
		
		hkDrive.mecanumDrive_Cartesian(strafe, rot, fwd, 0); //gyroangle = imu.getYaw()	
	}
	
	public void stopDrive() {
		hkDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
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
		hkDrive.mecanumDrive_Cartesian(0, -speed, 0, 0);
	}
	
	public void turnRightAtSpeed(double speed) {
		hkDrive.mecanumDrive_Cartesian(0, speed, 0, 0);
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
		armRelay.set(Relay.Value.kReverse);
		
	}
	
	public void openGrab(){
		armRelay.set(Relay.Value.kForward);
		
		
	}
	
	
	//AUTO LIFT METHODS -Adonis & Jatara
	public void autoLift(double position) {
		lifterLeft.set(position);
		lifterFollower.set(-position);  //accounts for lifters acting in opposite directions
	}
	
	public void autoDrop() {
	
		autoLift(bottom);
	}
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
		
		if(timerA > 0.5 && timerA < 2.0){
			autoLift(middle);
		}
		if(timerA > 2.0 && timerA < 4.0){
		    strafeLeftAtSpeed(0.3);
		}
		/*
		if(timerA > 4 && timerA < 6){
			goForwardAtSpeed(0.3);
		}
		if(timerA > 6 && timerA < 8){
			turnRightAtSpeed(0.3);
		}
		if(timerA > 8 && timerA < 10){
			goForwardAtSpeed(0.3);
		}
		if(timerA > 10 && timerA < 12){
			autoLift(low);
		}
		*/
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
	    		 lifterLeft.setPosition(middle);
	    		 lifterFollower.setPosition(middle);
	         }	
	         if(timerA > 5.00 && timerA < 6.00) {
	    		 hkDrive.mecanumDrive_Cartesian(0, 1, 0, 0);
	    	 }
	    	
    }
    public double squareInput(double x) {
      if(x > 0 ) {
    	return Math.pow(x, 2);
      }
      else{
    	  return -1*Math.pow(x, 2); 
      }
    }

	
    
    public void lifterLimitPosition(){

		//ENCODERS SETUP
	    lifterLeft.changeControlMode(CANTalon.ControlMode.Position);
		lifterFollower.changeControlMode(CANTalon.ControlMode.Position);
		lifterLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lifterFollower.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		//lifterFollower.reverseSensor(true);
		
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
			lifterFollower.setPosition(0);
			//lifterFollower.set(0);
			lifterFollower.set(-low);
			SmartDashboard.putString("Lifter Status", "ResetRight");
		}
		if(hitBL==false) {		//RESET THE LEFT ENCODER when hit bottom
			lifterLeft.setPosition(0);
			//lifterLeft.set(0);
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


}





