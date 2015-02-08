package org.usfirst.frc.team1660.robot;

//Code based on .drive class from FRC2465's Drive code:
//https://code.google.com/p/kauaibotsfirst2010/source/browse/trunk/Source/2015/DriveMule2015/src/org/usfirst/frc2465/Robot/subsystems/Drive.java
//+Uses Mecanum wheels
//+Takes into account the difference between the width & length of our drivebase
//+Takes into account the wheel size
//+Uses the NavX's gyro
//-Uses CANJaguars (see CANJaguar.ControlMode instantiation)
//+Integrates all of this with PID control


import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.kauailabs.nav6.frc.IMUAdvanced;



public class HKdriveClass extends PIDSubsystem {

	//MOTOR SETUP (from Robot Class)    
    RobotDrive hkDrive = Robot.hkDrive;
    CANTalon FL = Robot.frontleft;
  	CANTalon FR = Robot.frontright;
	CANTalon BL = Robot.backleft;
	CANTalon BR = Robot.backright;
	
	//NAVX GYRO SETUP (in HKdriveClass)
    SerialPort serial_port;
	public static IMUAdvanced imu;  //IMU imu;  
	public boolean first_iteration;
	
	//Mecanum Constants
    static final double cWidth          = 30.0;                 // Distance between left/right wheels
    static final double cLength         = 20.0;                // Distance btwn front/back wheels
    static final double wheelDiameter   = 6.0;                  // Per AndyMark Specs
    static final double wheelRadius     = wheelDiameter / 2;
    public static final int ROTATE_DIRECTION  = -1;

    // Proportional translation vs. Rotation
    //
    // For the same motor speed, the distance of translation and distance of rotation
    // are not the same, due to the proportions of the wheel radius, and the 
    // distance between front/back and left/right wheels.

    static final double cRotK = ((cWidth + cLength)/2) / wheelRadius;               // Rotational Coefficient
    
    static double invMatrix[][] = new double[][] {
        {  -1, 1,  cRotK },
        {   1, 1, -cRotK },
        {   1, 1,  cRotK },
        {  -1, 1, -cRotK },        
    };
    
    //AutoRotate Preferences
    CANTalon.ControlMode currControlMode;
    int maxOutputSpeed;
    int maxSpeedModeRPMs;    
    double tolerance_degrees = 2.0;
    double DefaultTargetDegrees = 0.0;
    
    //PID Preferences
    public static double p = 0.0070;   //proportional
    public static double i = 0.00001;	//integral
    public static double d = 0.0;		//derivative
    
    
    
    public HKdriveClass() {
    	super("HKdriveClass", p, i, d);
    	
    	try{
    		getPIDController().setContinuous(false);
            getPIDController().setInputRange(-180,180);
            getPIDController().setOutputRange(-1, 1);
            getPIDController().setAbsoluteTolerance(tolerance_degrees);
            setSetpoint(DefaultTargetDegrees);
            disable();
            
            hkDrive.setSafetyEnabled(false);
            
            /* FL.getPowerCycled();
            FR.getPowerCycled();
            BL.getPowerCycled();
            BR.getPowerCycled();*/
            
            maxSpeedModeRPMs = (int)(2650.0/12.75);
            setMode(CANTalon.ControlMode.Speed); //from this class

            
            
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    
    	//FIRST ITERATION OF GYRO CALIBRATION
    	try {
            serial_port = new SerialPort(57600,SerialPort.Port.kUSB); //or use .kOnBoard
            byte update_rate_hz = 20;  // set from 4 to 100, default is 100
            imu = new IMUAdvanced(serial_port,update_rate_hz);  // or use imu = new IMU(serial_port,update_rate_hz);
        
        } catch (Exception ex) {
            //ex.printStackTrace();
        }
        if ( imu != null ) {
            LiveWindow.addSensor("IMU", "Gyro", imu);
        }
        
        	first_iteration = true;
 
    		
   	}
    
        

    	
// METHODS TO CONTROL MECANUM DRIVETRAIN WITH GYRO & PID.
// Call these methods.	
    	
    
    // Set the default command for a subsystem here.
    public void initDefaultCommand() {   
        //setDefaultCommand(new StickDrive());
    }	
    
    
    //Initializes motor to boundary values from encoder and battery (based on CANJaguar class)
    void initMotor( CANTalon motor ) {
        try {
            if ( currControlMode == CANTalon.ControlMode.Speed )
            {
                //motor.configMaxOutputVoltage(12.0);
                //motor.configNeutralMode(CANTalon.NeutralMode.Brake);
                //motor.setSpeedMode(CANTalon.kQuadEncoder, 250, .2, .003, 0);
            }
            else
            {
               // motor.setPercentMode();
            }
            motor.enableControl();
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }    
    
    void setMode( CANTalon.ControlMode controlMode ) {
        
        currControlMode = controlMode;

        if ( currControlMode == CANTalon.ControlMode.Speed )
        {
                maxOutputSpeed = maxSpeedModeRPMs;
        }
        else // kPercentVbus
        {
                maxOutputSpeed = 1;
        }
        
        initMotor(FL);
        initMotor(FR);
        initMotor(BR);
        initMotor(BL);    
    } 
    
    
    void checkForRestartedMotor( CANTalon motor, String strDescription )
    {
        if ( currControlMode != CANTalon.ControlMode.Speed )   // kSpeed is the default
        {
            try {
                if ( !motor.isAlive() )
                {
                    Timer.delay(0.10); // Wait 100 ms
                    initMotor( motor );
                    String error = "\n\n>>>>" + strDescription + "CANTalon Power Cycled - re-initializing";
                    System.out.println(error);
                }
            } catch (Exception ex) {
                ex.printStackTrace();
            }
        }
    }    

    void mecanumDriveFwdKinematics( double wheelSpeeds[], double velocities[] )
    {
        for ( int i = 0; i < 3; i++ )
        {
            velocities[i] = 0;
            for ( int wheel = 0; wheel < 4; wheel ++ )
            {
                    velocities[i] += wheelSpeeds[wheel] * (1 / invMatrix[wheel][i]);
            }
            velocities[i] *= ((double)1.0/4);
        }
    }

    void mecanumDriveInvKinematics( double velocities[], double[] wheelSpeeds)
    {
        for ( int wheel = 0; wheel < 4; wheel ++ )
        {
            wheelSpeeds[wheel] = 0;
            for ( int i = 0; i < 3; i++ )
            {
                    wheelSpeeds[wheel] += velocities[i] * invMatrix[wheel][i];
            }
        }
    }    

    boolean fod_enable = true;
    double next_autorotate_value = 0.0;
    
    public void doMecanum( double vX, double vY, double vRot) {
        
        // If auto-rotating, replace vRot with the next
        // calculated value
        
        if ( getAutoRotation() ) {
            vRot = next_autorotate_value;
        }
        
        boolean imu_connected = false;
        if ( imu != null ) { 
                imu_connected = imu.isConnected();
        }
                
        // Field-oriented drive - Adjust input angle for gyro offset angle
        
        double curr_gyro_angle_degrees = 0;
        if ( fod_enable && imu_connected ) 
        {
                curr_gyro_angle_degrees = imu.getYaw();
        }
        double curr_gyro_angle_radians = curr_gyro_angle_degrees * Math.PI/180;       
          
        double temp = vX * Math.cos( curr_gyro_angle_radians ) + vY * Math.sin( curr_gyro_angle_radians );
        vY = -vX * Math.sin( curr_gyro_angle_radians ) + vY * Math.cos( curr_gyro_angle_radians );
        vX = temp;
        
        try {
            double excessRatio = (double)1.0 / ( Math.abs(vX) + Math.abs(vY) + Math.abs(vRot) );
            if ( excessRatio < 1.0 )
            {
                vX      *= excessRatio;
                vY      *= excessRatio;
                vRot    *= excessRatio;
            }
            
            vRot *= (1/cRotK);
            vRot *= ROTATE_DIRECTION;
            
            SmartDashboard.putNumber( "vRot", vRot);
            double wheelSpeeds[] = new double[4];
            double velocities[] = new double[3];
            velocities[0] = vX;
            velocities[1] = vY;
            velocities[2] = vRot;
            
            mecanumDriveInvKinematics( velocities, wheelSpeeds );
            
            /*
            checkForRestartedMotor( FL, "Front Left" );
            checkForRestartedMotor( FR, "Front Right" );
            checkForRestartedMotor( BL, "Back Left" );
            checkForRestartedMotor( BR, "Back Right" );
            */
            FL.set(maxOutputSpeed * wheelSpeeds[0] * -1);
            FR.set(maxOutputSpeed * wheelSpeeds[1]     );
            BL.set(maxOutputSpeed * wheelSpeeds[2] * -1);
            BR.set(maxOutputSpeed * wheelSpeeds[3]     );
            
            //CANTalon.updateSyncGroup(syncGroup);
            
            SmartDashboard.putNumber( "SpeedOut_FrontLeft", FL.get());
            SmartDashboard.putNumber( "SpeedOut_BackLeft", BL.get());
            SmartDashboard.putNumber( "SpeedOut_FrontRight", FR.get());
            SmartDashboard.putNumber( "SpeedOut_BackRight", BR.get());
            
            SmartDashboard.putNumber( "Speed_FrontLeft", FL.getSpeed());
            SmartDashboard.putNumber( "Speed_BackLeft", BL.getSpeed());
            SmartDashboard.putNumber( "Speed_FrontRight", FR.getSpeed());
            SmartDashboard.putNumber( "Speed_BackRight", BR.getSpeed());
            
        } catch (Exception ex) {
            ex.printStackTrace();
        }        
    }

    protected double returnPIDInput() {
        double current_yaw = 0.0;
        if ( imu.isConnected() ) {
            current_yaw = imu.getYaw();
        }
        SmartDashboard.putNumber( "AutoRotatePIDInput", current_yaw);
        return current_yaw;
    }

    protected void usePIDOutput(double d) {
        next_autorotate_value = d;
        SmartDashboard.putNumber( "AutoRotatePIDOutput", next_autorotate_value);
    }
    
    public void setAutoRotation(boolean enable) {
        if ( enable ) {
            getPIDController().enable();
        }
        else {
            getPIDController().disable();
        }
    }
    
    public boolean getAutoRotation() {
        SmartDashboard.putBoolean( "AutoRotateEnabled", getPIDController().isEnable());
        return getPIDController().isEnable();
    }
    
    public void setFODEnabled(boolean enabled) {
        fod_enable = enabled;
    }
    
    public boolean getFODEnabled() {
        return fod_enable;
    }

    
	//ZERO THE YAW ON GYRO AFTER CALIBRATION   
	public void zeroYaw(){
		//Calibration is complete after ~20 seconds after robot is powered on.
		//During Cal, robot should be still.
		
		boolean is_calibrating = imu.isCalibrating();
		if ( first_iteration && !is_calibrating ) {
		    Timer.delay( 0.3 );
		    imu.zeroYaw();
		    first_iteration = false;
		}
	
	}   


}
