package org.usfirst.frc.team4141.robot.autocommands;
//import edu.wpi.first.wpilibj.Encoder; Nope

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.MDSubsystem;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class ClosedLoopDriveDistanceCommand extends MDCommand {
	
	//@SuppressWarnings("unlikely-arg-type")
	//MDDriveSubsystem driveSubsystems = (MDDriveSubsystem)robot.getSubsystems().get("driveSystem");
	//WPI_TalonSRX talon = (WPI_TalonSRX)driveSubsystems.getMotors().get(MDDriveSubsystem.MotorPosition.rearLeft.toString());
	
	private MDDriveSubsystem driveSubsystem;
	//Raw variables are in encoder units (ticks)
	private double m_targetDistanceFT; 
	private double m_targetDistanceRaw;
	private int m_speedFTPS; 
	private int m_speedRaw;
	private double m_wheelCircumferenceInches = (Math.PI)*(6); //wheel diameter is 6in
	private double m_nativeUnitsPerRotation = 4096;
	//1549;
	//1290.88841
	private boolean m_reverse;
	
	// ------------------------------------------------ //
	
	/**
	 * Constructor for the DriveDistanceCommand
	 * Within the constructor is a fail-safe to check that the Drive Subsystem
	 * is connected and ready to be used. If the Drive Subsystem is not connected 
	 * the Robot will not enable.
	 *  
	 * @param robot - the robot object
	 * @param name - name of this DriveDistanceCommand
	 * @param targetDistanceInFeet - Desired distance to travel (in feet)
	 * @param speed - The speed of the robot in feet per second (only go to the tenths place!)
	 * @param reverse - Set to false if going forward, but set to true of going backwards
	 */
	public ClosedLoopDriveDistanceCommand(MDRobotBase robot, String name, double targetDistanceInFeet, double speedInFTPS, boolean reverse) {
		super(robot, name);
		
		m_targetDistanceFT = targetDistanceInFeet;
	    m_targetDistanceRaw = (m_targetDistanceFT*12)/m_wheelCircumferenceInches*m_nativeUnitsPerRotation;
//	    m_targetDistanceRaw = m_targetDistanceFT*2607.59458762;
//		m_speedFTPS = (int)(speedInFTPS*10);
//	    m_speedRaw = m_speedFTPS*261/10;
	    m_speedRaw = (int) ((m_speedFTPS/10)*(12/m_wheelCircumferenceInches)*(m_nativeUnitsPerRotation));
		m_reverse = reverse;
		
		// Make sure that the Drive Subsystem is active
		if(!getRobot().getSubsystems().containsKey("driveSystem")){
			log(Level.ERROR, "initialize()", "Drive subsystem not found");
			throw new IllegalArgumentException("Drive Subsystem not found");
		}
		driveSubsystem = (MDDriveSubsystem)getRobot().getSubsystems().get("driveSystem"); 
		requires(driveSubsystem);
		
		System.out.println("Closed loop drive distance command has been constructed");
		
		
	}

	// Initialize is called when the command first starts
	 
	protected void initialize() {
		System.out.println("Closed loop initialize");
		MDRobotBase robot = this.getRobot();
		MDSubsystem driveSubsystem = robot.getSubsystems().get("driveSystem");		
		String motorName = MDDriveSubsystem.MotorPosition.rearLeft.toString();
		WPI_TalonSRX talon = (WPI_TalonSRX)driveSubsystem.getMotors().get(motorName);
		
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talon.configClosedloopRamp(2, 0); //Set ramp seconds from stop to full throttle
		talon.setInverted(m_reverse); //Invert the signal if needed
		talon.configMotionCruiseVelocity(m_speedRaw, 10);
		talon.configPeakOutputForward(1, 0);
		talon.configPeakOutputReverse(-1, 0);
		talon.set(ControlMode.Position, m_targetDistanceRaw);
		
	}
}
	
	