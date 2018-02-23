package org.usfirst.frc.team4141.robot.autocommands;
//import edu.wpi.first.wpilibj.Encoder; Nope

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class DriveDistanceCommand extends MDCommand {
	
//Raw variables are in encoder units (ticks)
	private double m_desiredDistanceFT; 
	private double m_desiredDistanceRaw = m_desiredDistanceFT*15490.66092;
	private int m_speedFTPS; 
	private int m_speedRaw = m_speedFTPS*1549/10;
	//1290.88841
	private MDDriveSubsystem driveSubsystem;
	
	// ------------------------------------------------ //
	
	/**
	 * Constructor for the DriveDistanceCommand
	 * Within the constructor is a fail-safe to check that the Drive Subsystem
	 * is connected and ready to be used. If the Drive Subsystem is not connected 
	 * the Robot will not enable.
	 *  
	 * @param robot - the robot object
	 * @param name - name of this DriveDistanceCommand
	 * @param targetDistanceInFeet - Desired distance to travel (in feet) - NOTE: Negative means move backwards (Maybe...)
	 * @param speed - The speed of the robot in feet per second (Can be a double, but only go to the tenths place!)
	 */
	public DriveDistanceCommand(MDRobotBase robot, String name, double targetDistanceInFeet, double speed) {
		super(robot, name);
		m_desiredDistanceFT = targetDistanceInFeet;
		m_speedFTPS = (int)(speed*10);
		
		// Make sure that the Drive Subsystem is active
		if(!getRobot().getSubsystems().containsKey("driveSystem")){
			log(Level.ERROR, "initialize()", "Drive subsystem not found");
			throw new IllegalArgumentException("Drive Subsystem not found");
		}
		driveSubsystem = (MDDriveSubsystem)getRobot().getSubsystems().get("driveSystem"); 
		requires(driveSubsystem);
		
		
	}

	// Initialize is called when the command first starts
	 
	protected void initialize() {
		
		TalonSRX encoderMotor = new TalonSRX(2); //Set the TalonSRX ID that the encoder is attached to
		encoderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		encoderMotor.configClosedloopRamp(2, 0); //Set ramp seconds from stop to full throttle
		encoderMotor.configMotionCruiseVelocity(m_speedRaw, 0);
		encoderMotor.configPeakOutputForward(1, 0);
		encoderMotor.configPeakOutputReverse(-1, 0);
		encoderMotor.set(ControlMode.MotionMagic, m_desiredDistanceRaw);
		
	}
}
	
	