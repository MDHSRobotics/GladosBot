package org.usfirst.frc.team4141.robot.autocommands;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;


public class DriveDistanceCommand extends MDCommand {
	
	private double m_targetDistanceInFeet;      // Desired distance to travel (in feet) - NOTE: Negative means move backwards
	private double m_power;                     // Power setting for drive: 0.0 to +1.0
	private double m_distanceTraveled;			// Distanced traveled thus far (in feet)
	private double m_velocity;			        // Velocity (feet/second) at current power setting
	private double m_elapsedTime;				// Time (in seconds) that this command has executed
	private boolean m_movingForward;			// True if moving forward; False if moving backward
	private Timer m_timer; 						// Timer for this command
	
	private int counter;
	
	private double velocityAtFullPower = 2;     // Velocity (feet/second) at full power - THIS IS A GUESS - CHECK IT!!
	
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
	 * @param targetDistanceInFeet - Desired distance to travel (in feet) - NOTE: Negative means move backwards
	 * @param power - Power setting for drive: 0.0 to +1.0
	 */
	public DriveDistanceCommand(MDRobotBase robot, String name, double targetDistanceInFeet, double power) {
		super(robot, name);
		
		// Make sure that the Drive Subsystem is active
		if(!getRobot().getSubsystems().containsKey("driveSystem")){
			log(Level.ERROR, "initialize()", "Drive subsystem not found");
			throw new IllegalArgumentException("Drive Subsystem not found");
		}
		driveSubsystem = (MDDriveSubsystem)getRobot().getSubsystems().get("driveSystem"); 
		requires(driveSubsystem);
		
		m_targetDistanceInFeet = targetDistanceInFeet;
		m_movingForward = (targetDistanceInFeet > 0);       // Use sign of distance to determine whether moving forward or backward
		m_power = power;
		m_velocity = m_power * velocityAtFullPower;  		// Scale velocity at full power by the current power (which is between 0 and 1.0)
		m_distanceTraveled = 0.;
		m_elapsedTime = 0.;
		m_timer = new Timer();
	}

	// Initialize is called when the command first starts
	 
	protected void initialize() {
		counter = 0;
		m_distanceTraveled = 0.;
		m_elapsedTime = 0.;
		m_timer.reset();
		m_timer.start();
		System.out.println("Starting DriveDistance Command: Target = " + m_targetDistanceInFeet + " feet");
	}
		
	// isFinished is called every 20ms to determine whether the robot has traveled the requested distance
	
	protected boolean isFinished() {
		if(m_distanceTraveled >= m_targetDistanceInFeet){
			m_timer.stop();
			System.out.println("Finished DriveDistance Command: Target = " + m_targetDistanceInFeet + "; Actual = " + m_distanceTraveled + "; Elapsed time = " + m_elapsedTime);
			return true;
		}
		else {
			return false;
		}
	}
	
	// Execute is called every 20ms - It ensures that the robot is still traveling and computes current distance
	
	protected void execute() {
		
		
		// Keep robot moving in the requested direction
		if (m_movingForward) driveSubsystem.forward(m_power);
		else driveSubsystem.forward(-m_power);					// Not sure if this is the correct way to move in reverse
		
		m_elapsedTime = m_timer.get();							// Return number of seconds since the timer was started
		m_distanceTraveled = m_elapsedTime * m_velocity;		// Distance traveled (feet) = elapsed time (seconds) * velocity (feet per second)
		
		if (++counter >= 50) {
			System.out.println("Executing Drive Distance Command: Elapsed time= " + m_elapsedTime + "; Distance traveled= " + m_distanceTraveled);
			counter = 0;
		}
	}
	
	// End is called when the command is terminated 

	@Override
		protected void end() {
		 	// Just stop the DriveSubsystem motors
			driveSubsystem.stop();
		}
}
