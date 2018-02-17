package org.usfirst.frc.team4141.robot.autocommands;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;

import edu.wpi.first.wpilibj.Timer;


//  Command to rotate robot

public class TurnCommand extends MDCommand {
	
	private double m_targetAngle;			// Target angle to rotate in degrees (positive is clock-wise)
	private double m_power;
	private double m_currentAngle;
	private double m_elapsedTime;
	private int m_counter;
	private Timer m_timer; 	
	private double m_angularVelocity;			        // Angular velocity (degrees/second) at current power setting
	private boolean m_turningRight;						// True if turning right; False if turning left 
	
	private double angularVelocityAtFullPower = 70;     // Angular velocity (degrees/second) at full power - THIS IS A GUESS - CHECK IT!!
	
	private MDDriveSubsystem driveSubsystem;
	
	// ------------------------------------------------ //
	
	/**
	 * Constructor for the TurnCommand
	 * Within the constructor is a fail-safe to check that the Drive Subsystem
	 * is connected and ready to be used. If the Drive Subsystem is not connected 
	 * the Robot will not enable.
	 *  
	 * @param robot - the robot object
	 * @param name - name of this DriveDistanceCommand
	 * @param targetAngle - Desired angle to turn (in degrees) - NOTE: Positive angles result in clockwise rotation
	 * @param power - Power setting for drive: 0.0 to +1.0
	 */
	
	public TurnCommand(MDRobotBase robot, String name, double targetAngle, double power) {
		super(robot, name);
		
		if(!getRobot().getSubsystems().containsKey("driveSystem")){
			log(Level.ERROR, "initialize()", "Drive subsystem not found");
			throw new IllegalArgumentException("Drive Subsystem not found");
		}
		driveSubsystem = (MDDriveSubsystem)getRobot().getSubsystems().get("driveSystem"); 
		requires(driveSubsystem);
		
		m_turningRight = (targetAngle > 0);       // Use sign of targetAngle to determine whether turning right or left
		m_targetAngle = targetAngle;
		m_power = power;
		m_currentAngle = 0.;
		m_elapsedTime = 0.;
		m_timer = new Timer();
		m_angularVelocity = m_power * angularVelocityAtFullPower;  		// Scale velocity at full power by the current power (which is between 0 and 1.0)
	}

	
	 // Initialize is called when the command first starts
	 
	protected void initialize() {
		//	Initialize timer and other stuff
		m_timer.reset();
		m_timer.start();
		m_elapsedTime = 0.;
		m_counter = 0; 
		System.out.println("Starting " + this.getName() + "; Target = " + m_targetAngle + " degrees" + "; Power =" + m_power);
	}
		
	// isFinished is called every 20ms to determine whether the robot has rotated the requested angle
	
	protected boolean isFinished() {
		
		if(m_targetAngle < 0.){
			
			// Negative target angle - that is, counter-clockwise rotation
			if(m_currentAngle <= m_targetAngle ){
				m_timer.stop();
				System.out.println("Finished " + this.getName()+ "; Target = " + m_targetAngle + "; Actual = " + m_currentAngle + "Elapsed time = " + m_elapsedTime);
				return true;
			}
			else {
				return false;
			}
		}
		else {
			
			// Positive target angle - that is, clockwise rotation
			if(m_currentAngle >= m_targetAngle){
				m_timer.stop();
				System.out.println("Finished " + this.getName() + "; Target = " + m_targetAngle + "; Actual = " + m_currentAngle + "Elapsed time = " + m_elapsedTime);
				return true;
			}
			else {
				return false;
			}
		}
		
	}
	
	
	// Execute is called every 20ms - It ensures that the robot is still traveling and computes current angle
	
	protected void execute() {
		// Keep robot moving in the requested direction
		if (m_turningRight) driveSubsystem.pivot(m_power);
		else driveSubsystem.pivot(-m_power);					
		
		m_elapsedTime = m_timer.get();		// Return number of seconds since the timer was started
		
		m_currentAngle = m_elapsedTime * m_angularVelocity;		// Degrees turned (degrees) = elapsed time (seconds) * angular velocity (degrees per second)
		
		// Need to somehow tell the drive subsystem to rotate (pivot) the robot
		
		if(++m_counter >= 50){
			System.out.println("Executing Turn Command: Elapsed time= " + m_elapsedTime + "; Current angle = " + m_currentAngle);
			m_counter = 0;
		}
		
	}
		
	// End is called when the command is terminated. 
	
	@Override
		protected void end() {
	 		// Just stop the DriveSubsystem motors
			driveSubsystem.stop();
		}
}
