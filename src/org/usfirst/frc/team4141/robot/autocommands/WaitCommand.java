package org.usfirst.frc.team4141.robot.autocommands;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;


public class WaitCommand extends MDCommand {
	
	private double m_elapsedTime;				// Time (in seconds) that this command has executed
	private Timer m_timer; 						// Timer for this command
	private double m_waitTime;
	
	private int counter;
	
	// ------------------------------------------------ //
	
	/**
	 * Constructor for the WaitCommand
	 * Within the constructor is a fail-safe to check that the Drive Subsystem
	 * is connected and ready to be used. If the Drive Subsystem is not connected 
	 * the Robot will not enable.
	 *  
	 * @param robot - the robot object
	 * @param name - name of this WaitCommand
	 * @param targetDistanceInFeet - Desired distance to travel (in feet) - NOTE: Negative means move backwards
	 * @param power - Power setting for drive: 0.0 to +1.0
	 */
	public WaitCommand(MDRobotBase robot, String name, double waitTime) {
		super(robot, name);
	
		m_waitTime = waitTime;
		m_elapsedTime = 0.;
		m_timer = new Timer();
	}

	// Initialize is called when the command first starts
	 
	protected void initialize() {
		counter = 0;
		m_elapsedTime = 0.;
		m_timer.reset();
		m_timer.start();
		System.out.println("Starting "+ this.getName() +"; Target wait time= " + m_waitTime + " seconds");
	}
		
	// isFinished is called every 20ms to determine whether the robot has traveled the requested distance
	
	protected boolean isFinished() {
		if(m_elapsedTime >= m_waitTime){
			m_timer.stop();
			System.out.println("Finished "+ this.getName() + "; Elapsed time= " + m_elapsedTime + " seconds");
			return true;
		}
		else {
			return false;
		}
	}
	
	// Execute is called every 20ms - It ensures that the robot is still traveling and computes current distance
	
	protected void execute() {
		
		m_elapsedTime = m_timer.get();							// Return number of seconds since the timer was started
		
		if (++counter >= 50) {
			System.out.println("Executing Wait Command: Elapsed time= " + m_elapsedTime + " seconds");
			counter = 0;
		}
	}
	
	// End is called when the command is terminated 

	@Override
		protected void end() {
		}
}
