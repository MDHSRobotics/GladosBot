package org.usfirst.frc.team4141.robot.commands;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.AutonomousSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;


/**
 * RopeRiseCommand is a command class based off the MDCommand.
 * This command calls the ropeSubsystem, which in this case 
 * uses motors to lift the robot up until a desired location
 * is met by the driver. 
 * 
 * @see RopeSubsystem
 */
public class AUTOPosThree_RRR extends MDCommand {
	
	private AutonomousSubsystem autoSubsystem;
	
	// ------------------------------------------------ //
	
	/**
	 * The constructor is used to hold the parameters robot and name.
	 * Within the constructor is a fail-safe to check that the ropeSubsystem
	 * is connected and ready to be used. If the ropeSubsystem is not connected 
	 * the Robot will not enable.
	 *  
	 * @param robot the default name used after the MDRobotBase in the constructor
	 * @param name the default name used after the string in the constructor
	 * @return true if the ropeSubsystem is found, false if not.
	 */
	public AUTOPosThree_RRR(MDRobotBase robot, String name) {
		super(robot, name);
		if(!getRobot().getSubsystems().containsKey("autoSubsystem")){
			log(Level.ERROR, "initialize()", "Autonomous subsystem not found");
			throw new IllegalArgumentException("Autonomous Subsystem not found");
		}
		autoSubsystem = (AutonomousSubsystem)getRobot().getSubsystems().get("autoSubsystem"); 
		requires(autoSubsystem);
	}

	// ------------------------------------------------ //
	
	/**
	 * When the command first starts nothing happens.
	 */
	protected void initialize() {
		}
	
	/**
	 * The robot does not stop the procedure until it is told by the driver.
	 * 
	 * @return false because the command never ends by itself.
	 */
	protected boolean isFinished() {
		return false;
	}
	
	/**
	 * This method runs the rope system in the upwards direction until 
	 * it reads no input from the driver. 
	 */
	protected void execute() {
		if (autoSubsystem!=null)autoSubsystem.AUTOPosThree_RRR();
		log(Level.DEBUG,"execute()","Position 3 RRR");
		autoSubsystem.AUTOPosThree_RRR();
	}
	
	/**
	 * This signifies the end of the command by stopping the ropeSubsystem motors
	 * due to the halt of input by the driver.
	 */
	@Override
		protected void end() {
			
		autoSubsystem.end();
			
		}
}
