package org.usfirst.frc.team4141.robot.commands;

import javax.swing.DebugGraphics;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDJoystick;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.LiftSubsystem;


/**
 * RopeRiseCommand is a command class based off the MDCommand.
 * This command calls the ropeSubsystem, which in this case 
 * uses motors to lift the robot up until a desired location
 * is met by the driver. 
 * 
 * @see RopeSubsystem
 */
public class RiseCommand extends MDCommand {
	
	private LiftSubsystem liftSubsystem;
	
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
	public RiseCommand(MDRobotBase robot) {
		super(robot, "RiseCommand");
		if(!getRobot().getSubsystems().containsKey("liftSubsystem")){
			log(Level.ERROR, "initialize()", "lift subsystem not found");
			throw new IllegalArgumentException("lift Subsystem not found");
		}
		liftSubsystem = (LiftSubsystem)getRobot().getSubsystems().get("liftSubsystem"); 
		requires(liftSubsystem);
	}

	// ------------------------------------------------ //
	
	/**
	 * When the command first starts nothing happens.
	 */
	
	private MDJoystick xbox = null;
	
	protected void initialize() {
			super.initialize();
			xbox = getRobot().getOi().getJoysticks().get("xbox");
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
		if (liftSubsystem!=null)liftSubsystem.raise(xbox);
		log(Level.DEBUG,"execute()","Raiseing");
	}
	
	/**
	 * This signifies the end of the command by stopping the ropeSubsystem motors
	 * due to the halt of input by the driver.
	 */
	@Override
		protected void end() {
		super.end();
		liftSubsystem.stop();
			
		}
}
