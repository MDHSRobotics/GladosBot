package org.usfirst.frc.team4141.robot.subsystems;

import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.MDSubsystem;
import org.usfirst.frc.team4141.MDRobotBase.config.ConfigSetting;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * RopeSubsystem is a subsystem class based off the MDSubsystem.
 * This subsystem allows the use of a motor to either lift or 
 * lower the robot on a rope.
 */
public class LiftSubsystem extends MDSubsystem {
	
	private double liftSpeed=0.75;
	private SpeedController liftSpeedController;
	public static String motorName="liftSpeedController";
	private double governor = 1.0;
	
	// ------------------------------------------------ //
	
	/**
	 * This method houses the configuring of a SpeedController as a fail-safe to check 
	 * that the SpeedController is connected and ready to be used. If the SpeedController
	 * is not connected the Robot will not enable.
	 *  
	 * @return true if the SpeedController is found, false if not.
	 */
	
	public MDSubsystem configure(){
		super.configure();
		//setCore(true);
		
		if(getMotors()==null 
				|| !getMotors().containsKey(motorName))
			throw new IllegalArgumentException("Invalid motor configuration for Lift system.");
		liftSpeedController = (SpeedController)(getMotors().get(motorName));
	return this;
}
	
	/**
	 * The constructor is used to hold the parameters robot and name.
	 *  
	 * @param robot the default name used after the MDRobotBase in the constructor
	 * @param name the default name used after the string in the constructor
	 */
	public LiftSubsystem(MDRobotBase robot, String name) {
		super(robot, name);
	}
	
	// ------------------------------------------------ //

	/**
	 * This calls the variable ropeController to go in a positive direction
	 * which raises the robot up the rope.
	 */
	
//	public void arcadeLift(Joystick xbox) {
//		 // double rightTriggerValue = joystick.getRawAxis(3);
//		 //	double leftTriggerValue = -joystick.getRawAxis(2);
//			double forwardAxisValue = -xbox.getRawAxis(1);
//			double forward = (forwardAxisValue)*(1.0-(1.0-governor));
//		  	double rotate = -xbox.getRawAxis(2); //(Changed to accompass shifting w/controller and deadzoned)
//	  	  //debug("forward = " + forward + ", rotate = " + rotate);
//		  	double[] speeds = interpolator.calculate(forward, rotate);
//		    //debug("left: "+speeds[0]+", right: "+speeds[1]);
//		  	differentialDrive.tankDrive(-speeds[0], -speeds[1]);
//		}
//		
	public void raise(Joystick xbox){
		//positive speed=wind
		//negative speed=unwind
		double liftAxisValue = xbox.getRawAxis(2);
		double upwardSpeed = (liftAxisValue)*(1.0-(1.0-governor));
		liftSpeedController.set(upwardSpeed);
		debug("lift speed is at " + upwardSpeed);
	}
	
	/**
	 * This calls the variable ropeController to go in a negative direction
	 * which lowers the robot down the rope.
	 */
	public void lower(){
		//positive speed=wind
		//negative speed=unwind
		liftSpeedController.set(-liftSpeed);
	}
	
	/**
	 * This calls the variable ropeController to halt its speed to 0.
	 */
	public void stop(){
		liftSpeedController.set(0);
		// doesn't stop motor when OI has  whenPressed
		
	}
	
	// ------------------------------------------------ //

	/**
	 * This method is used for defining the variable liftSpeed to show up 
	 * on the MDConsole. 
	 */
	@Override
	protected void setUp() {
		if(getConfigSettings().containsKey("liftSpeed")) liftSpeed = getConfigSettings().get("liftSpeed").getDouble();
		if(getConfigSettings().containsKey("governor")) governor = getConfigSettings().get("governor").getDouble();
		
	}

	/**
	 * This method allows us to change the values of the variable on the fly, 
	 * without going and re-deploying the code every time we want to change the value.
	 * Instead we can now do it with the new and improved MDConsole.
	 */
	@Override
	public void settingChangeListener(ConfigSetting changedSetting) {
		if(changedSetting.getName().equals("liftSpeed")) liftSpeed = changedSetting.getDouble();
		if(changedSetting.getName().equals("governor")) governor = changedSetting.getDouble();

	}

	/**
	 * This method holds the default command used in a subsystem.
	 * Which is not being used in this subsystem. 
	 */
	@Override
	protected void initDefaultCommand() {

	}

}
