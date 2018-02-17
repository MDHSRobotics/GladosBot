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
public class ExtendSubsystem extends MDSubsystem {
	
	private double extendSpeed=0.45;
	private SpeedController extendSpeedController;
	public static String extendclawMotorName="extendSpeedController";
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
				|| !getMotors().containsKey(extendclawMotorName))
			throw new IllegalArgumentException("Invalid motor configuration for Claw system.");
		extendSpeedController = (SpeedController)(getMotors().get(extendclawMotorName));
	return this;
	
	
}
	
	/**
	 * The constructor is used to hold the parameters robot and name.
	 *  
	 * @param robot the default name used after the MDRobotBase in the constructor
	 * @param name the default name used after the string in the constructor
	 */
	public ExtendSubsystem(MDRobotBase robot, String name) {
		super(robot, name);
		System.out.println("\n \n \n \n \n \n \n \n \n Constructing claw subsystem " + name);
	}
	
	// ------------------------------------------------ //
	/**
	 * This calls the variable ropeController to halt its speed to 0.
	 */
	public void extend(Joystick xbox){
		//positive speed=wind
		//negative speed=unwind
		double upwardAxisValue = xbox.getRawAxis(1);
		double upwardSpeed = (upwardAxisValue)*(1.0-(1.0-governor));
//		double downwardAxisValue = xbox.getRawAxis(3);
//		double downwardSpeed = (downwardAxisValue)*(1.0-(1.0-governor));
//		double moveSpeed = upwardSpeed-downwardSpeed;
		extendSpeedController.set(upwardSpeed);
		//debug("lift speed is at " + upwardSpeed);
	}
	
	
	public void stopextend(){
		extendSpeedController.stopMotor();
		
	}
	// ------------------------------------------------ //

	/**
	 * This method is used for defining the variable liftSpeed to show up 
	 * on the MDConsole. 
	 */
	@Override
	protected void setUp() {
//		if(getConfigSettings().containsKey("extendSpeed")) extendSpeed = getConfigSettings().get("clawSpeed").getDouble();
		if(getConfigSettings().containsKey("governor")) governor = getConfigSettings().get("governor").getDouble();
	}

	/**
	 * This method allows us to change the values of the variable on the fly, 
	 * without going and re-deploying the code every time we want to change the value.
	 * Instead we can now do it with the new and improved MDConsole.
	 */
	@Override
	public void settingChangeListener(ConfigSetting changedSetting) {
		if(changedSetting.getName().equals("extendSpeed")) extendSpeed = changedSetting.getDouble();
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
