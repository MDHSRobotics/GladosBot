package org.usfirst.frc.team4141.robot.subsystems;

import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.MDSubsystem;
import org.usfirst.frc.team4141.MDRobotBase.config.ConfigSetting;

public class AutonomousSubsystem extends MDSubsystem {
 
	private double autoSpeed;
	private long autoDuration;
	private MDDriveSubsystem driveSystem;
	
	
	//--------------------------------------------------------//
	
	public AutonomousSubsystem(MDRobotBase robot, String name) {
		super(robot, name);
		setCore(true);
	}
	
	//--------------------------------------------------------//

	
	@Override
	protected void setUp() {
		
		if(getConfigSettings().containsKey("autoSpeed")) autoSpeed = getConfigSettings().get("autoSpeed").getDouble();
		if(getConfigSettings().containsKey("autoDuration")) autoDuration = (long) (getConfigSettings().get("autoDuration").getDouble()*1000);
	}
	
	@Override
	public void settingChangeListener(ConfigSetting changedSetting) {
		
		if(changedSetting.getName().equals("autoSpeed")) autoSpeed = changedSetting.getDouble();
		if(changedSetting.getName().equals("autoDuration")) autoDuration = (long) (changedSetting.getDouble()*1000);
	}
	
	@Override
	protected void initDefaultCommand() {

	}
	
	//--------------------------------------------------------//
	
	public double getAutoSpeed() {
		return autoSpeed;
	}
	
	public long getAutoDuration() {
		return autoDuration;
	}
	
	public void AUTOPosOne_RLR(){
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
	}
	public void AUTOPosTwo_RLR(){
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
	}
	public void AUTOPosThree_RLR(){
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
	}
	public void AUTOPosOne_LRL(){
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
	}
	public void AUTOPosTwo_LRL(){
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
	}
	public void AUTOPosThree_LRL(){
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
	}
	public void AUTOPosOne_LLL(){
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(180);
		driveSystem.driveDistance(10, .5);
		driveSystem.turn(90);
		driveSystem.driveDistance(10, .5);
	}
	public void AUTOPosTwo_LLL(){
		driveSystem.driveDistance(10, .5);
	}
	public void AUTOPosThree_LLL(){
		driveSystem.driveDistance(10, .5);
	}
	public void AUTOPosOne_RRR(){
		driveSystem.driveDistance(10, .5);
	}
	public void AUTOPosTwo_RRR(){
		driveSystem.driveDistance(10, .5);
	}
	public void AUTOPosThree_RRR(){
		driveSystem.driveDistance(10, .5);
	}
	public void end(){
		driveSystem.stop();
	}


}
