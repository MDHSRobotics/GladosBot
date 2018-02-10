package org.usfirst.frc.team4141.robot.commands;


import java.util.Hashtable;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDJoystick;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.MDSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;


public class ArcadeDriveCommand extends MDCommand {
	MDDriveSubsystem driveSys;
	public ArcadeDriveCommand(MDRobotBase robot) {
		super(robot,"ArcadeDriveCommand");
		System.out.println("in arcade drive command");
		System.out.println(robot.toString());
		Hashtable<String, MDSubsystem> subsystems = robot.getSubsystems();
		System.out.println(subsystems.toString());
		System.out.println("\n \n \n \n \n \n \n frgwesgr");
		System.out.flush();
		MDSubsystem sys = subsystems.get("driveSystem");
		System.out.println("in arcade drive command" + sys.toString());
		requires(sys);
		driveSys = (MDDriveSubsystem)sys;
    }
	
	private MDJoystick joystick = null;
	@Override
	protected void initialize() {
		super.initialize();
		joystick = getRobot().getOi().getJoysticks().get("joystick");
	}

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	driveSys.arcadeDrive(joystick);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	super.end();
    	driveSys.stop();
    }

}