package org.usfirst.frc.team4141.robot.autocommands;

import org.usfirst.frc.team4141.MDRobotBase.MDCommandGroup;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;



public class AUTOPosOne_LLL extends MDCommandGroup {
	

	public AUTOPosOne_LLL(MDRobotBase robot, String name) {
		super(robot, name);
		
		// Need to insert a Wait command here with a variable wait time
		
		addSequential(new ClosedLoopDriveDistanceCommand(robot, "STEP 1: ClosedLoopDriveDistanceCommand", 10, 1.5, false));
//		addSequential(new TurnCommand(robot, "STEP 2: TurnCommand", 180, .5));
//		addSequential(new DriveDistanceCommand(robot, "STEP 3: DriveDistanceCommand", 10, .5, false));
//		addSequential(new TurnCommand(robot, "STEP 4: TurnCommand", 90, .5));
//		addSequential(new DriveDistanceCommand(robot, "STEP 5: DriveDistanceCommand", 10, .5, false));

	}
	
}

