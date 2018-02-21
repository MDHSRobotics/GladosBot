package org.usfirst.frc.team4141.robot.autocommands;

import org.usfirst.frc.team4141.MDRobotBase.MDCommandGroup;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.robot.commands.MDPrintCommand;



public class AUTOPosOne_LLL extends MDCommandGroup {
	

	public AUTOPosOne_LLL(MDRobotBase robot, String name) {
		super(robot, name);
		
		addSequential(new MDPrintCommand(robot, name, "Excecuting command group " + name));
		// Need to insert a Wait command here with a variable wait time
		addSequential(new WaitCommand(robot, "STEP 0: Wait Command", 5.));
		addSequential(new DriveDistanceCommand(robot, "STEP 1: DriveDistanceCommand", 10, .5));
		addSequential(new TurnCommand(robot, "STEP 2: TurnCommand", 180, .5));
		addSequential(new DriveDistanceCommand(robot, "STEP 3: DriveDistanceCommand", 10, .5));
		addSequential(new TurnCommand(robot, "STEP 4: TurnCommand", 90, .5));
		addSequential(new DriveDistanceCommand(robot, "STEP 5: DriveDistanceCommand", 10, .5));

	}
	
}

 