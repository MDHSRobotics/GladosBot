package org.usfirst.frc.team4141.robot;

//===================================================================== Imported Systems ===================================================================== //
// Base Systems
import org.usfirst.frc.team4141.MDRobotBase.ConsoleOI;
import org.usfirst.frc.team4141.MDRobotBase.MDJoystick;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.OIBase;
import org.usfirst.frc.team4141.MDRobotBase.RioHID;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosOne_LLL;
import org.usfirst.frc.team4141.robot.autocommands.ClosedLoopDriveDistanceCommand;
// Commands
import org.usfirst.frc.team4141.robot.commands.ExtendCommand;
//import org.usfirst.frc.team4141.robot.commands.MDMoveCommand;
//import org.usfirst.frc.team4141.robot.commands.MDMoveCommand.Direction;
import org.usfirst.frc.team4141.robot.commands.MDPrintCommand;
import org.usfirst.frc.team4141.robot.commands.ClawCommand;
import org.usfirst.frc.team4141.robot.commands.LiftCommand;
import org.usfirst.frc.team4141.robot.commands.ToggleOrientationCommand;

// ===================================================================== Unused Imports ===================================================================== //
// import org.usfirst.frc.team4141.robot.commands.MDMoveCommand;
// import org.usfirst.frc.team4141.robot.commands.MDMoveCommand.Direction;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * We use devices including but not limited to:
 * 	Xbox Controller
 * 	Flight Simulator Controller
 * 
 */

public class OI extends OIBase{
	
	public OI(MDRobotBase robot) {
		super(robot);
		System.out.println("OI created");
	}


	public void configureOI(){
		
		/**
		 * 
		 * A robot should have 1 or more operator interfaces (OI)
		 * Typically, a robot will have at least 1 joystick and 1 console (MDConsole in our instance)
		 * We control what buttons are mapped in both the console and the joystick.
		 * We configure the joysticks below.
		 * 
		 */	
	
					// === Joystick Mapping Structure === //
/*
     	add(new MDJoystick(getRobot(), "[joystickName]", [JOYSTICK NUMBER]);
		    .("[JOYSTICK_BUTTON_NAME]",[JOYSTICK BUTTON NUMBER],new [COMMAND NAME]))
			.whileHeld("[JOYSTICK_BUTTON_NAME]",[JOYSTICK BUTTON NUMBER],new [COMMAND NAME]))
			.configure()
		);
*/
		
		add(new MDJoystick(getRobot(), "joystick", 0)

			//.whenPressed("rightBumper",5,new MDPrintCommand(getRobot(),"Right Bumper Command","Right Bumper Command message"))
			//.whileHeld("leftBumper",6,new MDPrintCommand(getRobot(),"Left Bumper Command","Left Bumper Command message"))
		    //the following commands are test move commands useful in testing drive configuration and set up
		    //comment out and replace as needed
			//.whenPressed("X",1,new MDMoveCommand(getRobot(),"left command",Direction.left))
			//.whenPressed("A",2,new MDMoveCommand(getRobot(),"reverse command",Direction.reverse))
			//.whenPressed("B",3,new MDMoveCommand(getRobot(),"right command",Direction.right))
			//.whenPressed("Y",4,new MDMoveCommand(getRobot(),"forward command",Direction.forward))
//			.whileHeld("5",5,new RiseCommand(getRobot(),"RiseCommand"))
//			.whileHeld("3",3,new LowerCommand(getRobot(),"LowerCommand"))
//			.whileHeld("6",6,new OpenClaw(getRobot(),"OpenClaw"))
//			.whileHeld("4",4,new CloseClaw(getRobot(),"CloseClaw"))
//			.whileHeld("1",1,new ExtendCommand(getRobot(),"ExtendCommand"))
//			.whileHeld("2",2,new RetractCommand(getRobot(),"RetractCommand"))
//			.whenPressed("4",4,new ClosedLoopDriveDistanceCommand(getRobot(), "ClosedLoopDriveDistanceCommand", 10, 2, true))	
			.whenPressed("5",5,new ToggleOrientationCommand(getRobot(), "ToggleOrientationCommand"))
			.configure()
		);
		
		add(new MDJoystick(getRobot(), "xbox", 1)
//				.whileHeld("5",5,new RiseCommand(getRobot(),"RiseCommand"))
//				.whileHeld("6",6,new LowerCommand(getRobot(),"LowerCommand"))
//				.whileHeld("3",3,new OpenClaw(getRobot(),"OpenClaw"))
//				.whileHeld("4",4,new CloseClaw(getRobot(),"CloseClaw"))
//				.whileHeld("1",1,new ExtendCommand(getRobot(),"ExtendCommand"))
//				.whileHeld("2",2,new RetractCommand(getRobot(),"RetractCommand"))
				.configure()
			);

//                               Configure the RioHID 
//         Uncomment the following to attach a command to the user button on the RoboRIO. 
		// ============================================================================= //
		add(new RioHID(getRobot())
			.whileHeld(new MDPrintCommand(getRobot(),"ExampleCommand1","ExampleCommand1 message"))
			.configure()
		);
		
		
//                         Configure the MDConsole "Joystick"
//          We add commands here that will show up as buttons in the MDConsole
		// ==================================================================== //
		add(new ConsoleOI(getRobot())
				.whileHeld("LiftCommand",5,new LiftCommand(getRobot()))
				.whileHeld("ClawCommand",3,new ClawCommand(getRobot()))
				.whileHeld("ExtendCommand",1,new ExtendCommand(getRobot()))
				.configure()
			);		
		
	}

}  


// ========================================== Unused Code ========================================== //

// .whenPressed("X",1,new MDMoveCommand(getRobot(),"left command",Direction.left))
// .whenPressed("A",2,new MDMoveCommand(getRobot(),"reverse command",Direction.reverse))
// .whenPressed("B",3,new MDMoveCommand(getRobot(),"right command",Direction.right))
// .whenPressed("Y",4,new MDMoveCommand(getRobot(),"forward command",Direction.forward))
// .whenPressed("rightBumper",5,new MDPrintCommand(getRobot(),"Right Bumper Command","Right Bumper Command message"))
// .whileHeld("leftBumper",6,new MDPrintCommand(getRobot(),"Left Bumper Command","Left Bumper Command message"))