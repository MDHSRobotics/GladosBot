package org.usfirst.frc.team4141.robot;

//===================================================================== Imported Systems ===================================================================== //
//import java.util.Hashtable;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDCommandGroup;
import org.usfirst.frc.team4141.MDRobotBase.sensors.MD_BuiltInAccelerometer;
import org.usfirst.frc.team4141.MDRobotBase.sensors.MD_IMU;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.MDSubsystem;
import org.usfirst.frc.team4141.MDRobotBase.MDTalonSRX;
import org.usfirst.frc.team4141.MDRobotBase.config.DoubleConfigSetting;
import org.usfirst.frc.team4141.MDRobotBase.config.IntegerConfigSetting;
import org.usfirst.frc.team4141.MDRobotBase.config.StringConfigSetting;
import org.usfirst.frc.team4141.robot.autocommands.DriveDistanceCommand;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosOne_LLL;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosThree_RRR;
import org.usfirst.frc.team4141.robot.commands.ClawCommand;
import org.usfirst.frc.team4141.robot.commands.ExtendCommand;
import org.usfirst.frc.team4141.robot.commands.MDPrintCommand;
import org.usfirst.frc.team4141.robot.commands.LiftCommand;
import org.usfirst.frc.team4141.robot.subsystems.AutonomousSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.ClawSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.CoreSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.ExtendSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem.MotorPosition;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem.Type;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

/**
 * This system is the entire brain of the robot.
 * A brain takes to different parts of the body and tells it what to do (right?).
 * We assign motors and what positions to different subsystems.
 * A robot is composed of subsystems
 * A robot will typically have one (1) drive system and several other fit to purpose subsystems
 */

public class Robot extends MDRobotBase {

	// This runs as soon as we press enable in Driver Station (the first thing it does essentially).
	
	public void teleopInit() {
		super.teleopInit();

		LiftCommand liftCommand = new LiftCommand(this);
		liftCommand.start();
		ExtendCommand extendCommand = new ExtendCommand(this);
		extendCommand.start();
		ClawCommand clawCommand = new ClawCommand(this);
		clawCommand.start();


	}
	

	
	// ================================================================================ Robot Configuration ========================================================================== //
	
	@Override
	protected void configureRobot() {		
	
		// The name needs to be updated every year to keep track.
		// The AutoCommand changes every year and is based off the competition and team agreement.
		
		debug("\nEnter configured Robot");
		add( new CoreSubsystem(this, "core")
				 .add("name",new StringConfigSetting("GladosBot")) // <--- Name
				 .add("autoCommand",new StringConfigSetting("AUTOPosOne_LLL")) // <--- Default AutoCommand
				 .configure()
		);	
		

		// ================================================== Drive Subsystem Configuration ==================================================================== //				
		
		/*********************************************************************************
		 * 																				 *
		 * 	There are two (2) different types of drive systems.							 *
		 * 	TankDrive: Typically the one used by default. Simple as it gets.			 *
		 * 	MecanumDrive: Allows moving left and right without turning but not as fast.	 *
		 *	Each drive type is programmed and hot-swappable below.						 *
		 * 																				 *
		 *********************************************************************************
		 */

		MDDriveSubsystem driveSubsystem = new MDDriveSubsystem(this, "driveSystem", Type.TankDrive);
		add(driveSubsystem);

//		System.out.println("Hashtable after adding drive");
//		Hashtable<String, MDSubsystem> ht = this.getSubsystems();
//		System.out.println(ht.toString());
		driveSubsystem.add("accelerometer", new MD_BuiltInAccelerometer());
//		System.out.println("\nMDDrive after adding accelerometer");
//		System.out.println(driveSubsystem.toString());
		driveSubsystem.add("IMU", new MD_IMU())
				.add(MotorPosition.frontLeft, new MDTalonSRX(1))
				.add(MotorPosition.frontRight, new MDTalonSRX(3))
				.add(MotorPosition.rearLeft, new MDTalonSRX(2))
				.add(MotorPosition.rearRight, new MDTalonSRX(4))
				//.add("Drive-F", new DoubleConfigSetting(0.0, 1.0, 0.0))
		 	    //.add("Drive-P", new DoubleConfigSetting(0.0, 1.0, 0.1))
				//.add("Drive-I", new DoubleConfigSetting(0.0, 1.0, 0.8))
				//.add("Drive-D", new DoubleConfigSetting(0.0, 1.0, 0.1))
				.add("Ramp Time In seconds", new DoubleConfigSetting(0.0, 10.0, 1.0))
				.add("forwardSpeed", new DoubleConfigSetting(0.0, 1.0, 0.25)) //High Speed - Turn Factor
		 	    .add("rotateSpeed", new DoubleConfigSetting(0.0, 1.0, 1.0)) //Slow Speed - Turn Factor
				.add("governor", new DoubleConfigSetting(0.0, 1.0, 1.0)); //Speed Governor

// 							These are used for MecanumDrive
				// ==================================================== //
/*				.add(MotorPosition.rearLeft, new MDTalonSRX(1))
				.add(MotorPosition.rearRight, new MDTalonSRX(2))                               */

//			Enable these if you want to configure PID values within MDConsole
				// ==================================================== //
/* 				.add("Drive-F", new DoubleConfigSetting(0.0, 1.0, 0.0))
              	.add("Drive-P", new DoubleConfigSetting(0.0, 1.0, 0.1))
 				.add("Drive-I", new DoubleConfigSetting(0.0, 1.0, 0.8))
				.add("Drive-D", new DoubleConfigSetting(0.0, 1.0, 0.1))                          */

				driveSubsystem.configure();
			
		// =================================================== Other Subsystems ======================================================================== //		
				
		/************************************************************************************
		 * 																					*
		 * 	Developing a subsystem is very simple.											*
		 * 	We need the name, the string, and whatever motors and configurations we want.	*
		 * 	Each new subsystem must be identified to the correct							*
		 * 																					*
		 ************************************************************************************
		 */		
				
				// === Joystick Mapping Structure === //
/*
 		add(new [SUBSYSTEM_NAME](this, "[subsystemName]")
	    	.add("scenario1Speed", new DoubleConfigSetting(0.0, 1.0, 0.5))
	    	.add([SUBSYSTEM NAME.motorName, new MDTalonSRX(1))
				.configure()
		);
*/
		
			
				
				
		add(new AutonomousSubsystem(this, "autoSubsystem")
				.add("delayStartTime", new DoubleConfigSetting(0.0, 15.0, 0.0))
				.add("startingPosition", new IntegerConfigSetting(1, 3, 1))
				.configure()
		);
		
		add(new LiftSubsystem(this, "liftSubsystem")
				.add(LiftSubsystem.liftMotor1, new MDTalonSRX(7))
				.add(LiftSubsystem.liftMotor2, new MDTalonSRX(8))
				.add("liftSpeed", new DoubleConfigSetting(0.0, 1.0, 1.0))
				.add("governor", new DoubleConfigSetting(0.0, 1.0, 1.0)) //Speed Governor
				.configure()
		);
		
		ClawSubsystem clawSubsystem = new ClawSubsystem(this, "clawSubsystem");
		add(clawSubsystem);
		clawSubsystem.add(ClawSubsystem.clawMotorName, new MDTalonSRX(5))
				.add("clawSpeed", new DoubleConfigSetting(0.0, 1.0, 1.0))
				.add("governor", new DoubleConfigSetting(0.0, 1.0, 1.0)) //Speed Governor
				.configure();
		
		ExtendSubsystem extendSubsystem = new ExtendSubsystem(this, "extendSubsystem");
		add(extendSubsystem);
				extendSubsystem.add(ExtendSubsystem.extendclawMotorName, new MDTalonSRX(6))
				.add("extendSpeed", new DoubleConfigSetting(0.0, 1.0, 1.0))
				.add("governor", new DoubleConfigSetting(0.0, 1.0, 1.0)) //Speed Governor
				.configure();
				
		initAutoCommands();				// Create all of the possible command groups
				
		debug("\n \n \n Done configuring the Robot.");
		debug("Printing the state of the Robot...");
		debug(this.toString());
		
	}
		
		private void initAutoCommands(){
			MDCommandGroup[] autoCommandArray = new MDCommandGroup[2];
			//autoCommandArray[0] = new  MDPrintCommand(this,"AutonomousCommand","AutonomousCommand message");
			autoCommandArray[0] = new  AUTOPosOne_LLL(this,"AUTOPosOne_LLL");
//			autoCommandArray[1] = new  AUTOPosOne_LRL(this,"AUTOPosOne_LRL");
//			autoCommandArray[2] = new  AUTOPosOne_RLR(this,"AUTOPosOne_RLR");
//			autoCommandArray[3] = new  AUTOPosOne_RRR(this,"AUTOPosOne_RRR");
//			
//			autoCommandArray[4] = new  AUTOPosTwo_LLL(this,"AUTOPosTwo_LLL");
//			autoCommandArray[5] = new  AUTOPosTwo_LRL(this,"AUTOPosTwo_LRL");
//			autoCommandArray[6] = new  AUTOPosTwo_RLR(this,"AUTOPosTwo_RLR");
//			autoCommandArray[7] = new  AUTOPosTwo_RRR(this,"AUTOPosTwo_RRR");
//			
//			autoCommandArray[8] = new  AUTOPosThree_LLL(this,"AUTOPosThree_LLL");
//			autoCommandArray[9] = new  AUTOPosThree_LRL(this,"AUTOPosThree_LRL");
//			autoCommandArray[10] = new  AUTOPosThree_RLR(this,"AUTOPosThree_RLR");
			autoCommandArray[1] = new  AUTOPosThree_RRR(this,"AUTOPosThree_RRR");

			setAutonomousCommand(autoCommandArray, "AUTOPosOne_LLL"); 
	}
	
// =================================================== Autonomous Configurations ======================================================================== //				
		
		public void autonomousInit() {
			String commandName = null;
			String colorAssignment;
			
			System.out.println("\n\n========================\n=== Starting Autonomous Mode ===\n=======================");

			// Get the starting position for this match
			AutonomousSubsystem autoSubsystem = (AutonomousSubsystem) getSubsystem("autoSubsystem");
			int startingPosition = autoSubsystem.getStartingPosition();
			System.out.println("Starting Position = " + startingPosition);
			
			// Get color assignment for this match
			colorAssignment = getColorAssignment();
			System.out.println("Match Color Assignment: " + colorAssignment);
			
			switch(startingPosition){
				case 1:
					commandName = "AUTOPos" + "One" + "_" + colorAssignment;
					break;
				case 2:
					commandName = "AUTOPos" + "Two" + "_" + colorAssignment;
					break;
				case 3:
					commandName = "AUTOPos" + "Three" + "_" + colorAssignment;
					break;
			
			}

			System.out.println("Setting auto command to " + commandName);
			System.out.flush();
			
			setAutoCommand(commandName);
			
			if (autonomousCommand != null) {
				debug("autonomous command should start");
				autonomousCommand.start();
			}
			else {
				System.out.println("autonomousCommand is unexpectedly null");
			}
			
		}


		private String getColorAssignment(){
			String matchColorAssignment = new String();
			// Insert code here to read color assignment from FMS
			String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
			//make sure to use only the first 3 characters of the game message
			matchColorAssignment = gameMessage.substring(0,3);
			
			System.out.println("Get Color Assignment: " + matchColorAssignment);

			return matchColorAssignment;
			
		}
		
	// =================================================================================================================================================== //		
	
}


// ===================================================================== Unused Code ======================================================================= //


// This sets the default AutonomousCommand in MDConsole.
// In some cases it is desirable to have more than 1 auto command and make a decision at game time which command to use
// ===================================================================================================================
/*
 	setAutonomousCommand(new MDCommand[]{

		new MDPrintCommand(this,"AutonomousCommand","AutonomousCommand message")
	}, "AutonomousCommand"  //specify the default
);
*/


// ======================================================================
//System.out.println("Hashtable after adding drive");
//Hashtable<String, MDSubsystem> ht = this.getSubsystems();
//System.out.println(ht.toString());
//======================================================================


//======================================================================
// add(new MDDriveSubsystem(this, "driveSystem", Type.TankDrive)
//======================================================================


//add(new ClawSubsystem(this, "clawSubsystem")
//.add(ClawSubsystem.motorName, new MDTalonSRX(2))
//.add(ClawSubsystem.motorName2, new MDTalonSRX(0))
//.add("clawSpeed", new DoubleConfigSetting(0.0, 1.0, 0.5))
//.configure()
//);


/*

@Override
public void teleopPeriodic() {
	super.teleopPeriodic();
}
	
@Override
public void autonomousPeriodic() {
	super.autonomousPeriodic();
}	

@Override
public void onConnect(Session session) {
	super.onConnect(session);

}

*/	
