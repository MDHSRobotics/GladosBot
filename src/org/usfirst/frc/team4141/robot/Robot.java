package org.usfirst.frc.team4141.robot;

//===================================================================== Imported Systems ===================================================================== //
import java.util.Hashtable;

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDCommandGroup;
import org.usfirst.frc.team4141.MDRobotBase.sensors.MD_BuiltInAccelerometer;
import org.usfirst.frc.team4141.MDRobotBase.sensors.MD_IMU;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.MDSubsystem;
import org.usfirst.frc.team4141.MDRobotBase.config.DoubleConfigSetting;
import org.usfirst.frc.team4141.MDRobotBase.config.StringConfigSetting;
import org.usfirst.frc.team4141.robot.Robot.fieldPosition;
import org.usfirst.frc.team4141.robot.autocommands.DriveDistanceCommand;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosOne_LLL;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosOne_LRL;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosOne_RLR;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosOne_RRR;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosThree_LLL;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosThree_LRL;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosThree_RLR;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosThree_RRR;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosTwo_LLL;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosTwo_LRL;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosTwo_RLR;
import org.usfirst.frc.team4141.robot.autocommands.AUTOPosTwo_RRR;
import org.usfirst.frc.team4141.robot.commands.ArcadeDriveCommand;
import org.usfirst.frc.team4141.robot.commands.MDPrintCommand;
import org.usfirst.frc.team4141.robot.commands.RiseCommand;
import org.usfirst.frc.team4141.robot.subsystems.AutonomousSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.ClawSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.CoreSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem.MotorPosition;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem.Type;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
		RiseCommand riseCommand = new RiseCommand(this);
		riseCommand.start();
	}
	
	public enum fieldPosition {
		ONE,
		TWO,
		THREE
	}
	
	// ================================================================================ Robot Configuration ========================================================================== //
	
	private fieldPosition startingPosition = fieldPosition.ONE;
	
	@Override
	protected void configureRobot() {		
	
		// The name needs to be updated every year to keep track.
		// The AutoCommand changes every year and is based off the competition and team agreement.
		
		debug("\nEnter configured Robot");
		add( new CoreSubsystem(this, "core")
				 .add("name",new StringConfigSetting("GladosBot")) // <--- Name
				 .add("autoCommand",new StringConfigSetting("[Insert Command Here]")) // <--- Default AutoCommand
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
			
//				   DriveSystem factors including Gyro and Accelerometer
				// ==================================================== //
				driveSubsystem.add("accelerometer", new MD_BuiltInAccelerometer());
				driveSubsystem.add("IMU", new MD_IMU())

//						Initial Driving Motors (Kind of important)
				// ==================================================== //				
				.add(MotorPosition.left, new WPI_TalonSRX(4))
				.add(MotorPosition.right, new WPI_TalonSRX(3))
				
//				Speed Governor limits how fast the driving speed (1.0 = 100%)
				// ==================================================== //
				.add("highSpeed", new DoubleConfigSetting(0.0, 1.0, 0.25)) //High Speed - Turn Factor
		 	    .add("lowSpeed", new DoubleConfigSetting(0.0, 1.0, 0.4)) //Slow Speed - Turn Factor
				.add("governor", new DoubleConfigSetting(0.0, 1.0, 1.0)); //Speed Governor

// 							These are used for MecanumDrive
				// ==================================================== //
/*				.add(MotorPosition.rearLeft, new WPI_TalonSRX(1))
				.add(MotorPosition.rearRight, new WPI_TalonSRX(2))                               */

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
	    	.add([SUBSYSTEM NAME.motorName, new WPI_TalonSRX(1))
				.configure()
		);
*/
				
		add(new AutonomousSubsystem(this, "autoSubsystem")
				.add("scenario1Speed", new DoubleConfigSetting(0.0, 1.0, 0.5))
				.configure()
		);
		
		add(new LiftSubsystem(this, "liftSubsystem")
				.add(LiftSubsystem.motorName, new WPI_TalonSRX(1))
				.add("liftSpeed", new DoubleConfigSetting(0.0, 1.0, 0.5))
				.configure()
		);
		
		ClawSubsystem clawSubsystem = new ClawSubsystem(this, "clawSubsystem");
		add(clawSubsystem);
		clawSubsystem.add(ClawSubsystem.clawMotorName, new WPI_TalonSRX(5))
				.add(ClawSubsystem.extendclawMotorName, new WPI_TalonSRX(6))
				.add("clawSpeed", new DoubleConfigSetting(0.0, 1.0, 0.5))
				.configure();
	}
		
		private void initAutoCommands(){
			MDCommandGroup[] autoCommandArray = new MDCommandGroup[1];
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
//			autoCommandArray[11] = new  AUTOPosThree_RRR(this,"AUTOPosThree_RRR");

			setAutonomousCommand(autoCommandArray, "AUTOPosOne_LLL"); 
		
		debug("\n \n \n Done configuring the Robot.");
		debug("Printing the state of the Robot...");
		debug(this.toString());

	}
	
// =================================================== Autonomous Configurations ======================================================================== //				
		
		public void autonomousInit() {
			String commandName = null;
			String colorAssignment;
			
			// Get color assignment for this match
			colorAssignment = getColorAssignment();
			
			switch(startingPosition){
				case ONE:
					commandName = "AutoPos" + "One" + "_" + colorAssignment;
					break;
				case TWO:
					commandName = "AutoPos" + "Two" + "_" + colorAssignment;
					break;
				case THREE:
					commandName = "AutoPos" + "Three" + "_" + colorAssignment;
					break;
			
			}
			
			setAutoCommand(commandName);
			
			if (autonomousCommand != null) {
				debug("autonomous command should start");
				autonomousCommand.start();
			}
			else {
				debug("autonomousCommand is unexpectedly null");
			}
			
		}


		private String getColorAssignment(){
			String matchColorAssignment = new String();
			// Insert code here to read color assignment from FMS
			matchColorAssignment = "LLL";
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
//.add(ClawSubsystem.motorName, new WPI_TalonSRX(2))
//.add(ClawSubsystem.motorName2, new WPI_TalonSRX(0))
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
