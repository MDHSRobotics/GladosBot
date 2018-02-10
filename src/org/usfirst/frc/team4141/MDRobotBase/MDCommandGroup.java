package org.usfirst.frc.team4141.MDRobotBase;



import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;

import edu.wpi.first.wpilibj.command.CommandGroup;

public abstract class MDCommandGroup extends CommandGroup {
	private MDRobotBase robot;

	public MDCommandGroup(MDRobotBase robot,String name) {
		super(name);
		this.robot=robot;
	}

	public void log(String methodName, String message) {
		getRobot().log(this.getClass().getSimpleName()+"."+methodName+"()", message);
		
	}
	public void log(Level level, String methodName, String message) {
		getRobot().log(level,this.getClass().getSimpleName()+"."+methodName+"()", message);
	}

	public MDRobotBase getRobot() {
		return robot;
	}
	
	public String toString() {
		String objectString;
		objectString = "\n===========================================";
		objectString += "\nCommand Group class = " + this.getClass().getName();
		objectString += "\nCommand Group name = "  + this.getName();
		objectString += "\nSubsystem = "  + this.getSubsystem();
		objectString += "\nIs Canceled = "  + this.isCanceled();
		objectString += "\nIs Completed = "  + this.isCompleted();
		objectString += "\nIs Finished = "  + this.isFinished();
		objectString += "\nIs Interruptible = "  + this.isInterruptible();
		objectString += "\nIs Running = "  + this.isRunning();
		
		return objectString;
	}


}
