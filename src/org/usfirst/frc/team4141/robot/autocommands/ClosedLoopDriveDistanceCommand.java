package org.usfirst.frc.team4141.robot.autocommands;
//import edu.wpi.first.wpilibj.Encoder; Nope

import org.usfirst.frc.team4141.MDRobotBase.MDCommand;
import org.usfirst.frc.team4141.MDRobotBase.MDRobotBase;
import org.usfirst.frc.team4141.MDRobotBase.MDSubsystem;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.robot.subsystems.MDDriveSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class ClosedLoopDriveDistanceCommand extends MDCommand {
	
	//@SuppressWarnings("unlikely-arg-type")
	//MDDriveSubsystem driveSubsystems = (MDDriveSubsystem)robot.getSubsystems().get("driveSystem");
	//WPI_TalonSRX talon = (WPI_TalonSRX)driveSubsystems.getMotors().get(MDDriveSubsystem.MotorPosition.rearLeft.toString());
	
	private MDDriveSubsystem driveSubsystem;
	//Raw variables are in encoder units (ticks)
	private double m_targetDistanceFT; 
	private double m_targetDistanceRaw;
	private int m_speedFTPS; 
	private int m_speedRaw;
	private double m_wheelCircumferenceInches = (Math.PI)*(6); //wheel diameter is 6in
	private double m_nativeUnitsPerRotation = 4096;
	private WPI_TalonSRX talon;
	//1549;
	//1290.88841
	
	
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/*
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;
	
	/* choose so that Talon does not report sensor out of phase */
	public static boolean kSensorPhase = false;

	/* choose based on what direction you want to be positive,
		this does not affect motor invert. */
	public static boolean kMotorInvert = false;
	
	// ------------------------------------------------ //
	
	/**
	 * Constructor for the DriveDistanceCommand
	 * Within the constructor is a fail-safe to check that the Drive Subsystem
	 * is connected and ready to be used. If the Drive Subsystem is not connected 
	 * the Robot will not enable.
	 *  
	 * @param robot - the robot object
	 * @param name - name of this DriveDistanceCommand
	 * @param targetDistanceInFeet - Desired distance to travel (in feet)
	 * @param speed - The speed of the robot in feet per second (only go to the tenths place!)
	 * @param reverse - Set to false if going forward, but set to true of going backwards
	 */
	public ClosedLoopDriveDistanceCommand(MDRobotBase robot, String name, double targetDistanceInFeet, double speedInFTPS, boolean reverse) {
		super(robot, name);
		
		m_targetDistanceFT = targetDistanceInFeet;
	    m_targetDistanceRaw = (m_targetDistanceFT*12)/m_wheelCircumferenceInches*m_nativeUnitsPerRotation;
	    m_speedRaw = (int) ((m_speedFTPS/10)*(12/m_wheelCircumferenceInches)*(m_nativeUnitsPerRotation));
		kMotorInvert = reverse;
		
		// Make sure that the Drive Subsystem is active
		if(!getRobot().getSubsystems().containsKey("driveSystem")){
			log(Level.ERROR, "initialize()", "Drive subsystem not found");
			throw new IllegalArgumentException("Drive Subsystem not found");
		}
		driveSubsystem = (MDDriveSubsystem)getRobot().getSubsystems().get("driveSystem"); 
		requires(driveSubsystem);
		
		System.out.println("Closed loop drive distance command has been constructed");
		
		
	}

	// Initialize is called when the command first starts
	 
	protected void initialize() {
		System.out.println("Closed loop initialize");
		MDRobotBase robot = this.getRobot();
		driveSubsystem = (MDDriveSubsystem)robot.getSubsystems().get("driveSystem");		
		String motorName = MDDriveSubsystem.MotorPosition.rearLeft.toString();
		talon = (WPI_TalonSRX)driveSubsystem.getMotors().get(motorName);
		
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		talon.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

		/* set closed loop gains in slot0, typically kF stays zero. */
		talon.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		talon.config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		talon.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		talon.config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
		
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talon.configClosedloopRamp(2, 0); //Set ramp seconds from stop to full throttle
		talon.setInverted(kMotorInvert); //Invert the signal if needed
		talon.configMotionCruiseVelocity(m_speedRaw, 10);
		talon.configPeakOutputForward(1, 0);
		talon.configPeakOutputReverse(-1, 0);

		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = talon.getSensorCollection().getPulseWidthPosition();
		/* mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (kSensorPhase)
			absolutePosition *= -1;
		if (kMotorInvert)
			absolutePosition *= -1;
		/* set the quadrature (relative) sensor to match absolute */
		talon.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		// Set the target for closed loop
		talon.set(ControlMode.Position, m_targetDistanceRaw);
	
		System.out.println("Target Distance in Feet= " + m_targetDistanceFT + "\n Target Distance Raw= " + m_targetDistanceRaw);
		
	}
	protected boolean isFinished() {
		double kVoltThreshold = 1.0;
		int kErrorThreshold = 700;
		
		double motorVolts = Math.abs(talon.getMotorOutputVoltage());
		int motorError = Math.abs(talon.getClosedLoopError(0));

		if(motorVolts <= kVoltThreshold && motorError <= kErrorThreshold){
			System.out.println("Finished: Closed Loop Error = " + motorError + "; Motor output (volts) = " + motorVolts);
			
			return true;
		}
		System.out.println("Still Working: Closed Loop Error = " + motorError + "; Motor output (volts) = " + motorVolts);
		return false;
	}
	
	protected void execute() {
		talon.set(ControlMode.Position, m_targetDistanceRaw);
	}
	
	protected void end() {
		super.end();
		talon.set(0);
	}
	
}
	
	