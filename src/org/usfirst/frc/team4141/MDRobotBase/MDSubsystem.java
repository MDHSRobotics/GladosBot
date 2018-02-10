package org.usfirst.frc.team4141.MDRobotBase;

import java.util.Hashtable;
import java.util.Set;

import org.usfirst.frc.team4141.MDRobotBase.config.ConfigPreferenceManager;
import org.usfirst.frc.team4141.MDRobotBase.config.ConfigSetting;
import org.usfirst.frc.team4141.MDRobotBase.eventmanager.LogNotification.Level;
import org.usfirst.frc.team4141.MDRobotBase.sensors.Sensor;
import org.usfirst.frc.team4141.MDRobotBase.sensors.SensorReading;

//import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Sendable;
//import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public abstract class MDSubsystem extends Subsystem {
//	protected String name;
	private MDRobotBase robot;
	private Hashtable<String,SpeedController> motors;
	private Hashtable<String,SolenoidBase> solenoids;
	private Hashtable<String,Sensor> sensors;
	private Hashtable<String,ConfigSetting> configSettings;	
	private boolean isConfigured = false;
	private boolean isCore = false;
	
	public boolean isCore(){ return isCore;}
	public void setCore(boolean isCore){this.isCore = isCore;}
	
	public MDSubsystem add(String name,SpeedController motor){
		if(isConfigured) return this;
		motors.put(name, motor);
		return this;
	}
	public Hashtable<String,SpeedController> getMotors() {
		return motors;
	}
	public MDSubsystem add(String name,SolenoidBase solenoid){
		if(isConfigured) return this;
		solenoids.put(name, solenoid);
		return this;
	}
	public Hashtable<String,SolenoidBase> getSolenoids() {
		return solenoids;
	}	
	public MDSubsystem add(String name,Sensor sensor) {
//		System.out.println("adding sensor "+name+" to "+getName());
		if(isConfigured) return this;
		sensors.put(name, sensor);
		sensor.setName(name);
		sensor.setSubsystem(this);
		return this;
	}
	public Hashtable<String,Sensor> getSensors() {
		return sensors;
	}	
	public MDSubsystem add(String name, ConfigSetting setting) {
		if(isConfigured) return this;
		setting.setName(name);
		setting.setSubsystem(this);
//		System.out.printf("%s set to %s\n",name,setting.getValue().toString());
		configSettings.put(name, setting);
		return this;
	}
	public Hashtable<String,ConfigSetting> getConfigSettings() {
		return configSettings;
	}	
	public MDSubsystem(MDRobotBase robot, String name) {
		super(name);
//		System.out.println("after calling super on MDSubsystem");
		this.robot=robot;
//		this.name = name;
		motors = new Hashtable<String,SpeedController>();
		solenoids = new Hashtable<String,SolenoidBase>();
		sensors = new Hashtable<String,Sensor>();
		configSettings = new Hashtable<String,ConfigSetting>();
		isConfigured = false;
	}
	//TODO Check if error
	public MDSubsystem configure(){
//		System.out.println("MDSubsystem.config #1");
//		System.out.println(this.toString());
//		System.out.println("^^^^^^^^^^^^^^^");
		if(this.motors!=null && this.motors.size()>0){
			Set<String> keys = motors.keySet();
			for(String key : keys){
				SpeedController speedController = motors.get(key);
				if(speedController instanceof Sendable){
//					setName(getName(), key);
					LiveWindow.add((Sendable) speedController);
//					LiveWindow.addActuator(getName(), key, (LiveWindowSendable)speedController);
					
				}
			}
		}
//		System.out.println("MDSubsystem.config #2");
//		System.out.println(this.toString());
//		System.out.println("^^^^^^^^^^^^^^^");
		
		if(this.solenoids!=null && this.solenoids.size()>0){
			Set<String> keys = solenoids.keySet();
			for(String key : keys){
				SolenoidBase item = solenoids.get(key);
				if(item instanceof Sendable){
//					setName(getName(), key);
					LiveWindow.add((Sendable) item);
				}
			}
		}
		if(this.sensors!=null && this.sensors.size()>0){
			Set<String> keys = sensors.keySet();
			for(String key : keys){
				Sensor item = sensors.get(key);
				if(item instanceof Sendable){
//					setName(getName(), key);
					LiveWindow.add((Sendable) item);
				}
				System.out.println("system "+getName()+" has sensor "+key+" observe: "+item.observe());
				if(item.observe()){
					robot.add(item);
					for(SensorReading reading : item.getReadings()){
						if(reading == null){
							debug("reading unexpectedly null in sensor "+item.getName());
						}
						else{
							debug("registering reading "+reading.getName()+" from "+item.getName());	
							if(reading.observe()){
								robot.add(reading);
							}
						}
						
					}
				}
			}
		}
//		System.out.println("MDSubsystem.config #3");
//		System.out.println(this.toString());
//		System.out.println("^^^^^^^^^^^^^^^");
		
		if(configSettings!=null && configSettings.size()>0){
			for(String settingName : configSettings.keySet()){
				ConfigPreferenceManager.register(configSettings.get(settingName));
			}
		}
		
		isConfigured = true;
		setUp();
		return this;
	}
//	public String getName() {
//		return name;
//	}
	public MDRobotBase getRobot() {
		return robot;
	}

	public void log(String methodName, String message) {
		getRobot().log(this.getClass().getName()+"."+methodName+"()", message);	
	}
	public void log(Level level, String methodName, String message) {
		getRobot().log(level,this.getClass().getName()+"."+methodName+"()", message);
	}
	
	public boolean isConfigured(){ return isConfigured;}
	
	
	protected abstract void setUp();
	public abstract void settingChangeListener(ConfigSetting setting);
	
	public ConfigSetting getSetting(String settingName) {
		if(configSettings!=null && configSettings.containsKey(settingName)){
			return configSettings.get(settingName);
		}
		return null;
	}
	public boolean hasSetting(String settingName) {
		if(configSettings!=null && configSettings.containsKey(settingName)){
			return true;
		}
		return false;
	}
	public void debug(String message) {
		getRobot().debug(message);		
	}
	public String toString() {
		String objectString;
		objectString = "\n===========================================";
		objectString += "\nSubsystem class = " + this.getClass().getName();
		objectString += "\nSubsystem name = "  + this.getName();
		objectString += "\nSubsystem for robot = " + this.robot.getName();
		objectString += "\nConfigured Flag = " + this.isConfigured;
		objectString += "\nCore Flag = " + this.isCore;
		
		if (motors != null) {
			// get set of motors keys
	        Set<String> setOfMotorKeys = motors.keySet();
	        
			objectString += "\nNumber of motors = " + setOfMotorKeys.size();
			int i = 1;
			
	        // Loop over keys
	        for (String key : setOfMotorKeys) {
	    		objectString += "\nMotor #" + i;
	    		// Print Motor
	    		objectString += motors.get(key).toString();
	    		++i;
	        }
	        objectString += "\nEnd of motors";
	        objectString += "\n-----------------";
		}
		else {
			objectString += "Warning: motors hashtable not defined!";
		}
		
		if (sensors != null) {
			// get set of sensors keys
	        Set<String> setOfSensorKeys = sensors.keySet();
	        
			objectString += "\nNumber of sensors = " + setOfSensorKeys.size();
			int i = 1;
			
	        // Loop over keys
	        for (String key : setOfSensorKeys) {
	    		objectString += "\nSensor #" + i;
	    		// Print Sensor Information
	    		Sensor sensor = sensors.get(key);
	    		objectString += "\n  Name = " + sensor.getName();
	    		objectString += "\n  Subsystem = " + sensor.getSubsystemObject().getName();
	    		
	    		++i;
	        }
	        objectString += "\nEnd of sensors";
	        objectString += "\n-----------------";
		}
		else {
			objectString += "Warning: sensors hashtable not defined!";
		}
		
		return objectString;
	}
}
