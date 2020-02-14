package frc.robot;

public class Constants
	{
	
	    // How much the joystick must move before anything will happen
		public static double joystickDeadbandV = 0.07;
		public static double joystickDeadbandH = 0.05;
		
		// PCM Device ID
		public static final int PCM_NUM = 0;

		// Ground Pick-up IDs
		public static final int GROUND_PICK_UP_TALON_ID = 1;
		public static final int GROUND_PICK_UP_SOLENOID_ID = 1;

		// Singulator IDs
		public static final int INNER_SINGULATOR_TALON_ID = 3;

		// Singulator intake IDs
		public static final int SINGULATOR_INTAKE_TALON_ID = 2;

		// Shooter IDs
		public static final int LEADER_SHOOT = 12;
		public static final int FOLLOWER_SHOOT = 13;

		// Speed Shifter Values
		public static final double SPEED_TO_SHIFT_UP = 5.0; // ft per sec
		public static final double SPEED_TO_SHIFT_DOWN = 3.0; // ft per sec

		// Used with Talons
		public static final int PID_IDX = 0; // The Talons support up to 2 PID loops, with indexes 0 and 1.  We only use 0.
		public static final int CAN_TIMEOUT_MS = 10; // The amount of time to wait for the CAN transaction to finish
		public static final int CAN_ORDINAL_SLOT0 = 0;
		
		// Indices for solenoids
		public static final int SHIFTER_SOLENOID_NUM  = 0;
		
		// Indices for SparkMaxes
		// Drive base
		public static final int LEFT_LEAD = 7;
		public static final int LEFT_F = 6;
		// public static final int LEFT_F2 = 12;
		public static final int RIGHT_LEAD = 9;
		public static final int RIGHT_F = 8;
		// public static final int RIGHT_F2 = 22;
		public static final int INDEXER_BELT_MOTOR_ID = 10;

		
		// Drivebase constants
		public static final double DRIVE_WHEEL_DIAMETER_IN = 6;
		public static final double LOW_GEAR_RATIO = 20.83;
		public static final double HIGH_GEAR_RATIO = 9.17;

		// Digital I/O pin names
		public static final int SINGULATOR_SENSOR = 0;
		public static final int INDEXER_ENTRANCE_SENSOR = 1;
		public static final int INDEXER_POWERCELL_LOADED_SENSOR = 2;
		public static final int SHOOTER_LOADED_SENSOR = 3;
	}
