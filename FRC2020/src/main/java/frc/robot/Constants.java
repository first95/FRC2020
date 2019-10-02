package frc.robot;

public class Constants
	{
	
	    // How much the joystick must move before anything will happen
		public static double joystickDeadbandV = 0.07;
		public static double joystickDeadbandH = 0.05;
		
		// Properties of the robot design 
		public static final double ROBOT_WHEELBASE_WIDTH_INCHES = 23.0; // Distance between the centers of the wheels
		public static final double ROBOT_TOP_SPEED_LOW_GEAR_FPS = 7; 
		public static final double ROBOT_TOP_SPEED_HIGH_GEAR_FPS = 17; 
		
		// Used in closed-loop control
		public static final double ELEVATOR_ON_TARGET_THRESHOLD_INCHES = 1; // Elevator will call itself close enough at this point
		public static final double DRIVEPOD_ON_TARGET_THRESHOLD_INCHES = 1; // Each drivepod will call itself close enough at this point		
		public static final double CARGO_HANDLER_ON_TARGET_DEG = 2; // Cargo handler wrist will call itself close enough at this point
		public static final double HGL_ON_TARGET_DEG = 5; // HGL wrist will call itself close enough at this point

		// Speed Shifter Values
		public static final double SPEED_TO_SHIFT_UP = 5.5; // ft per sec
		public static final double SPEED_TO_SHIFT_DOWN = 5.0; // ft per sec

		// Used with Talons
		public static final int PID_IDX = 0; // The Talons support up to 2 PID loops, with indexes 0 and 1.  We only use 0.
		public static final int CAN_TIMEOUT_MS = 10; // The amount of time to wait for the CAN transaction to finish
		public static final int CAN_ORDINAL_SLOT0 = 0;
		
		// Indices for solenoids
		public static final int SHIFTER_SOLENOID_NUM  = 0;
		public static final int HS_OPEN_A = 7;
		public static final int HS_PUSH_A = 6;
		public static final int CLIMBER_SOL_REAR = 1;
		public static final int CLIMBER_SOL_FRONT = 5;
		public static final int BRAKES_SOL = 5;
		
		// Indices for sensors
        public static final int ELEVATOR_HOME_SWITCH_DIO_NUM = 0;
        // Sensors looking at the floor for the white tape lines.  In order, left to right.
		public static final int[] LINE_SENSOR_DIO_NUM = {5,7,8,6,4}; // {2, 3, 4, 5, 6, 7, 8};
		public static final int[] FORWARD_FACING_SENSOR_DIO_NUM = {2,3};

		// Indices for Talons
		// Drive base
		public static final int LEFT_LEAD = 10;
		public static final int LEFT_F1 = 11;
		public static final int LEFT_F2 = 12;
		public static final int RIGHT_LEAD = 20;
		public static final int RIGHT_F1 = 21;
		public static final int RIGHT_F2 = 22;
		
		// Elevator
		public static final int ELEV_FOLLOWER_DRIVER = 31;
		// public static final int ELEV_FOLLOWER_DRIVER = 15;
		public static final int ELEV_LEADER_DRIVER = 35;
		// public static final int ELEV_LEADER_DRIVER = 25;

		// Hatch Ground Loader
		public static final int HGL_INTAKE = 14;
		public static final int HGL_WRIST = 24;
		
		// Cargo handler
		public static final int CARGO_HANDLER_WRIST = 29;
		public static final int CARGO_HANDLER_INTAKE = 30;
		
		// Climber
		public static final int CLIMBER_DRIVER = 33;

		// Sensors attached via CAN
		public static final int PIGEON_NUM = 30;		

		// Current limiting parameters
		public static final int DRIVEPOD_MAX_CURRENT_CONTINUAL_AMPS = 10;
		public static final int DRIVEPOD_MAX_CURRENT_PEAK_AMPS = 5;
		public static final int DRIVEPOD_MAX_CURRENT_PEAK_DURATION_MS = 100;	
		public static final double CARGO_HANDLER_INTAKE_CUR_SPIKE_AMPS = 10;
		public static final double HGL_INTAKE_CUR_SPIKE_AMPS = 12;	
		public static final double HGL_WRIST_CUR_LIMIT_AMPS = 8;
	}