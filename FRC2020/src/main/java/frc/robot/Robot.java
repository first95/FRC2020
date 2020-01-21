package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHabLevelTwo;
import frc.robot.subsystems.HatchGroundLoader;
import frc.robot.subsystems.HatchScorer;
import frc.robot.subsystems.PathFinderSystem;
import frc.robot.commands.drivebase.PathFinderCommand;
import frc.robot.subsystems.VisionCoprocessor;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import frc.robot.SelectedStrategy;
import frc.robot.components.AdjustedTalon;
import frc.robot.components.FakeTalon;
import frc.robot.components.StartPosition;
import frc.robot.oi.XBox360Controller;
import frc.robot.subsystems.Brakes;
import frc.robot.components.DrivePod;
//import frc.robot.subsystems.DriveBase.GearShiftMode;
import frc.robot.commands.compound.ForwardTenFeet;
//import jaci.pathfinder.Pathfinder;
import frc.robot.commands.drivebase.PathFinderCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {

	private IMotorControllerEnhanced leader;

	// private StartPosition robotStartSide; // The location where the robot began
	// private String gameData;
	//Command autonomousCommand;

	// Components of the robot
	// public static DriveBase drivebase;
	//public static Elevator elevator;
	//public static HatchScorer hScorer;
	//public static HatchGroundLoader hGroundLoader;
	//public static CargoHandler cargoHandler;
	//public static Climber climber;
	//public static ClimberHabLevelTwo climber2;
	//public static Compressor compressor;
	// public static OI oi;
	//public static VisionCoprocessor vision;
	// public static Brakes brakes;
	// public static PathFinderSystem pathfinder;
	// public static StartPosition startPosition;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		// Initialize all subsystems
		// drivebase = new DriveBase(true);
		//elevator = new Elevator(true);
		// hScorer = new HatchScorer(true);
		// hGroundLoader = new HatchGroundLoader(true);
		// cargoHandler = new CargoHandler(true);
		// climber = new Climber(false);
		// climber2 = new ClimberHabLevelTwo(false);
		// compressor = new Compressor();
		//vision = new VisionCoprocessor();
		// brakes = new Brakes(true);
		//pathfinder = new PathFinderSystem(true, false, 0, 1);
        // oi = new OI();

		// Show what command your subsystem is running on the SmartDashboard
		// SmartDashboard.putData(drivebase);
		//SmartDashboard.putData(elevator);
		// SmartDashboard.putData(hGroundLoader);
		// SmartDashboard.putData(climber);

		// Disable brakes on talons to make it
		// easier to push
		// drivebase.brake(false);
		//elevator.brake(false);

	}


	//private int Encoder_Pos = 0;
  	private int Left_To_Right_Offset_Inches;
  	private double Top_Speed;
  	public int Right_Encoder_Pos = 0;
  	public int Left_Encoder_Pos = 0;
  	private double wheel_diameter = Constants.WHEEL_DIAMETER*0.0254;
  	public static double outputLeft, outputRight;
  	public static int whatPath;
  	public static int spin = 0;

  	// All of these are the possible path values
  	public static int ForwardTenFeet = 1;

  	private static boolean LeftOrRight;
  	private static boolean WhatGearAreWeIn;

  	public static EncoderFollower leftEncFollower;
  	public static EncoderFollower rightEncFollower;

  	public static Waypoint[] points;

	@Override
	public void autonomousInit()
	{
		if (whatPath == ForwardTenFeet)
    	{
      		points = new Waypoint[] {
        		new Waypoint(0, 0, 0),
        		new Waypoint(10, 0, 0)
      		};
    	}
   

	    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    	Trajectory trajectory = Pathfinder.generate(points, config);

    	// Wheelbase Width = 0.5m
    	TankModifier modifier = new TankModifier(trajectory).modify(0.5);

    	// Do something with the new Trajectories...
    	//Trajectory left = modifier.getLeftTrajectory();
    	//Trajectory right = modifier.getRightTrajectory();

	    leftEncFollower = new EncoderFollower(modifier.getLeftTrajectory());
    	rightEncFollower = new EncoderFollower(modifier.getRightTrajectory());

    	// Determine whether the encoder position is the left or right encoder position.
    	if(LeftOrRight)
    	{
      		Left_Encoder_Pos = (int) (Math.round(leftPod.getQuadEncPos() + Left_To_Right_Offset_Inches));
      		Right_Encoder_Pos = (int) (Math.round(rightPod.getQuadEncPos()));
    	}
    	else
    	{
			Left_Encoder_Pos = (int) (Math.round(leftPod.getQuadEncPos()));
			Right_Encoder_Pos = (int) (Math.round(rightPod.getQuadEncPos() + Left_To_Right_Offset_Inches));
    	}

    	leftEncFollower.configureEncoder(0, 1000, wheel_diameter);
    	rightEncFollower.configureEncoder(0, 1000, wheel_diameter);

    	// Determine what gear we are in, then make the top speed here equal to the top speed for the gear.
    	if(WhatGearAreWeIn)
    	{
    	  Top_Speed = Constants.ROBOT_TOP_SPEED_HIGH_GEAR_FPS;
    	}
    	else
    	{
    	  Top_Speed = Constants.ROBOT_TOP_SPEED_LOW_GEAR_FPS;
    	}

    	leftEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Top_Speed, 0);
    	rightEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Top_Speed, 0);
		// System.out.println("made it");
		// int maxTime_sec = 8;
		// double startTime_sec = Timer.getFPGATimestamp();
		// double elapTime_sec = Timer.getFPGATimestamp() - startTime_sec;
		// gameData = "";
		// // while ((gameData.length() < 3) & (elapTime_sec < maxTime_sec)) {
		// // 	gameData = DriverStation.getInstance().getGameSpecificMessage();
		// // 	elapTime_sec = Timer.getFPGATimestamp() - startTime_sec;
		// // }
		// if (gameData == "") {
			
		// 	gameData = "UUU";
		// } else {
		// 	System.out.println("Time to get game data was "+elapTime_sec+" seconds.");
		// }
		// //System.out.println("Plate assignments are " + gameData);

		// robotStartSide = oi.getRobotStartPosition();
		// System.out.println("Robot start side: " + robotStartSide);
		// System.out.println("Strategy #"  + getWhichStratIsSelected() + " has been selected.");
		
		// autonomousCommand = new PathFinderCommand(true, false, 20, 1);
		// autonomousCommand.start();
    }

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// robotPeriodic();
		outputLeft = leftEncFollower.calculate(Left_Encoder_Pos);
		outputRight = rightEncFollower.calculate(Right_Encoder_Pos);
		tank(outputLeft, outputRight);
	}




	private static Joystick driverController = new Joystick(0);
	private static Joystick weaponsController = new Joystick(1);

	public static final int BRAKES_DEPLOY = XBox360Controller.Button.X.Number();

	// Axes on drive controller
	public static final int DRIVE_FORWARD_AXIS = XBox360Controller.Axis.LEFT_STICK_Y.Number();
	public static final int DRIVE_TURN_AXIS = XBox360Controller.Axis.RIGHT_STICK_X.Number();

	public enum Controller {
		DRIVER,
		WEAPONS, // Weapons operator
	}

	 /**
     * Get the forward travel rate commanded by the driver
     * @return -1 for full speed backward, +1 for full speed forward
     */
	public static double getForwardAxis() {
		return driverController.getRawAxis(DRIVE_FORWARD_AXIS);
	}

    /**
     * Get the turn rate commanded by the driver
     * @return -1 for full turn leftward (CCW when looking down at the robot), +1 for full turn rightward (CW when looking down at the robot), 0 for no turn
     */
	public static double getTurnAxis() {
		return driverController.getRawAxis(DRIVE_TURN_AXIS);
	}

	/**
	 * Ask if the driver wants the robot to be in high gear
	 * @return
	 */
	public static boolean getHighGear() {
		return false; //return driverController.getRawButton(BUTTON_FORCE_HIGH_GEAR);
	}

	/**
	 * Ask if the driver wants the robot to be in low gear
	 * @return
	 */
	public static boolean getLowGear() {
		return false; //driverController.getRawButton(BUTTON_FORCE_LOW_GEAR);
	}





	// in inches
	private final double DISTANCE_FROM_OUTER_TO_INNER_WHEEL = 13.5;
	// in inches
	private final double DISTANCE_FROM_INNER_TO_INNER_WHEEL = 23.25;
	private final double RADIUS_OF_AVERAGED_WHEEL_CIRCLE = Math.sqrt(Math.pow((DISTANCE_FROM_INNER_TO_INNER_WHEEL/2), 2) + Math.pow(DISTANCE_FROM_OUTER_TO_INNER_WHEEL, 2));
	// The speed at which we want the center of the robot to travel
	// private final double SWEEPER_TURN_SPEED_INCHES_PER_SECOND = 3.5*12.0;
	private final double TURN_SPEED_INCHES_PER_SECOND = 36;
	// This is tied to speed, if you change the speed of the turn also change this value
	
	private final double SWEEPER_TURN_SPEED_INCHES_PER_SECOND = 24;
	private DrivePod leftPod, rightPod;
	private static Solenoid shifter;

	private double leftSpeed;
	private double rightSpeed;
	
	// private Pigeon imu;
	// private DigitalInput[] lineSensor;
    // private DigitalInput[] forwardFacingSensor;
	
	// Mode for the gearshift, as set by the auto moves
	public enum GearShiftMode {
		LOCK_HIGH_GEAR ,
		LOCK_LOW_GEAR, 
		AUTOSHIFT  ,
	}
	private GearShiftMode gearShiftMode = GearShiftMode.AUTOSHIFT;

	private Timer shiftTimer = new Timer();
	private boolean allowShift = true;
	private boolean allowDeshift = true;
	private boolean hasAlreadyShifted = false;
	
	public void driveBase(boolean realHardware) {
		// super();

		// Note that one pod must be inverted, since the gearbox assemblies are rotationally symmetrical
		leftPod = new DrivePod("Left", Constants.LEFT_LEAD, Constants.LEFT_F1, Constants.LEFT_F2, false, realHardware);
        rightPod = new DrivePod("Right", Constants.RIGHT_LEAD, Constants.RIGHT_F1, Constants.RIGHT_F2, true, realHardware);
        if(realHardware) {
            shifter = new Solenoid(Constants.SHIFTER_SOLENOID_NUM);
        } else {
            shifter = null;
        }
		
        // imu = new PigeonWrapper(Constants.PIGEON_NUM);

        // Initialize sensors
        // lineSensor = new DigitalInput[Constants.LINE_SENSOR_DIO_NUM.length];
        // int i = 0;
        // for (int dioNum : Constants.LINE_SENSOR_DIO_NUM) {
        //     lineSensor[i++] = new DigitalInput(dioNum);
        // }
        // forwardFacingSensor = new DigitalInput[Constants.FORWARD_FACING_SENSOR_DIO_NUM.length];
        // i = 0;
        // for (int dioNum : Constants.FORWARD_FACING_SENSOR_DIO_NUM) {
        //     forwardFacingSensor[i++] = new DigitalInput(dioNum);
        // }
	}

	/**
	 * When no other command is running let the operator drive around using the PS3
	 * joystick.
	 */
	// @Override
	// public void initDefaultCommand() {
	// 	setDefaultCommand(new PathFinderCommand(true, false, 20, 1));
	// }

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	public void log() {
		leftPod.log();
		rightPod.log();

		SmartDashboard.putNumber("leftDriveEncoder Value:", leftPod.getQuadEncPos());
		SmartDashboard.putNumber("rightDriveEncoder Value:", rightPod.getQuadEncPos());
		SmartDashboard.putNumber("leftDriveCurrent:", leftPod.getLeadCurrent());
		SmartDashboard.putNumber("RightDriveCurrent:", rightPod.getLeadCurrent());
		// SmartDashboard.putNumber("IMU Yaw",   imu.getYawPitchRoll()[0]);
		// SmartDashboard.putNumber("IMU Pitch", imu.getYawPitchRoll()[1]);
		// SmartDashboard.putNumber("IMU Roll",  imu.getYawPitchRoll()[2]);
		// SmartDashboard.putNumber("IMU Fused heading", imu.getFusedHeading());
		SmartDashboard.putBoolean("In High Gear", getGear());
		// SmartDashboard.putBoolean("Sensor 0 Tripped", forwardFacingSensor[0].get());
		// SmartDashboard.putBoolean("Sensor 1 Tripped", forwardFacingSensor[1].get());
        // int i = 0;
        // for (DigitalInput ls : lineSensor) {
        //     SmartDashboard.putBoolean("Line Sensor " + i, !ls.get());
        //     i++;
        // }
	}

	/**
	 * @return The robots heading in degrees.
	 */
	public double getHeading() {
		// return gyro.getAngle();
		return 0;
	}

	public boolean onTarget() {
		return leftPod.isOnTarget() && rightPod.isOnTarget();
	}

	/**
	 * Command that the robot should travel a specific distance along the carpet.
	 * Call this once to command distance - do not call repeatedly, as this will
	 * reset the distance remaining.
	 * 
	 * @param inchesToTravel
	 *            - number of inches forward to travel
	 */
	public void travelStraight(double inchesToTravel) {
		// Max speed back and forward, always make this number positve when setting it.
		leftPod.setMaxSpeed(0.9);
		rightPod.setMaxSpeed(0.9);

		leftPod.setCLPosition(inchesToTravel);
		rightPod.setCLPosition(inchesToTravel);
	}

	/**
	 * Command that the robot should travel a specific distance along the carpet.
	 * Call this once to command distance - do not call repeatedly, as this will
	 * reset the distance remaining.
	 * 
	 * @param inchesToTravel
	 *            - number of inches forward to travel
	 * @param inchesPerSecond
	 *            - speed at which to travel
	 */
	public void travelStraight(double inchesPerSecond, double inchesToTravel) {
		leftPod.driveForDistanceAtSpeed(inchesPerSecond, inchesToTravel);
		rightPod.driveForDistanceAtSpeed(inchesPerSecond, inchesToTravel);
	}

	// Talon Brake system
	public void brake(boolean isEnabled) {
		leftPod.enableBrakeMode(isEnabled);
		rightPod.enableBrakeMode(isEnabled);
	}

	public void pivotDegreesClockwise(double inchesPerSecond, double degreesToPivotCw) {
		double leftDistanceInches = (2 * RADIUS_OF_AVERAGED_WHEEL_CIRCLE * Math.PI) * (degreesToPivotCw/360);
		double rightDistanceInches = leftDistanceInches;
		double turnSign = (degreesToPivotCw > 0)? 1.0 : -1.0;
		leftPod.driveForDistanceAtSpeed( turnSign * inchesPerSecond, -leftDistanceInches);
		rightPod.driveForDistanceAtSpeed(turnSign * inchesPerSecond, rightDistanceInches);		
	}
	
	// Do not use this for turning! Use setPivotRate
	public void pivotDegreesClockwise(double degreesToPivotCw) {

		double leftDistanceInches = (2 * RADIUS_OF_AVERAGED_WHEEL_CIRCLE * Math.PI) * ((degreesToPivotCw)/360);
		double rightDistanceInches = leftDistanceInches;
		//leftDistanceInches *= PIVOT_FUDGE_FACTOR;
		//rightDistanceInches *= PIVOT_FUDGE_FACTOR;
		double turnSign = (degreesToPivotCw > 0)? 1.0 : -1.0;
		leftPod.driveForDistanceAtSpeed( turnSign * TURN_SPEED_INCHES_PER_SECOND, -leftDistanceInches);
		rightPod.driveForDistanceAtSpeed(turnSign * TURN_SPEED_INCHES_PER_SECOND, rightDistanceInches);
	}

	public void setPivotRate(double inchesPerSecond) {
		leftPod.setCLSpeed(inchesPerSecond, true);
		rightPod.setCLSpeed(inchesPerSecond, true);
	}
	
	/**
	 * Cause the robot's center to sweep out an arc with given radius and angle. A
	 * positive clockwise angle is forward and to the right, a negative clockwise
	 * angle is forward and to the left.
	 * 
	 * This does not take into account the drivebase's tendency toward straight
	 * turns.
	 * 
	 * @param degreesToTurnCw
	 * @param turnRadiusInches
	 */
	public void travelSweepingTurnForward(double degreesToTurnCw, double turnRadiusInches) {
		double leftDistanceInches;
		double rightDistanceInches;

		double fractionOfAFullCircumference = Math.abs(degreesToTurnCw / 360.0);
		double sweepTimeS = (fractionOfAFullCircumference * turnRadiusInches * 2.0 * Math.PI)
				/ SWEEPER_TURN_SPEED_INCHES_PER_SECOND;
		if (degreesToTurnCw > 0) {
			// Forward and to the right - CW
			leftDistanceInches = fractionOfAFullCircumference * Math.PI * 2.0
					* (turnRadiusInches + Constants.ROBOT_WHEELBASE_WIDTH_INCHES / 2.0);
			rightDistanceInches = fractionOfAFullCircumference * Math.PI * 2.0
					* (turnRadiusInches - Constants.ROBOT_WHEELBASE_WIDTH_INCHES / 2.0);
		} else {
			// Forward and to the left - CCW
			leftDistanceInches = fractionOfAFullCircumference * Math.PI * 2.0
					* (turnRadiusInches - Constants.ROBOT_WHEELBASE_WIDTH_INCHES / 2.0);
			rightDistanceInches = fractionOfAFullCircumference * Math.PI * 2.0
					* (turnRadiusInches + Constants.ROBOT_WHEELBASE_WIDTH_INCHES / 2.0);
		}
		double leftSpeedInchesPerSecond = leftDistanceInches / sweepTimeS;
		double rightSpeedInchesPerSecond = rightDistanceInches / sweepTimeS;

		leftPod.driveForDistanceAtSpeed(leftSpeedInchesPerSecond, leftDistanceInches);
		rightPod.driveForDistanceAtSpeed(rightSpeedInchesPerSecond, rightDistanceInches);
	}

	/** 
	 * Stop moving
	 */
	public void stop() {

	}

	/**
	 * Drive at the commanded throttle values
	 * @param leftThrottle between -1 and +1
	 * @param rightThrottle between -1 and +1
	 */
	public void tank(double leftThrottle, double rightThrottle) {
		leftPod.setThrottle(leftThrottle);
		rightPod.setThrottle(rightThrottle);
	}

	/**
	 * Drive with the given forward and turn values
	 * @param forward between -1 and +1
	 * @param spin between -1 and +1, where -1 is full leftward (CCW when viewed from above)
	 */
	public void arcade(double forward, double spin) {
		tank(forward - spin, forward + spin);
	}
	/**
	 * Drive with the forward and turn values from the joysticks
	 */
	public void arcade() {
		setMaxSpeed(1);
		double y = Robot.getForwardAxis();
		double x = Robot.getTurnAxis();

		/* "Exponential" drive, where the movements are more sensitive during
		 * slow movement, permitting easier fine control
		 */
		x = Math.pow(x, 3);
		y = Math.pow(y, 3);
		arcade(y, x);
	}
	
	public void setMaxSpeed(double maxSpeed) {
		leftPod.setMaxSpeed(maxSpeed);
		rightPod.setMaxSpeed(maxSpeed);
	}

	public double getLeftSpeed() {
		return leftPod.getEncoderVelocityFeetPerSecond();
	}

	public double getRightSpeed() {
		return rightPod.getEncoderVelocityFeetPerSecond();
	}

	public double getLeftEncoderPos() {

		return leftPod.getQuadEncPos();
	}

	public double getRightEncoderPos() {

		return rightPod.getQuadEncPos();
	}
	
	public double getRobotHeadingDegrees() {
		// return imu.getYawPitchRoll()[0];
		return 0;
	}

	private void setGear(boolean isHighGear) {
        //System.out.println("Shifting to " + (isHighGear? "high":"low") + " gear");
        if(shifter != null) {
            shifter.set(isHighGear);
        }
	}
	
	public static boolean getGear() {
		// True in high gear
        // False in low gear
        if(shifter != null) {
            return shifter.get();
        } else {
            return false;
        }
	}

	private void autoShift() {
		leftSpeed = Math.abs(getLeftSpeed());
		rightSpeed = Math.abs(getRightSpeed());

		//double elevatorHeight = Robot.elevator.getElevatorHeightFeet();
		//final double ELEVATOR_HEIGHT_SPEED_LIMIT_FT = ((Elevator.ElevatorHoldPoint.HATCH_COVER_MID.heightInches) + 2)/12;
		//System.out.println(elevatorHeight + " is how high the elevator is");
		//boolean elevatorIsTooHighToShift;
		//if (elevatorHeight <= ELEVATOR_HEIGHT_SPEED_LIMIT_FT) {
			//elevatorIsTooHighToShift = false;
		//}
		//else {
			//elevatorIsTooHighToShift = true;
		//}

		// Autoshift framework based off speed
		if (allowShift) {
			if (((leftSpeed < Constants.SPEED_TO_SHIFT_DOWN) && (rightSpeed < Constants.SPEED_TO_SHIFT_DOWN))) {
				setGear(false);
				//System.out.println(elevatorIsTooHighToShift + " case 1");

				if (hasAlreadyShifted) {
					allowDeshift = true;
					hasAlreadyShifted = false;
				}

			} else if (((leftSpeed > Constants.SPEED_TO_SHIFT_UP)) || ((rightSpeed > Constants.SPEED_TO_SHIFT_UP))) {
				//if (elevatorIsTooHighToShift == false) {
					if (allowDeshift) {
						shiftTimer.reset();
						shiftTimer.start();
						allowShift = false;
						setGear(true);
						//System.out.println(elevatorIsTooHighToShift + " case 2");
					}
				//}
				
			}
		} else if (shiftTimer.get() > 1.0) {
			allowShift = true;
			shiftTimer.stop();
			shiftTimer.reset();
			allowDeshift = false;
			hasAlreadyShifted = true;
			//System.out.println(elevatorIsTooHighToShift + " case 3");
		}

		// System.out.println("rightSpeed: " + rightSpeed + ", allowShift: " + allowShift);
		// System.out.println("leftSpeed: " + leftSpeed + ", allowShift: " + allowShift);
		// SmartDashboard.putBoolean("Allow Shift:", allowShift);
		// SmartDashboard.putBoolean("Allow Deshift:", allowDeshift);
		// SmartDashboard.putBoolean("Has Already Shifted:", hasAlreadyShifted);
	}
	

	/**
	 * Ask if an autonomous move has asked the robot to
	 * remain in a particular gear
	 * @return 0 for "choose gear automatically", -1 for low gear, 1 for high gear.
	 */
	public GearShiftMode getShiftMode() {
		return gearShiftMode;	
	}
	
	public void setShiftMode(GearShiftMode shiftMode) {
		gearShiftMode = shiftMode;
	}

	public void visit() {
		handleGear();
	}
	
	// If true it locks into high gear, if false locks into low gear
	private void handleGear() {
		// Driver commanded override?
		if(Robot.getHighGear()) {
			setGear(true);
		} else if (Robot.getLowGear()) {
			setGear(false);
		} else {
			// No override from driver.  Auto move commanded override?
			switch(gearShiftMode) {
				case LOCK_HIGH_GEAR: setGear(true); break;
				case LOCK_LOW_GEAR: setGear(false); break;
				// No override commanded; handle automatic gear shifting.
				case AUTOSHIFT: autoShift(); break;
			}
		}
	}
	
	public void pullPidConstantsFromSmartDash() {
		//leftPod.pullPidConstantsFromSmartDash();
		//rightPod.pullPidConstantsFromSmartDash();
    }
    
    /**
     * @return the count of line sensors
     */
    // public int getLineSensorCount() {
    //     return lineSensor.length;
    // }

    // /**
    //  * 
    //  * @return The index of the center sensor
    //  */
    // public int getCenterSensorIndex() {
    //     return  (int)Math.floor(getLineSensorCount() / 2.0);
    // }

    // /**
    //  * Query a line sensor.
    //  * @param i sensor index.  0 is the leftmost, getLineSensorCount()-1 is the rightmost.
    //  * @return true if sensor i sees the line.
    //  */
    // public boolean doesSensorSeeLine(int i) {
    //     return !lineSensor[i].get();
    // }

    // /**
    //  * Check if the line is detected at all
    //  * @return true if any sensor sees the line
    //  */
    // public boolean doesAnySensorSeeTheLine() {
    //     for (int i = 0; i < getLineSensorCount(); ++i) {
    //         if(doesSensorSeeLine(i)) {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    // /**
    //  * @return true if all forward-facing sensors indicate the presence of a wall
    //  */
    // public boolean doAllForwardSensorsSeeWall() {
	// 	for (int dio = 0; dio < forwardFacingSensor.length; dio++)
	// 	{
	// 		if(dio == 0)
	// 		{
	// 			if(forwardFacingSensor[dio].get())
	// 			{
	// 				return false;
	// 			}
	// 		}
	// 		else
	// 		{
	// 			if(!forwardFacingSensor[dio].get())
	// 			{
	// 				return false;
	// 			}
	// 		}
	// 	}
    //     return true;
    // }
	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	// public void disabledInit() {
	// 	drivebase.brake(false);
	// 	//elevator.brake(false);
	// }

	// public void disabledPeriodic() {
	// 	robotPeriodic();
	// }

	// @Override
	// public void robotPeriodic() {
	// 	// Scheduler.getInstance().run(); // Runs all active commands
	// 	// elevator.checkAndApplyHomingSwitch();
	// 	// drivebase.pullPidConstantsFromSmartDash();
	// 	// oi.visit();
	// 	// drivebase.visit();

	// 	// // Depending if you want all output or just limited
	// 	// // use either debugLog() or just log()
	// 	// // debugLog();
	// 	// log();
	// }

	// @Override
	// public void teleopInit() {

	// 	// Unlock the auto shifter
	// 	drivebase.setShiftMode(GearShiftMode.AUTOSHIFT);

	// 	drivebase.brake(true);
	// 	//elevator.brake(true);
	// }

	// /**
	//  * This function is called periodically during operator control
	//  */
	// @Override
	// public void teleopPeriodic() {
	// 	robotPeriodic();
	// }

	/**
	 * This function is called periodically during test mode
	 */
	// @Override
	// public void testPeriodic() {

	// }

	// /**
	//  * The log method puts interesting information to the SmartDashboard.
	//  */
	// private void log() {
	// 	debugLog();
	// }

	// private void debugLog() {
	// 	drivebase.log();
	// 	//brakes.log();
	// 	//elevator.log();
	// 	//cargoHandler.log();
	// 	//hGroundLoader.log();
	// 	// oi.log();
	// }

	// public StartPosition getRobotStartSide() {
	// 	return robotStartSide;
	// }

	// private StartPosition sideFromChar(char side)
	// {
	// 	if(side == 'A') {
	// 		return StartPosition.HAB_1_LEFT;
	// 	} else if(side == 'B') {
	// 		return StartPosition.HAB_1_CENTER;
	// 	} else if(side == 'C') {
	// 		return StartPosition.HAB_1_RIGHT;
	// 	} else if(side == 'D') {
	// 		return StartPosition.HAB_2_LEFT;
	// 	} else if(side == 'E') {
	// 		return StartPosition.HAB_2_RIGHT;
	// 	} else if(side == 'F') {
	// 		return StartPosition.HAB_3;
	// 	} else {
	// 		return StartPosition.UNKNOWN;
	// 	}
	// }

	// public StartPosition getWhichStratIsSelected()
	// {
	// 	return sideFromChar(gameData.charAt(0));
	// }
}
