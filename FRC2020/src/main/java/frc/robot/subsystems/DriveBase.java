
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.components.DrivePod;

/**
 * The DriveBase subsystem incorporates the sensors and actuators attached to
 * the robot's chassis. These include two 3-motor drive pods.
 */
public class DriveBase extends Subsystem {
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
	private Solenoid shifter;

	private double leftSpeed;
	private double rightSpeed;
		
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
	
	public DriveBase() {
		super();

		// Note that one pod must be inverted, since the gearbox assemblies are rotationally symmetrical
		leftPod = new DrivePod("Left", Constants.LEFT_LEAD, Constants.LEFT_F1, Constants.LEFT_F2, false);
        rightPod = new DrivePod("Right", Constants.RIGHT_LEAD, Constants.RIGHT_F1, Constants.RIGHT_F2, true);
		shifter = new Solenoid(Constants.SHIFTER_SOLENOID_NUM);
		
	}

	/**
	 * When no other command is running let the operator drive around using the PS3
	 * joystick.
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManuallyControlDrivebase());
	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	public void log() {
		leftPod.log();
		rightPod.log();

		SmartDashboard.putNumber("leftDriveEncoder Value:", leftPod.getQuadEncPos());
		SmartDashboard.putNumber("rightDriveEncoder Value:", rightPod.getQuadEncPos());
		SmartDashboard.putBoolean("In High Gear", getGear());
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
		double y = Robot.oi.getForwardAxis();
		double x = Robot.oi.getTurnAxis();

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
	
	/**
	 * Apply the appropriate gear decided by the auto/manual shifting logic
	 * @param isHighGear true for high gear, false for low gear
	 */
	private void setGear(boolean isHighGear) {
        //System.out.println("Shifting to " + (isHighGear? "high":"low") + " gear");
        if(shifter != null) {
            shifter.set(isHighGear);
        }
	}
	
	/**
	 * 
	 * @return true for high gear, false for low gear
	 */
	public boolean getGear() {
		// True in high gear
        // False in low gear
        if(shifter != null) {
            return shifter.get();
        } else {
            return false;
        }
	}

	/**
	 * Apply the correct gear for the speed of the drivebase right now.
	 * Should only be called when the gear shift mode permits auto shifting.
	 */
	private void autoShift() {
		leftSpeed = Math.abs(Robot.drivebase.getLeftSpeed());
		rightSpeed = Math.abs(Robot.drivebase.getRightSpeed());

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
					if (allowDeshift) {
						shiftTimer.reset();
						shiftTimer.start();
						allowShift = false;
						setGear(true);
						//System.out.println(elevatorIsTooHighToShift + " case 2");
					}
			}
		} else if (shiftTimer.get() > 1.0) {
			allowShift = true;
			shiftTimer.stop();
			shiftTimer.reset();
			allowDeshift = false;
			hasAlreadyShifted = true;
		}
	}
	

	/**
	 * Ask if an autonomous move has asked the robot to
	 * remain in a particular gear
	 */
	public GearShiftMode getShiftMode() {
		return gearShiftMode;	
	}
	
	/**
	 * To be called by an auto mode, to keep the drivebase in a certain
	 * gear or let it run free.
	 * 
	 * Can be overridden by the driver via the OI.
	 * @param shiftMode
	 */
	public void setShiftMode(GearShiftMode shiftMode) {
		gearShiftMode = shiftMode;
	}

	public void visit() {
		handleGear();
	}
	
	/**
	 * Enact whichever shift mode is appropriate
	 */
	private void handleGear() {
		// Driver commanded override?
		if(Robot.oi.getHighGear()) {
			setGear(true);
		} else if (Robot.oi.getLowGear()) {
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
    

}
