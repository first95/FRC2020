
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.components.DrivePodSpark;

/**
 * The DriveBase subsystem incorporates the sensors and actuators attached to
 * the robot's chassis. These include two 3-motor drive pods.
 */
public class DriveBase extends Subsystem {

	private DrivePodSpark leftPod, rightPod;
	private Solenoid shifter;

	private double leftSpeed;
	private double rightSpeed;

	// Mode for the gearshift, as set by the auto moves
	public enum GearShiftMode {
		LOCK_HIGH_GEAR, LOCK_LOW_GEAR, AUTOSHIFT,
	}

	private GearShiftMode gearShiftMode = GearShiftMode.AUTOSHIFT;

	private Timer shiftTimer = new Timer();
	private boolean allowShift = true;
	private boolean allowDeshift = true;
	private boolean hasAlreadyShifted = false;

	public DriveBase() {
		super();

		// Note that one pod must be inverted, since the gearbox assemblies are
		// rotationally symmetrical
		leftPod = new DrivePodSpark("Left", Constants.LEFT_LEAD, Constants.LEFT_F, false);
		rightPod = new DrivePodSpark("Right", Constants.RIGHT_LEAD, Constants.RIGHT_F, true);
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
	 * Turn dynamic braking on or off
	 * 
	 * @param isEnabled true to brake, false to freewheel
	 */
	public void brake(boolean isEnabled) {
		leftPod.enableBrakeMode(isEnabled);
		rightPod.enableBrakeMode(isEnabled);
	}

	/**
	 * Drive at the commanded throttle values
	 * 
	 * @param leftThrottle  between -1 and +1
	 * @param rightThrottle between -1 and +1
	 */
	public void driveWithTankControls(double leftThrottle, double rightThrottle) {
		leftPod.setThrottle(leftThrottle);
		rightPod.setThrottle(rightThrottle);
	}

	/**
	 * Drive with the given forward and turn values
	 * 
	 * @param forward between -1 and +1
	 * @param spin    between -1 and +1, where -1 is full leftward (CCW when viewed
	 *                from above)
	 */
	public void driveWithForwardAndSpin(double forward, double spin) {
		driveWithTankControls(forward - spin, forward + spin);
	}

	/**
	 * Drive with the forward and turn values from the joysticks
	 */
	public void driveWithJoysticks() {
		// setMaxSpeed(1);
		double y = Robot.oi.getForwardAxis();
		double x = Robot.oi.getTurnAxis();

		/*
		 * "Exponential" drive, where the movements are more sensitive during slow
		 * movement, permitting easier fine control
		 */
		x = Math.pow(x, 3);
		y = Math.pow(y, 3);
		driveWithForwardAndSpin(y, x);
	}

	// public void setMaxSpeed(double maxSpeed) {
	// 	leftPod.setMaxSpeed(maxSpeed);
	// 	rightPod.setMaxSpeed(maxSpeed);
	// }

	/**
	 * Get the instantaneous speed of the left side drive pod, in feet per second
	 * 
	 * @return instantaneous speed of the left side drive pod, in feet per second
	 */
	// public double getLeftSpeed() {
	// 	return leftPod.getEncoderVelocityFeetPerSecond();
	// }

	/**
	 * Get the instantaneous speed of the right side drive pod, in feet per second
	 * 
	 * @return instantaneous speed of the right side drive pod, in feet per second
	 */
	// public double getRightSpeed() {
	// 	return rightPod.getEncoderVelocityFeetPerSecond();
	// }

	/**
	 * Get the driven distance of the left drive pod in ticks
	 * 
	 * @return driven distance of the left drive pod in ticks
	 */
	// public double getLeftEncoderPos() {
	// 	return leftPod.getQuadEncPos();
	// }

	/**
	 * Get the driven distance of the right drive pod in ticks
	 * 
	 * @return driven distance of the right drive pod in ticks
	 */
	// public double getRightEncoderPos() {
	// 	return rightPod.getQuadEncPos();
	// }

	/**
	 * Apply the appropriate gear decided by the auto/manual shifting logic
	 * 
	 * @param isHighGear true for high gear, false for low gear
	 */
	private void setGear(boolean isHighGear) {
		// System.out.println("Shifting to " + (isHighGear? "high":"low") + " gear");
		if (shifter != null) {
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
		if (shifter != null) {
			return shifter.get();
		} else {
			return false;
		}
	}

	/**
	 * Apply the correct gear for the speed of the drivebase right now. Should only
	 * be called when the gear shift mode permits auto shifting.
	 */
	// private void autoShift() {
	// 	leftSpeed = Math.abs(Robot.drivebase.getLeftSpeed());
	// 	rightSpeed = Math.abs(Robot.drivebase.getRightSpeed());

	// 	// Autoshift framework based off speed
	// 	if (allowShift) {
	// 		if (((leftSpeed < Constants.SPEED_TO_SHIFT_DOWN) && (rightSpeed < Constants.SPEED_TO_SHIFT_DOWN))) {
	// 			setGear(false);
	// 			// System.out.println(elevatorIsTooHighToShift + " case 1");

	// 			if (hasAlreadyShifted) {
	// 				allowDeshift = true;
	// 				hasAlreadyShifted = false;
	// 			}

	// 		} else if (((leftSpeed > Constants.SPEED_TO_SHIFT_UP)) || ((rightSpeed > Constants.SPEED_TO_SHIFT_UP))) {
	// 			if (allowDeshift) {
	// 				shiftTimer.reset();
	// 				shiftTimer.start();
	// 				allowShift = false;
	// 				setGear(true);
	// 				// System.out.println(elevatorIsTooHighToShift + " case 2");
	// 			}
	// 		}
	// 	} else if (shiftTimer.get() > 1.0) {
	// 		allowShift = true;
	// 		shiftTimer.stop();
	// 		shiftTimer.reset();
	// 		allowDeshift = false;
	// 		hasAlreadyShifted = true;
	// 	}
	// }

	// /**
	//  * Ask if an autonomous move has asked the robot to remain in a particular gear
	//  */
	// public GearShiftMode getShiftMode() {
	// 	return gearShiftMode;
	// }

	// /**
	//  * To be called by an auto mode, to keep the drivebase in a certain gear or let
	//  * it run free.
	//  * 
	//  * Can be overridden by the driver via the OI.
	//  * 
	//  * @param shiftMode
	//  */
	// public void setShiftMode(GearShiftMode shiftMode) {
	// 	gearShiftMode = shiftMode;
	// }

	public void visit() {
		// handleGear();
	}

	/**
	 * Enact whichever shift mode is appropriate
	 */
	// private void handleGear() {
	// 	// Driver commanded override?
	// 	if (Robot.oi.getHighGear()) {
	// 		setGear(true);
	// 	} else if (Robot.oi.getLowGear()) {
	// 		setGear(false);
	// 	} else {
	// 		// No override from driver. Auto move commanded override?
	// 		switch (gearShiftMode) {
	// 		case LOCK_HIGH_GEAR:
	// 			setGear(true);
	// 			break;
	// 		case LOCK_LOW_GEAR:
	// 			setGear(false);
	// 			break;
	// 		// No override commanded; handle automatic gear shifting.
	// 		case AUTOSHIFT:
	// 			autoShift();
	// 			break;
	// 		}
	// 	}
	// }

}
