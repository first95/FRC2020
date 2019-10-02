package frc.robot.commands.hgroundloader;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CombinedControlHGroundLoader extends Command {
	// Note that a different set of constants is used in autonomous control
	public static double UP_DEG = 0.0;
	public static double COLLECT_DEG = 90.0;

	public enum WristHoldPoint {
		UP, // Positioned at its highest
		COLLECT, // Positioned to collect
		HERE, // Not a specific location - indicates holding whatever point the elevator is at
				// now.
	};

	private boolean wasHoldingPresentPositionLastIteration = false;
	private double intakeSpeed;

	public CombinedControlHGroundLoader() {
		// This method is run once during robot startup
		requires(Robot.hGroundLoader);
		this.setInterruptible(true);
	}

	@Override
	public synchronized void start() {
		// This method is called once when the command is activated
		seekHoldPoint(WristHoldPoint.HERE);
		wasHoldingPresentPositionLastIteration = true;
	}

	@Override
	protected void execute() {
		// This method is called every iteration
		// First do the simple intake control
		intakeSpeed = Robot.oi.getHGLIntakeSpeed();
		Robot.hGroundLoader.setIntakeSpeed(intakeSpeed);
		SmartDashboard.putNumber("HGL Intake Input", intakeSpeed);

		// Then do the more complicated wrist control
		final String HGL_WRIST_MODE = "HGL wrist mode";
		// First priority: Is the user holding down one of the seek buttons?
		if (Robot.oi.isHGLWristUpButtonPressed()) {
			SmartDashboard.putString(HGL_WRIST_MODE, "Seek up");
			seekHoldPoint(WristHoldPoint.UP);
			wasHoldingPresentPositionLastIteration = false;
		} else {
			// Second priority: Is the stick outside the deadband?
			if (Math.abs(Robot.oi.getHGLWristSpeed()) > 0.1) {
				SmartDashboard.putString(HGL_WRIST_MODE, "Set speed");
				Robot.hGroundLoader.setWristPitchSpeed(Robot.oi.getHGLWristSpeed());
				wasHoldingPresentPositionLastIteration = false;
			} else {
				// Third priority: hold the present position
				if (!wasHoldingPresentPositionLastIteration) {
					seekHoldPoint(WristHoldPoint.HERE);
					// System.out.println("TEST THE HGL WRIST HOLD POINT IS: " + WristHoldPoint.HERE);
					wasHoldingPresentPositionLastIteration = true;
				} else {
					// We already commanded the elevator to hold its present
					// position, so we don't need to command it to do so again.
				}
				SmartDashboard.putString(HGL_WRIST_MODE, "Hold present position");
			}
		 }
	}

	@Override
	public synchronized void cancel() {
		// Cancel any position seeking
		Robot.hGroundLoader.stopWrist();
	}

	@Override
	protected boolean isFinished() {
		return false; // until interrupted
	}

	/**
	 * Seek a predefined point. The CH wrist will seek and hold this point until it
	 * loses power or receives a new command. It will remember to seek this point
	 * when disabled, unless a command is given to cause it to forget.
	 * 
	 * @param point
	 *            - which point to seek, see WristHoldPoint
	 */
	private void seekHoldPoint(WristHoldPoint point) {
		double desiredAngleDeg = 0;
		switch (point) {
		case UP:
			desiredAngleDeg = UP_DEG;
			break;
		case COLLECT:
			desiredAngleDeg = COLLECT_DEG;
			break;
		case HERE: // Fallthrough - the HERE and default cases have the same action
		default:
			desiredAngleDeg = Robot.hGroundLoader.getWristAngleDeg();
			break;
		}

		Robot.hGroundLoader.setWristAngleDeg(desiredAngleDeg);
	}
}
