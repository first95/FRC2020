package frc.robot.commands.cargohandler;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CombinedControlCargoHandler extends Command {
	// Note that a different set of constants is used in autonomous control
	public static double UP_DEG = 0.0;
	public static double COLLECT_DEG = -50.0;

	public enum WristHoldPoint {
		UP, // Positioned at its highest
		COLLECT, // Positioned to collect
		HERE, // Not a specific location - indicates holding whatever point the elevator is at
				// now.
	};

	private boolean wasHoldingPresentPositionLastIteration = false;
	private double intakeSpeed;
	private boolean hasStarted = false;

	public CombinedControlCargoHandler() {
		// This method is run once during robot startup
		requires(Robot.cargoHandler);
		hasStarted = false;
	}

	@Override
	protected void execute() {
		// This method is called every iteration
		// Check if we've started yet
		if(!hasStarted) {
			seekHoldPoint(WristHoldPoint.HERE);
			wasHoldingPresentPositionLastIteration = true;
			hasStarted = true;
		}

		// First do the simple intake control
		intakeSpeed = Robot.oi.getCargoHandlerIntakeSpeed();
		Robot.cargoHandler.setIntakeSpeed(intakeSpeed);
		SmartDashboard.putNumber("Cargo Handler Intake Input", intakeSpeed);

		// Then do the more complicated wrist control
		final String CH_WRIST_MODE = "CH wrist mode";
		// First priority: Is the user holding down one of the seek buttons?
		if (Robot.oi.isCHWristUpButtonPressed()) {
			SmartDashboard.putString(CH_WRIST_MODE, "Seek up");
			seekHoldPoint(WristHoldPoint.UP);
			wasHoldingPresentPositionLastIteration = false;
		} else if (Robot.oi.isCHWristCollectButtonPressed()) {
			SmartDashboard.putString(CH_WRIST_MODE, "Seek collect");
			seekHoldPoint(WristHoldPoint.COLLECT);
			wasHoldingPresentPositionLastIteration = false;
		} else {
			// Second priority: Is the stick outside the deadband?
			if (Math.abs(Robot.oi.getCargoHandlerWristSpeed()) > 0.1) {
				SmartDashboard.putString(CH_WRIST_MODE, "Set speed");
				Robot.cargoHandler.setWristPitchSpeed(Robot.oi.getCargoHandlerWristSpeed());
				wasHoldingPresentPositionLastIteration = false;
			} else {
				// Third priority: hold the present position
				if (!wasHoldingPresentPositionLastIteration) {
					seekHoldPoint(WristHoldPoint.HERE);
					// System.out.println("TEST THE CH WRIST HOLD POINT IS: " + WristHoldPoint.HERE);
					wasHoldingPresentPositionLastIteration = true;
				} else {
					// We already commanded the elevator to hold its present
					// position, so we don't need to command it to do so again.
				}
				SmartDashboard.putString(CH_WRIST_MODE, "Hold present position");
			}
		 }
	}

	@Override
	public synchronized void cancel() {
        // Cancel any position seeking
        //System.out.println("CombinedControlCargoHandler.cancel()");
		Robot.cargoHandler.stopWrist();
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
			desiredAngleDeg = Robot.cargoHandler.getWristAngleDeg();
			break;
		}

		Robot.cargoHandler.setWristAngleDeg(desiredAngleDeg);
	}
}
