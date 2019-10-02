package frc.robot.commands.elevator;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorHoldPoint;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManuallyControlElevator extends Command {

	private boolean wasHoldingPresentPositionLastIteration = false;

	public ManuallyControlElevator() {
		// This method is run once during robot startup
		requires(Robot.elevator);
	}

	@Override
	public synchronized void start() {
		super.start();
		// This method is called once when the command is activated
		Robot.elevator.seekHoldPoint(ElevatorHoldPoint.HERE);
		wasHoldingPresentPositionLastIteration = true;
	}

	@Override
	protected void execute() {
		// This method is called every iteration

		final String ELEV_MODE = "Elevator mode";
		
		// First priority: Is the user holding down one of the seek buttons?
		if(Robot.oi.getCommandedHoldPoint() != ElevatorHoldPoint.NONE) {
			Robot.elevator.seekHoldPoint(Robot.oi.getCommandedHoldPoint());
			wasHoldingPresentPositionLastIteration = false;
			SmartDashboard.putString(ELEV_MODE, "Hold position " + Robot.oi.getCommandedHoldPoint().toString());
		} else {
			// Second priority: Is the stick outside the deadband?
			if (Math.abs(Robot.oi.getElevatorSpeed()) > 0) {
				SmartDashboard.putString(ELEV_MODE, "Set speed");
				Robot.elevator.setElevatorSpeed(Robot.oi.getElevatorSpeed());
				wasHoldingPresentPositionLastIteration = false;
			} else {
				if (Robot.oi.getElevatorHomeButtonPressed()) {
					// Third priority: re-set home based on button press,
					// but first set speed so not position holding
					Robot.elevator.setElevatorSpeed(0);
					Robot.elevator.checkAndApplyHomingSwitch();
					wasHoldingPresentPositionLastIteration = false;
					SmartDashboard.putString(ELEV_MODE, "Re-home based on button");
				} else {
					// Fourth priority: hold the present position
					if (!wasHoldingPresentPositionLastIteration) {
						Robot.elevator.seekHoldPoint(ElevatorHoldPoint.HERE);
						wasHoldingPresentPositionLastIteration = true;
					} else {
						// We already commanded the elevator to hold its present
						// position, so we don't need to command it to do so again.
					}
					SmartDashboard.putString(ELEV_MODE, "Hold present position");
				}
			}
		}
	}

	@Override
	public synchronized void cancel() {
		// Cancel any position seeking
		Robot.elevator.stopMotor();
	}

	@Override
	protected boolean isFinished() {
		return false; // until interrupted
	}

}
