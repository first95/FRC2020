package frc.robot.commands.elevator;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.command.Command;

public class SetElevatorHeight extends Command {
	
	private Elevator.ElevatorHoldPoint targetPoint = Elevator.ElevatorHoldPoint.NONE;
    private Double targetFeet = 0.0;
	
	public SetElevatorHeight(Elevator.ElevatorHoldPoint targetHoldPoint) {
		// This method is run once during robot startup
		requires(Robot.elevator);
        targetPoint = targetHoldPoint;
	}

	public SetElevatorHeight(double targetFeet) {
		// This method is run once during robot startup
		requires(Robot.elevator);
        this.targetFeet = targetFeet;
	}


    @Override
    protected void execute() {
        super.execute();

        if (targetPoint != Elevator.ElevatorHoldPoint.NONE) {
            Robot.elevator.seekHoldPoint(targetPoint);
        } else {
            Robot.elevator.setElevatorHeight(targetFeet);
        }
	}

	@Override
	public synchronized void cancel() {
		// Cancel any position seeking
		Robot.elevator.stopMotor();
	}
	
	@Override
	protected boolean isFinished() {
		return Robot.elevator.isOnTarget();
	}
}
