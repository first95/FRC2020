package frc.robot.commands.hgroundloader;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class SetIntakeThrottle extends Command {
	private double intakeSpeed;
	
	public SetIntakeThrottle (double speed) {
        requires(Robot.hGroundLoader);
        intakeSpeed = speed;
    }
    
    @Override
    public synchronized void start() {
        System.out.println("SetIntakeThrottle.start()");
        super.start();
    }

	@Override
	protected void execute() {
		Robot.hGroundLoader.setIntakeSpeed(intakeSpeed);
	}

    @Override
    protected boolean isFinished() {
        boolean fin = true;
        System.out.println("SetIntakeThrottle.isFinished() - " + fin);
        return fin;
    }

    @Override
    protected void end() {
        System.out.println("SetIntakeThrottle.end()");
        super.end();
    }
}