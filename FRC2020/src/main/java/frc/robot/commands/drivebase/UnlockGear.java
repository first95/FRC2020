package frc.robot.commands.drivebase;

import frc.robot.Robot;
import frc.robot.subsystems.DriveBase.GearShiftMode;
import edu.wpi.first.wpilibj.command.Command;

public class UnlockGear extends Command{

	@Override
	protected void initialize() {
		super.initialize();
		Robot.drivebase.setShiftMode(GearShiftMode.AUTOSHIFT);
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

}
