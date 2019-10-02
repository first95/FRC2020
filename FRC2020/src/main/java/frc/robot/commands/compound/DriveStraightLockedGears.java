package frc.robot.commands.compound;

import frc.robot.commands.drivebase.DriveStraight;
import frc.robot.commands.drivebase.LockGear;
import frc.robot.commands.drivebase.UnlockGear;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveStraightLockedGears extends CommandGroup{
	
	public DriveStraightLockedGears(double inchesToTravel, boolean isHighGear) {
		addSequential(new LockGear(isHighGear));
		addSequential(new DriveStraight(inchesToTravel));
		addSequential(new UnlockGear());
	}
	
}
