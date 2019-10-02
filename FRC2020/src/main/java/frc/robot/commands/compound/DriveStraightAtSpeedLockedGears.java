package frc.robot.commands.compound;

import frc.robot.commands.drivebase.DriveStraight;
import frc.robot.commands.drivebase.LockGear;
import frc.robot.commands.drivebase.SetMaxSpeed;
import frc.robot.commands.drivebase.UnlockGear;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveStraightAtSpeedLockedGears extends CommandGroup{
	
	public DriveStraightAtSpeedLockedGears(double inchesToTravel, boolean isHighGear, double maxSpeed) {
		addSequential(new LockGear(isHighGear));
		addSequential(new SetMaxSpeed(maxSpeed));
		addSequential(new DriveStraight(inchesToTravel));
		addSequential(new UnlockGear());
		addSequential(new SetMaxSpeed(1));
	}
	
}
