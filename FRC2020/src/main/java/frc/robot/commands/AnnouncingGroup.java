package frc.robot.commands;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;

import edu.wpi.first.wpilibj.Joystick;
public class AnnouncingGroup extends CommandGroup {

	public AnnouncingGroup() {  
        super();

        addSequential(new RumbleCommand(OI.Controller.DRIVER, Joystick.RumbleType.kLeftRumble, 1.0, 1.0));
        addSequential(new RumbleCommand(OI.Controller.DRIVER, Joystick.RumbleType.kRightRumble, 1.0, 1.0));
        addSequential(new Nothing());
	}
}