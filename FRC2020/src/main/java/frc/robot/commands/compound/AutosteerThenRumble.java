package frc.robot.commands.compound;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI.Controller;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.drivebase.AutosteerAlongLine;

/**
 * This command runs autosteer until it finishes (sees wall with both forward sensors),
 * and then rumbles. Would prefer if rumble were conditional on center bottom sensor
 * seeing line.
 */
public class AutosteerThenRumble extends CommandGroup {
    public static final double RUMBLE_TIME_S = 0.5;
    public AutosteerThenRumble() {
        // Autosteer along line
        addSequential(new AutosteerAlongLine());
        // Buzz it after autosteer completes
        addSequential(new RumbleCommand(Controller.DRIVER, RumbleType.kLeftRumble, 1.0, RUMBLE_TIME_S));
    }
}