/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.commands.hscorer;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Score a hatch assuming aligned properly vertically
 */
public class ScoreHatch extends CommandGroup {
	
	public ScoreHatch() {
        addSequential(new PushIt(true));
        addSequential(new GrabIt(false));
        addSequential(new PushIt(false));
	}

}
