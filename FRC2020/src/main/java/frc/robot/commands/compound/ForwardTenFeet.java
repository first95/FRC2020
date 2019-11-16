/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivebase.PathFinderCommand;

public class ForwardTenFeet extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ForwardTenFeet()
  {
    addSequential(new PathFinderCommand(true, false, 20, 1));
  }
}
