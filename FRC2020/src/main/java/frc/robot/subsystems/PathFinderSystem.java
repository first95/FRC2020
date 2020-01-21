/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.Nothing;
import frc.robot.commands.drivebase.PathFinderCommand;
import frc.robot.components.DrivePod;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Add your docs here.
 */
public class PathFinderSystem extends Subsystem
{
  // public static boolean lOrR;
  // public static boolean whatGearAreWeIn;
  // public static double Left_To_Right_Distance;
  // public static int whatPath;

  // public PathFinderSystem(boolean lOrR2, boolean whatGearAreWeIn2, double Left_To_Right_Distance2, int whatPath) {
  //   super();
  //   this.lOrR = lOrR2;
  //   this.whatGearAreWeIn = whatGearAreWeIn2;
  //   this.Left_To_Right_Distance = Left_To_Right_Distance2;
  //   this.whatPath = whatPath;
  // }


  @Override
  protected void initDefaultCommand()
  {
    setDefaultCommand(new Nothing());//new PathFinderCommand(lOrR, whatGearAreWeIn, Left_To_Right_Distance, whatPath));
  }
}
