/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * An example command. You can replace me with your own command.
 */
public class AutoMoves extends CommandGroup {
  public AutoMoves() {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.m_subsystem);
    addSequential(new AutoSpinUpShooter(1000));
    addSequential(new AutoShoot(1000));
    addSequential(new AutoDrive(1000));
  }

}
