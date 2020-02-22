/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class PositionCloseLoopControlCommand extends Command {
  public PositionCloseLoopControlCommand() {
      requires(Robot.drivebase);
  }

  @Override
  public void start() {
        super.start();

        // Read Set Rotations from SmartDashboard
        double rotations = SmartDashboard.getNumber("SetPoint", 0);

        Robot.drivebase.travleDistance(rotations);

        // Display set point (in rotations) on SmartDashboard
		    SmartDashboard.putNumber("SetPoint", rotations);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
