/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class ManuallyControlShooter extends Command {
  public static double MANUAL_RUN_SPEED = 0.4;
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double current_speed = 0;

  public ManuallyControlShooter() { 
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      // Singulator speed setting
      // if(Robot.oi.getShooterButton()) {
      //   Robot.shooter.runShooterOpen(MANUAL_RUN_SPEED);
      //   current_speed = MANUAL_RUN_SPEED;
      // } else {
      //   // Slowing down motor and don't want to do it too fast
      //   if (current_speed < MIN_RUN_SPEED) {
      //     current_speed = 0;
      //   } else {
      //     current_speed = current_speed*MANUAL_RUN_SPEED;
      //   }
      //   Robot.shooter.runShooterOpen(current_speed);
      // }
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
