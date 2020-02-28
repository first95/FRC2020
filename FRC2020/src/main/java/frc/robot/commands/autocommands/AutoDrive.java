/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Auto Drice for specified number of miliseconds at specified percent speed
 */
public class AutoDrive extends Command {
  private long startTime;
  private long timeOutMs;
  private double speed;

  public AutoDrive(long timeOutMs, double speed) {
    this.timeOutMs = timeOutMs;
    this.speed = speed;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivebase.driveWithTankControls(speed, speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ((startTime + timeOutMs) < System.currentTimeMillis() );
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivebase.driveWithTankControls(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
