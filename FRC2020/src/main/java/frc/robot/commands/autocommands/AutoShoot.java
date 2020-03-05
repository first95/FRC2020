/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoShoot extends Command {
  private long startTime;
  private long timeOutMs;

  public int time = 0;
  public static double INDEXER_RUN_SPEED = 0.3;
  public static double SINGULATOR_RUN_SPEED = 0.5;

  public static double MANUAL_RUN_SPEED_SHOOTER = 0.5;
  public static double TARGET_RUN_SPEED_SHOOTER = 3200; // ideal speed in RPM
  public static double RUN_TOLERANCE_SHOOTER = 120; // tolerance range for shooter speed
  public static double MAINTAIN_RUN_SPEED_SHOOTER = 0.38; // want this to roughly hold target RPM
  public static double SLOW_RUN_SPEED_SHOOTER = MAINTAIN_RUN_SPEED_SHOOTER - 0.04; // want this to slow down a bit but not fully
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double actual_speed = 0;
  private double current_speed = 0;



  public AutoShoot(long timeOutMs) {
    this.timeOutMs = timeOutMs;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.powerCellMover);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if ((System.currentTimeMillis() - startTime) < (timeOutMs - 2000)) {
      actual_speed = Robot.powerCellMover.getShooterSpeed();
      if (actual_speed < TARGET_RUN_SPEED_SHOOTER - RUN_TOLERANCE_SHOOTER) {
        current_speed = 1.0; // speed up as quickly as possible
      } else if (actual_speed < TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER) {
        current_speed = MAINTAIN_RUN_SPEED_SHOOTER;
      } else {
        // implies actual_speed >= TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER
        current_speed = SLOW_RUN_SPEED_SHOOTER;
      }
      Robot.powerCellMover.runShooterClosed(1000, ControlType.kVelocity);

      if (time >= 140) {
        Robot.powerCellMover.runIndexer(0.6);
      } else {
        Robot.powerCellMover.runIndexer(0);
        time++;
      }
    } else {
      time = 0;
      // Slowing down motor and don't want to do it too fast
      if (current_speed < MIN_RUN_SPEED) {
        current_speed = 0;
      } else {
        current_speed = current_speed * MANUAL_RUN_SPEED_SHOOTER;
      }
      Robot.powerCellMover.runShooterClosed(1000, ControlType.kVelocity);
    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ((startTime + timeOutMs) < System.currentTimeMillis() );
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.powerCellMover.runShooterOpen(0);
    Robot.powerCellMover.runShooterClosed(0, ControlType.kVelocity);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
