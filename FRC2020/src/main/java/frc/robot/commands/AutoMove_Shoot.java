/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.subsystems.PowerCellMover;

public class AutoMove_Shoot extends TimedCommand {
  public static double SHOOT_DURATION_S = 10; // Amount of time to wait for this command to finish

  public int time = 0;
  public static double INDEXER_RUN_SPEED = 0.5;
  public static double SINGULATOR_RUN_SPEED = 0.5;

  public static double MANUAL_RUN_SPEED_SHOOTER = 0.5;
  public static double TARGET_RUN_SPEED_SHOOTER = 2400; // ideal speed in RPM
  public static double RUN_TOLERANCE_SHOOTER = 120; // tolerance range for shooter speed
  public static double MAINTAIN_RUN_SPEED_SHOOTER = 0.38; // want this to roughly hold target RPM
  public static double SLOW_RUN_SPEED_SHOOTER = MAINTAIN_RUN_SPEED_SHOOTER - 0.04; // want this to slow down a bit but not fully
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double actual_speed = 0;
  private double current_speed = 0;

  public AutoMove_Shoot() {
    super(SHOOT_DURATION_S);
    requires(Robot.powerCellMover);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (time < 1000) {
      // Get actual speed
      actual_speed = Robot.powerCellMover.getShooterSpeed();
      if (actual_speed < TARGET_RUN_SPEED_SHOOTER - RUN_TOLERANCE_SHOOTER) {
        current_speed = 0.8; // speed up quickly
      } else if (actual_speed < TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER) {
        current_speed = MAINTAIN_RUN_SPEED_SHOOTER;
      } else {
        // implies actual_speed >= TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER
        current_speed = SLOW_RUN_SPEED_SHOOTER;
      }
      Robot.powerCellMover.runShooterOpen(current_speed);

      if (time >= 70) {
        Robot.powerCellMover.runIndexer(1);
        Robot.powerCellMover.setSingulatorSpeed(0.4);
      } else {
        Robot.powerCellMover.runIndexer(0);
        Robot.powerCellMover.setSingulatorSpeed(0);
        time++;
      }
    } else {
      // Slowing down motor and don't want to do it too fast
      if (current_speed < MIN_RUN_SPEED) {
        current_speed = 0;
      } else {
        current_speed = current_speed * MANUAL_RUN_SPEED_SHOOTER;
      }
      Robot.powerCellMover.runShooterOpen(current_speed);
    }
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
