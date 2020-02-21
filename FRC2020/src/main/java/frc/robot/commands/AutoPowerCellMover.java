/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.PowerCellMover;

public class AutoPowerCellMover extends Command {

  public boolean shooterButtonPressed = Robot.oi.getShooterButton();
  public static boolean shooterIsLoadedCheck;
  public static boolean movingFromSingulator;
  public static boolean movingIntoIndexer;
  public static boolean isInIdexer;

  public static boolean dummy = false;

  public AutoPowerCellMover() {
    requires(Robot.powerCellMover);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    dummy = true;

    if(shooterButtonPressed)
    {
      // Robot.powerCellMover.setSingulatorSpeed(0.8);
      // Robot.powerCellMover.runIndexer(0.8);
      // Robot.shooter.runShooterOpen(0.8);
    }
    else
    {
      if(Robot.powerCellMover.getShooterLoadedSensor() == true)
      {
        // Robot.powerCellMover.setSingulatorSpeed(0);
        // Robot.powerCellMover.runIndexer(0);
        // Robot.shooter.runShooterOpen(0);

        shooterIsLoadedCheck = true;
      }
      else
      {
        shooterIsLoadedCheck = false;
      }

      if(shooterIsLoadedCheck == false)
      {
        if(Robot.powerCellMover.getSingulatorSensor() == true && Robot.powerCellMover.getIndexerEntranceSensor() == false)
        {
          // Robot.powerCellMover.setSingulatorSpeed(0.8);
          // Robot.powerCellMover.runIndexer(0);
          movingFromSingulator = true;
          movingIntoIndexer = false;
          isInIdexer = false;
        }
        else if(Robot.powerCellMover.getIndexerEntranceSensor() == true && Robot.powerCellMover.getIndexerLoadedSensor() == false)
        {
          // Robot.powerCellMover.setSingulatorSpeed(0);
          // Robot.powerCellMover.runIndexer(0.2);
          movingFromSingulator = false;
          movingIntoIndexer = true;
          isInIdexer = false;
        }
        else if(Robot.powerCellMover.getIndexerLoadedSensor() == true)
        {
          // Robot.powerCellMover.setSingulatorSpeed(0);
          // Robot.powerCellMover.runIndexer(0);
          movingFromSingulator = false;
          movingIntoIndexer = false;
          isInIdexer = true;
        }
        else
        {
          // Robot.powerCellMover.setSingulatorSpeed(0);
          // Robot.powerCellMover.runIndexer(0);
          movingFromSingulator = false;
          movingIntoIndexer = false;
          isInIdexer = false;
        }
      }
      else
      {
        // Robot.powerCellMover.setSingulatorSpeed(0);
        // Robot.powerCellMover.runIndexer(0);
        movingFromSingulator = false;
        movingIntoIndexer = false;
        isInIdexer = false;
      }
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
