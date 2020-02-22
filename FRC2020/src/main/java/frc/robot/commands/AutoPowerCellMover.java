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

  private boolean wasDeployedButtonPressed = false;

  public boolean shooterButtonPressed = Robot.oi.getShooterButton();
  public static boolean shooterIsLoadedCheck;
  public static boolean movingFromSingulator;
  public static boolean movingIntoIndexer;
  public static boolean isInIdexer;

  public static boolean dummy = false;

  public int time = 0;

  public static double MANUAL_RUN_SPEED = 0.4;
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double current_speed = 0;

  public AutoPowerCellMover() {
    requires(Robot.powerCellMover);
    // requires(Robot.shooter);
    // requires(Robot.groundPickUp);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // If the deploy button was not pressed during the last loop and is pressed
    // during the current loop,
    // toggle deploy
    // dummy = true;

    System.out.println("time = " + time);

    if (Robot.powerCellMover.getShooterLoadedSensor() == true) {
      Robot.powerCellMover.setSingulatorSpeed(0);
      Robot.powerCellMover.runIndexer(0);
      Robot.powerCellMover.runShooterOpen(0);

      shooterIsLoadedCheck = true;
    } else {
      shooterIsLoadedCheck = false;
    }

    if (shooterIsLoadedCheck == false) {
      if ((Robot.powerCellMover.getSingulatorSensor() == false)
          && (Robot.powerCellMover.getIndexerEntranceSensor() == false)
          && (Robot.powerCellMover.getIndexerLoadedSensor() == false)) {
        Robot.powerCellMover.setSingulatorSpeed(0);
        Robot.powerCellMover.runIndexer(0);
        movingFromSingulator = false;
        movingIntoIndexer = false;
        isInIdexer = false;
      } else if ((Robot.powerCellMover.getSingulatorSensor() == true)
          && (Robot.powerCellMover.getIndexerEntranceSensor() == false)
          && (Robot.powerCellMover.getIndexerLoadedSensor() == false)) {
        Robot.powerCellMover.setSingulatorSpeed(0.8);
        Robot.powerCellMover.runIndexer(0);
        movingFromSingulator = true;
        movingIntoIndexer = false;
        isInIdexer = false;
      } else if ((Robot.powerCellMover.getSingulatorSensor() == true)
          && (Robot.powerCellMover.getIndexerEntranceSensor() == true)
          && (Robot.powerCellMover.getIndexerLoadedSensor() == false)) {
        Robot.powerCellMover.setSingulatorSpeed(0.3);
        Robot.powerCellMover.runIndexer(0.5);
        movingFromSingulator = true;
        movingIntoIndexer = true;
        isInIdexer = false;
      } else if (Robot.powerCellMover.getSingulatorSensor() == true
          && Robot.powerCellMover.getIndexerEntranceSensor() == false
          && Robot.powerCellMover.getIndexerLoadedSensor() == true) {
        Robot.powerCellMover.setSingulatorSpeed(0.8);
        Robot.powerCellMover.runIndexer(0);
        movingFromSingulator = true;
        movingIntoIndexer = false;
        isInIdexer = true;
      } else if (Robot.powerCellMover.getSingulatorSensor() == true
          && Robot.powerCellMover.getIndexerEntranceSensor() == true
          && Robot.powerCellMover.getIndexerLoadedSensor() == true) {
        Robot.powerCellMover.setSingulatorSpeed(0.3);
        Robot.powerCellMover.runIndexer(0.5);
        movingFromSingulator = true;
        movingIntoIndexer = true;
        isInIdexer = true;
      } else if (Robot.powerCellMover.getSingulatorSensor() == false
          && Robot.powerCellMover.getIndexerEntranceSensor() == true
          && Robot.powerCellMover.getIndexerLoadedSensor() == false) {
        Robot.powerCellMover.setSingulatorSpeed(0);
        Robot.powerCellMover.runIndexer(0.5);
        movingFromSingulator = false;
        movingIntoIndexer = true;
        isInIdexer = false;
      } else if (Robot.powerCellMover.getSingulatorSensor() == false
          && Robot.powerCellMover.getIndexerEntranceSensor() == true
          && Robot.powerCellMover.getIndexerLoadedSensor() == true) {
        Robot.powerCellMover.setSingulatorSpeed(0);
        Robot.powerCellMover.runIndexer(0.5);
        movingFromSingulator = false;
        movingIntoIndexer = true;
        isInIdexer = true;
      } else {
        Robot.powerCellMover.setSingulatorSpeed(0);
        Robot.powerCellMover.runIndexer(0);
        movingFromSingulator = false;
        movingIntoIndexer = false;
        isInIdexer = false;
      }
    } else if (shooterIsLoadedCheck == true && Robot.powerCellMover.getSingulatorSensor()) {
      Robot.powerCellMover.setSingulatorSpeed(0.8);
      Robot.powerCellMover.runIndexer(0);
      movingFromSingulator = true;
      movingIntoIndexer = false;
      isInIdexer = false;
    } else {
      Robot.powerCellMover.setSingulatorSpeed(0);
      Robot.powerCellMover.runIndexer(0);
      movingFromSingulator = false;
      movingIntoIndexer = false;
      isInIdexer = false;
    }
  }

  public void autoIndexGroundPickUp()
  {
    if (!wasDeployedButtonPressed && Robot.oi.getGroundPickUpDeployed()) {
      Robot.powerCellMover.toggleGroundPickUpDeploy();
      System.out.println("ground pickup has been deployed");
    }

    Robot.powerCellMover.setRollerSpeed(Robot.oi.getGroundPickUpRollerAxis());
  }

  public void autoIndexShooter()
  {
    if (Robot.oi.getShooterButton()) {
      Robot.powerCellMover.runShooterOpen(MANUAL_RUN_SPEED);
      current_speed = MANUAL_RUN_SPEED;

      if(time++ >= 1000)
      {
        Robot.powerCellMover.runIndexer(1);
        Robot.powerCellMover.setSingulatorSpeed(0.4);
      }
      else
      {
        Robot.powerCellMover.runIndexer(0);
        Robot.powerCellMover.setSingulatorSpeed(0);
      }
      
    } else {
      // Slowing down motor and don't want to do it too fast
      if (current_speed < MIN_RUN_SPEED) {
        current_speed = 0;
      } else {
        current_speed = current_speed * MANUAL_RUN_SPEED;
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
