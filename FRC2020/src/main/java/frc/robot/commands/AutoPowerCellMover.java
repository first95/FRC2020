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

  public static boolean shooterIsLoadedCheck;
  public static boolean movingFromSingulator;
  public static boolean movingIntoIndexer;
  public static boolean isInIdexer;

  private boolean wasDeployedButtonPressed = false;

  public static boolean dummy = false;

  public int time = 0;
  public static double INDEXER_RUN_SPEED = 0.5;
  public static double SINGULATOR_RUN_SPEED = 0.25;

  public static double MANUAL_RUN_SPEED_SHOOTER = 0.5;
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double current_speed = 0;

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
    // dummy = true;

    // if (Robot.oi.getBackwardsButtonPressed() == false) {
    //   if (Robot.powerCellMover.getShooterLoadedSensor() == true
    //       && Robot.powerCellMover.getSingulatorSensor() == false) {
    //     Robot.powerCellMover.setSingulatorSpeed(0);
    //     Robot.powerCellMover.runIndexer(0);
    //     Robot.powerCellMover.runShooterOpen(0);
    //     AutoPowerCellMoverGroundCollect();
    //     AutoPowerCellMoverShooter();
    //   } else if (Robot.powerCellMover.getShooterLoadedSensor() == false) {
    //     if (Robot.powerCellMover.getSingulatorSensor() == false
    //         && Robot.powerCellMover.getIndexerEntranceSensor() == false
    //         && Robot.powerCellMover.getIndexerLoadedSensor() == false) {
    //       Robot.powerCellMover.setSingulatorSpeed(0);
    //       Robot.powerCellMover.runIndexer(0);
    //       movingFromSingulator = false;
    //       movingIntoIndexer = false;
    //       isInIdexer = false;
    //       AutoPowerCellMoverGroundCollect();
    //       AutoPowerCellMoverShooter();
    //     } else if (Robot.powerCellMover.getSingulatorSensor() == true
    //         && Robot.powerCellMover.getIndexerEntranceSensor() == false) {
    //       Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
    //       Robot.powerCellMover.runIndexer(0);
    //       movingFromSingulator = true;
    //       movingIntoIndexer = false;
    //       isInIdexer = false;
    //       AutoPowerCellMoverGroundCollect();
    //       AutoPowerCellMoverShooter();
    //       while (Robot.powerCellMover.getSingulatorSensor() == true
    //           && Robot.powerCellMover.getIndexerEntranceSensor() == false) {
    //         Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
    //         AutoPowerCellMoverGroundCollect();
    //         AutoPowerCellMoverShooter();
    //       }
    //       while (Robot.powerCellMover.getIndexerEntranceSensor() == false) {
    //         Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
    //         AutoPowerCellMoverGroundCollect();
    //         AutoPowerCellMoverShooter();
    //       }
    //     } else if (Robot.powerCellMover.getIndexerEntranceSensor() == true) {
    //       Robot.powerCellMover.setSingulatorSpeed(0);
    //       Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
    //       movingFromSingulator = false;
    //       movingIntoIndexer = true;
    //       isInIdexer = false;
    //       while (Robot.powerCellMover.getIndexerEntranceSensor() == true
    //           && Robot.powerCellMover.getShooterLoadedSensor() == false) {
    //         Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
    //         AutoPowerCellMoverGroundCollect();
    //         AutoPowerCellMoverShooter();
    //       }
    //       while (Robot.powerCellMover.getIndexerLoadedSensor() == true
    //           && Robot.powerCellMover.getShooterLoadedSensor() == false) {
    //         Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
    //         AutoPowerCellMoverGroundCollect();
    //         AutoPowerCellMoverShooter();
    //       }
    //       while (Robot.powerCellMover.getIndexerLoadedSensor() == false
    //           && Robot.powerCellMover.getShooterLoadedSensor() == false) {
    //         Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
    //         AutoPowerCellMoverGroundCollect();
    //         AutoPowerCellMoverShooter();
    //       }
    //       Robot.powerCellMover.runIndexer(0);
    //       AutoPowerCellMoverGroundCollect();
    //       AutoPowerCellMoverShooter();
          // } else if (Robot.powerCellMover.getSingulatorSensor() == true
          // && Robot.powerCellMover.getIndexerEntranceSensor() == true
          // && Robot.powerCellMover.getIndexerLoadedSensor() == false) {
          // Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
          // Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
          // movingFromSingulator = true;
          // movingIntoIndexer = true;
          // isInIdexer = false;
          // AutoPowerCellMoverGroundCollect();
          // AutoPowerCellMoverShooter();
          // } else if (Robot.powerCellMover.getSingulatorSensor() == false
          // && Robot.powerCellMover.getIndexerEntranceSensor() == false
          // && Robot.powerCellMover.getIndexerLoadedSensor() == true) {
          // Robot.powerCellMover.setSingulatorSpeed(0);
          // Robot.powerCellMover.runIndexer(0);
          // movingFromSingulator = false;
          // movingIntoIndexer = false;
          // isInIdexer = true;
          // AutoPowerCellMoverGroundCollect();
          // AutoPowerCellMoverShooter();
          // } else if (Robot.powerCellMover.getSingulatorSensor() == true
          // && Robot.powerCellMover.getIndexerEntranceSensor() == false
          // && Robot.powerCellMover.getIndexerLoadedSensor() == true) {
          // Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
          // Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
          // movingFromSingulator = true;
          // movingIntoIndexer = false;
          // isInIdexer = true;
          // AutoPowerCellMoverGroundCollect();
          // AutoPowerCellMoverShooter();
          // } else if (Robot.powerCellMover.getSingulatorSensor() == false
          // && Robot.powerCellMover.getIndexerEntranceSensor() == true
          // && Robot.powerCellMover.getIndexerLoadedSensor() == true) {
          // Robot.powerCellMover.setSingulatorSpeed(0);
          // Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
          // movingFromSingulator = false;
          // movingIntoIndexer = true;
          // isInIdexer = true;
          // AutoPowerCellMoverGroundCollect();
          // AutoPowerCellMoverShooter();
          // } else if (Robot.powerCellMover.getSingulatorSensor() == true
          // && Robot.powerCellMover.getIndexerEntranceSensor() == true
          // && Robot.powerCellMover.getIndexerLoadedSensor() == true) {
          // Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
          // Robot.powerCellMover.runIndexer(INDEXER_RUN_SPEED);
          // movingFromSingulator = true;
          // movingIntoIndexer = true;
          // isInIdexer = true;
          // AutoPowerCellMoverGroundCollect();
          // AutoPowerCellMoverShooter();
    //     } else {
    //       Robot.powerCellMover.setSingulatorSpeed(0);
    //       Robot.powerCellMover.runIndexer(0);
    //       movingFromSingulator = false;
    //       movingIntoIndexer = false;
    //       isInIdexer = false;
    //       AutoPowerCellMoverGroundCollect();
    //       AutoPowerCellMoverShooter();
    //     }

    //   } else if (Robot.powerCellMover.getShooterLoadedSensor() == true
    //       && Robot.powerCellMover.getSingulatorSensor() == true) {
    //     Robot.powerCellMover.setSingulatorSpeed(0.3);
    //     Robot.powerCellMover.runIndexer(0);
    //     movingFromSingulator = true;
    //     movingIntoIndexer = false;
    //     isInIdexer = false;
    //     AutoPowerCellMoverGroundCollect();
    //     AutoPowerCellMoverShooter();
    //     while (Robot.powerCellMover.getSingulatorSensor() == true
    //         && Robot.powerCellMover.getIndexerEntranceSensor() == false) {
    //       Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
    //       AutoPowerCellMoverGroundCollect();
    //       AutoPowerCellMoverShooter();
    //     }
    //     while (Robot.powerCellMover.getIndexerEntranceSensor() == false) {
    //       Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
    //       AutoPowerCellMoverGroundCollect();
    //       AutoPowerCellMoverShooter();
    //     }
    //   } else {
    //     Robot.powerCellMover.setSingulatorSpeed(0);
    //     Robot.powerCellMover.runIndexer(0);
    //     movingFromSingulator = false;
    //     movingIntoIndexer = false;
    //     isInIdexer = false;
    //     AutoPowerCellMoverGroundCollect();
    //     AutoPowerCellMoverShooter();
    //   }
    // } else if (Robot.oi.getBackwardsButtonPressed() == true) {
    //   Robot.powerCellMover.setSingulatorSpeed(-1);
    //   Robot.powerCellMover.runIndexer(-1);
    //   AutoPowerCellMoverGroundCollect();
    //   AutoPowerCellMoverShooter();
    // }
  }

  public void AutoPowerCellMoverGroundCollect() {
    if (!wasDeployedButtonPressed && Robot.oi.getGroundPickUpDeployed()) {
      Robot.powerCellMover.toggleGroundPickUpDeploy();
      // System.out.println("ground pickup has been deployed");
    }
    if (Robot.oi.getGroundPickUpRollerAxis() > 0) {
      Robot.powerCellMover.setRollerSpeed(Robot.oi.getGroundPickUpRollerAxis());
      Robot.powerCellMover.setSingulatorIntakeSpeed(Robot.oi.getGroundPickUpRollerAxis());
    } else if (Robot.oi.getHumanPlayerStationPickUpRollerAxis() > 0) {
      Robot.powerCellMover.setRollerSpeed(-Robot.oi.getHumanPlayerStationPickUpRollerAxis());
      Robot.powerCellMover.setSingulatorIntakeSpeed(Robot.oi.getHumanPlayerStationPickUpRollerAxis());
    } else if ((Robot.oi.getGroundPickUpRollerAxis() > 0 && Robot.oi.getHumanPlayerStationPickUpRollerAxis() > 0)
        || (Robot.oi.getGroundPickUpRollerAxis() < 0 && Robot.oi.getHumanPlayerStationPickUpRollerAxis() < 0)) {
      Robot.powerCellMover.setRollerSpeed(0);
      Robot.powerCellMover.setSingulatorIntakeSpeed(0);
    } else {
      Robot.powerCellMover.setRollerSpeed(0);
      Robot.powerCellMover.setSingulatorIntakeSpeed(0);
    }
  }

  public void AutoPowerCellMoverShooter() {
    if (Robot.oi.getShooterButton()) {
      Robot.powerCellMover.runShooterOpen(MANUAL_RUN_SPEED_SHOOTER);
      current_speed = MANUAL_RUN_SPEED_SHOOTER;
      if (time >= 70) {
        Robot.powerCellMover.runIndexer(1);
        Robot.powerCellMover.setSingulatorSpeed(0.4);
      } else {
        Robot.powerCellMover.runIndexer(0);
        Robot.powerCellMover.setSingulatorSpeed(0);
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