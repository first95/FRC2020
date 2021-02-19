/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.PowerCellMover;

public class AutoPowerCellMover extends Command {

  public static boolean shooterIsLoadedCheck;
  public static boolean movingFromSingulator;
  public static boolean movingIntoIndexer;
  public static boolean isInIdexer;

  public static boolean shooterSensor;
  public static boolean indexerLoadedSensor;
  public static boolean indexerEntranceSensor;
  public static boolean singulatorSensor;

  public static boolean wasIndexerLoadedSensorTrippedLastIteration;
  public static boolean wasSingulatorSensorTrippedLastIteration;

  private boolean wasDeployedButtonPressed = false;

  public static boolean dummy = false;

  public int idx2singulatorDelayCount = 0;
  public int spinupDelayCount = 0;
  public boolean shooterSpunUp = false;
  public static double INDEXER_RUN_SPEED = 0.5;
  public static double INDEXER_SHOOTING_RUN_SPEED = 1;

  public static double SINGULATOR_RUN_SPEED = 0.5;

  public static double MANUAL_RUN_SPEED_SHOOTER = 0.5;
  public static double TARGET_RUN_SPEED_SHOOTER = 2100; // ideal speed in RPM
  public static double RUN_TOLERANCE_SHOOTER = 50; // tolerance range for shooter speed
  public static double MAINTAIN_RUN_SPEED_SHOOTER = TARGET_RUN_SPEED_SHOOTER * Constants.RPM_TO_SHOOTER_POWER_CONVERSION; // want this to roughly hold target RPM
  public static double SLOW_RUN_SPEED_SHOOTER = MAINTAIN_RUN_SPEED_SHOOTER - 0.04; // want this to slow down a bit but not fully
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double actual_speed = 0;
  private double current_speed = 0;
  private double speedError = 0;
  private double speedErrorPercent = 0;
  private double lastSpeedErrorPercent = 0;
  private double speedProportional, speedIntegral, speedDerivative;
  private double correction = 0;
  private double cappedCorrection = 0;
  private double shooterkp = 3;
  private double shooterki = 0.00002;
  private double shooterkd = 16;
  

  public enum State {
    IDLE, SINGULATOR, INDEXER_ENTRANCE, INDEXER_LOADED_A, INDEXER_LOADED_B
  }

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
    shooterSensor = Robot.powerCellMover.getShooterLoadedSensor();
    indexerLoadedSensor = Robot.powerCellMover.getIndexerLoadedSensor();
    indexerEntranceSensor = Robot.powerCellMover.getIndexerEntranceSensor();
    singulatorSensor = Robot.powerCellMover.getSingulatorSensor();

    if (Robot.oi.getBackwardsButtonPressed() == false) {
      if (shooterSensor == false) {
        if (indexerLoadedSensor == false && indexerEntranceSensor == false) {
          if (singulatorSensor == true && wasSingulatorSensorTrippedLastIteration == false) {
            Switch(State.SINGULATOR);
            AutoPowerCellMoverGroundCollect();
            AutoPowerCellMoverShooter();
            wasSingulatorSensorTrippedLastIteration = true;
          } else if (singulatorSensor == true && wasSingulatorSensorTrippedLastIteration == true) {
            Switch(State.SINGULATOR);
            AutoPowerCellMoverGroundCollect();
            AutoPowerCellMoverShooter();
          } else if (singulatorSensor == false && wasSingulatorSensorTrippedLastIteration == true) {
            Switch(State.SINGULATOR);
            AutoPowerCellMoverGroundCollect();
            AutoPowerCellMoverShooter();
            wasSingulatorSensorTrippedLastIteration = false;
          } else {
            Switch(State.IDLE);
            AutoPowerCellMoverGroundCollect();
            AutoPowerCellMoverShooter();
          }
        } else if (indexerLoadedSensor == false && indexerEntranceSensor == true) {
          Switch(State.INDEXER_ENTRANCE);
          AutoPowerCellMoverGroundCollect();
          AutoPowerCellMoverShooter();
        } else if (indexerLoadedSensor == true && wasIndexerLoadedSensorTrippedLastIteration == false) {
          Switch(State.INDEXER_LOADED_A);
          AutoPowerCellMoverGroundCollect();
          AutoPowerCellMoverShooter();
          wasIndexerLoadedSensorTrippedLastIteration = true;
        } else if (indexerLoadedSensor == true && wasIndexerLoadedSensorTrippedLastIteration == true) {
          Switch(State.INDEXER_LOADED_A);
          AutoPowerCellMoverGroundCollect();
          AutoPowerCellMoverShooter();
        } else if (indexerLoadedSensor == false && wasIndexerLoadedSensorTrippedLastIteration == true) {
          Switch(State.INDEXER_LOADED_B);
          AutoPowerCellMoverGroundCollect();
          AutoPowerCellMoverShooter();
          wasIndexerLoadedSensorTrippedLastIteration = false;
        }
      } else if (shooterSensor == true && singulatorSensor == true) {
        Switch(State.SINGULATOR);
        AutoPowerCellMoverGroundCollect();
        AutoPowerCellMoverShooter();
      } else {
        Switch(State.IDLE);
        AutoPowerCellMoverGroundCollect();
        AutoPowerCellMoverShooter();
      }
    } else if (Robot.oi.getBackwardsButtonPressed() == true) {
      Robot.powerCellMover.setSingulatorSpeed(-1);
      Robot.powerCellMover.runIndexer(-0.8);
      AutoPowerCellMoverGroundCollect();
      AutoPowerCellMoverShooter();
    }
    
  }

  public void Switch(State whatState) {
    double setIndexer = 0;
    double runSingulator = 0;
    switch (whatState) {
    case IDLE:
      runSingulator = 0;
      setIndexer = 0;
      break;

    case SINGULATOR:
      runSingulator = SINGULATOR_RUN_SPEED;
      setIndexer = 0;
      break;

    case INDEXER_ENTRANCE:
      runSingulator = 0;
      setIndexer = INDEXER_RUN_SPEED;
      break;

    case INDEXER_LOADED_A:
      runSingulator = 0;
      setIndexer = INDEXER_RUN_SPEED;
      break;

    case INDEXER_LOADED_B:
      runSingulator = 0;
      setIndexer = 0;
      break;

    default:
      runSingulator = 0;
      setIndexer = 0;
      break;
    }
    Robot.powerCellMover.setSingulatorSpeed(runSingulator);
    Robot.powerCellMover.runIndexer(setIndexer);
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
      shooterkp = SmartDashboard.getNumber("Shooter kp", 3);
      shooterki = SmartDashboard.getNumber("Shooter ki", 0.00002);
      shooterkd = SmartDashboard.getNumber("Shooter kd", 16);
      // Get actual speed
      actual_speed = Robot.powerCellMover.getShooterSpeed();
      SmartDashboard.putNumber("ProcessVariable", actual_speed);
      /*if (actual_speed < TARGET_RUN_SPEED_SHOOTER - RUN_TOLERANCE_SHOOTER) {
        current_speed = 1.0; // speed up as quickly as possible
      } else if (actual_speed < TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER) {
        current_speed = MAINTAIN_RUN_SPEED_SHOOTER;
      } else {
        // implies actual_speed >= TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER
        current_speed = SLOW_RUN_SPEED_SHOOTER;
      }*/
      if (OI.auto_shooting) {
        TARGET_RUN_SPEED_SHOOTER = OI.auto_shooting_speed;
        MAINTAIN_RUN_SPEED_SHOOTER = TARGET_RUN_SPEED_SHOOTER * Constants.RPM_TO_SHOOTER_POWER_CONVERSION;
      }
      speedError = TARGET_RUN_SPEED_SHOOTER - actual_speed;
      speedErrorPercent = speedError / TARGET_RUN_SPEED_SHOOTER;
      speedProportional = speedErrorPercent;
      speedIntegral = speedIntegral + speedErrorPercent;
      speedDerivative = speedErrorPercent - lastSpeedErrorPercent;
      correction = (speedProportional * shooterkp) + (speedIntegral * shooterki) + (speedDerivative * shooterkd) + MAINTAIN_RUN_SPEED_SHOOTER;
      cappedCorrection = Math.max(Math.min(correction, 1.0), SLOW_RUN_SPEED_SHOOTER);
      Robot.powerCellMover.runShooterOpen(cappedCorrection);
      lastSpeedErrorPercent = speedErrorPercent;

      if (spinupDelayCount > 35 ) {
        Robot.powerCellMover.runIndexer(INDEXER_SHOOTING_RUN_SPEED);
        shooterSpunUp = true;
        if (idx2singulatorDelayCount >= 5) {
           Robot.powerCellMover.setSingulatorSpeed(SINGULATOR_RUN_SPEED);
        } else {
          idx2singulatorDelayCount++;
        }
      } else {
        Robot.powerCellMover.runIndexer(0);
        Robot.powerCellMover.setSingulatorSpeed(0);
        spinupDelayCount++;
      }
    } else {
      idx2singulatorDelayCount = 0;
      spinupDelayCount = 0;
      shooterSpunUp = false;
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
