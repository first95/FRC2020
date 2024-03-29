/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ManuallyControlPowerCellMovers extends Command {

  public static double MANUAL_RUN_SPEED_SHOOTER = 0.8;
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double current_speed = 0;

  public static double MANUAL_RUN_SPEED_INDEXER = 0.8;

  public static double spinningSpeed = 0.8;
  public static double intakeSpeed = 0.8;

  private boolean wasDeployedButtonPressed = false;

  public ManuallyControlPowerCellMovers() { 
    requires(Robot.powerCellMover);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Shooter speed setting
    if(Robot.oi.getShooterButton()) {
      Robot.powerCellMover.runShooterOpen(MANUAL_RUN_SPEED_SHOOTER);
      current_speed = MANUAL_RUN_SPEED_SHOOTER;
    } else {
      // Slowing down motor and don't want to do it too fast
      if (current_speed < MIN_RUN_SPEED) {
        current_speed = 0;
      } else {
        current_speed = current_speed*MANUAL_RUN_SPEED_SHOOTER;
      }
      Robot.powerCellMover.runShooterOpen(current_speed);
    }

    // Indexer speed setting
    if (Robot.oi.getRunIndexer()) {
      Robot.powerCellMover.runIndexer(MANUAL_RUN_SPEED_INDEXER);
    } else {
      Robot.powerCellMover.runIndexer(0);
    }

    // Singulator speed setting
    if(Robot.oi.getSingulatorButton()) {
      Robot.powerCellMover.setSingulatorSpeed(spinningSpeed);
    } else {
      Robot.powerCellMover.setSingulatorSpeed(0);
    }


    // Ground pickup deploy and speed settings
    // If the deploy button was not pressed during the last loop and is pressed during the current loop,
      // toggle deploy
    if (!wasDeployedButtonPressed && Robot.oi.getGroundPickUpDeployed())
    {
      Robot.powerCellMover.toggleGroundPickUpDeploy();
      // System.out.println("ground pickup has been deployed");
    }

    Robot.powerCellMover.setRollerSpeed(Robot.oi.getGroundPickUpRollerAxis());

    SmartDashboard.putBoolean("SingulatorOccupied",Robot.powerCellMover.getSingulatorSensor());
    SmartDashboard.putBoolean("IndexerEntranceOccupied",Robot.powerCellMover.getIndexerEntranceSensor());
    SmartDashboard.putBoolean("IndexerPosition1Occupied",Robot.powerCellMover.getIndexerLoadedSensor());
    SmartDashboard.putBoolean("ShooterLoaded",Robot.powerCellMover.getShooterLoadedSensor());

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
