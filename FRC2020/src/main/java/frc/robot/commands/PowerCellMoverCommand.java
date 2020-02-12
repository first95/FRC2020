/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PowerCellMoverCommand extends Command {

  public static double MANUAL_RUN_SPEED = 0.8;

  public static double spinningSpeed = 0.8;
  public static double intakeSpeed = 0.8;

  private boolean wasDeployedButtonPressed = false;

  public PowerCellMoverCommand() { 
    requires(Robot.powerCellMover);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Indexer speed setting
    if (Robot.oi.getRunIndexer()) {
      Robot.powerCellMover.runIndexer(MANUAL_RUN_SPEED);
    } else {
      Robot.powerCellMover.runIndexer(0);
    }

    // Singulator speed setting
    if(Robot.oi.getSingulatorButton()) {
      Robot.powerCellMover.setSingulatorSpeed(spinningSpeed, -intakeSpeed);
    } else {
      Robot.powerCellMover.setSingulatorSpeed(0, 0);
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
