/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class GroundPickUpCommand extends Command {
    // Stores whether or not the deploy button was pressed during the last loop
    private boolean wasDeployedButtonPressed = false;
    
    public GroundPickUpCommand() {
        requires(Robot.groundPickUp);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      // If the deploy button was not pressed during the last loop and is pressed during the current loop,
      // toggle deploy
      if (!wasDeployedButtonPressed && Robot.oi.getGroundPickUpDeployed())
      {
            Robot.groundPickUp.toggleGroundPickUpDeploy();
      }

      Robot.groundPickUp.setRollerSpeed(Robot.oi.getGroundPickUpRollerAxis());
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
