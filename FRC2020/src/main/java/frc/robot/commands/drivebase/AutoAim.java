/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI.Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoAim extends Command {

  private double headingLastError = 0;
  private double headingIntegral = 0;
  private double rangeLastError = 0;
  private double rangeIntegral = 0;
  private double desiredDistance = 81;

  public AutoAim(double desiredDistance) {
    requires(Robot.drivebase);
    requires(Robot.limelight);
    this.desiredDistance = desiredDistance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double headingLeft = 0;
    double headingRight = 0;
    double rangeLeft = 0;
    double rangeRight = 0;
    double left = 0;
    double right = 0;
    double headingErrorPercent = 0;
    double rangeErrorPercent = 0;

    boolean headingOnTarget = false;
    boolean rangeOnTarget = false;
    boolean onTarget = false;

    double headingError = Robot.limelight.getTX();
    double rangeError = Robot.limelight.getFloorDistanceToTarg() - desiredDistance;
    double targetValid = Robot.limelight.getTV();

    double headingProportional = 0;
    double headingDerivitive = 0;
    double headingRawCorrection = 0;
    double headingkp = SmartDashboard.getNumber("Vision heading Kp", 1);
    double headingki = SmartDashboard.getNumber("Vision heading Ki", 0);
    double headingkd = SmartDashboard.getNumber("Vision heading Kd", 0);

    double rangeProportional = 0;
    double rangeDerivitive = 0;
    double rangeRawCorrection = 0;
    double rangekp = SmartDashboard.getNumber("Vision range Kp", 1);
    double rangeki = SmartDashboard.getNumber("Vision range Ki", 0);
    double rangekd = SmartDashboard.getNumber("Vision range Kd", 0);

    if (targetValid == 1) {
      if (Math.abs(headingError) > Constants.VISION_HEADING_TOLERANCE_DEG) {
        headingErrorPercent = (headingError / Constants.VISION_CAM_FOV_X_DEG);
        headingProportional = headingErrorPercent;
        headingIntegral = headingErrorPercent + headingIntegral;
        headingDerivitive = headingErrorPercent - headingLastError;
        headingRawCorrection = Math.max(Math.min((headingProportional * headingkp) + (headingIntegral * headingki) + (headingDerivitive * headingkd), Constants.VISION_HEADING_MAX_SPEED_PERCENT), -Constants.VISION_HEADING_MAX_SPEED_PERCENT);
        if (Math.abs(headingRawCorrection) < Constants.VISION_HEADING_MIN_SPEED_PERCENT) {
          headingRight = Math.copySign(Constants.VISION_HEADING_MIN_SPEED_PERCENT, headingRawCorrection);
        }
        else {
          headingRight = headingRawCorrection;
        }
        headingLeft = -headingRight;
        headingOnTarget = false;
      }
      else {
        headingLeft = 0;
        headingRight = 0;
        headingOnTarget = true;
      }
    }
    else {
      Robot.oi.Rumble(Controller.DRIVER, RumbleType.kLeftRumble, 1.0, 0.5);
    }

    left = headingLeft + rangeLeft;
    right = headingRight + rangeRight;
    headingLastError = headingErrorPercent;
    rangeLastError = rangeErrorPercent; 
    Robot.drivebase.driveWithTankControls(left, right);
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
