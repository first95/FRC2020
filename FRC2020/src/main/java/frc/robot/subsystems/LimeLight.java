/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Gets numbers from and allows for control of the limelight
 */
public class LimeLight extends Subsystem {
  private final NetworkTable limelight_target_data;
  private double tv, tx, ty, distance, floorDistance, tvert;

  public LimeLight(String hostname) {
    limelight_target_data = NetworkTableInstance.getDefault().getTable("limelight-"+ hostname);
  }

  @Override
  public void periodic() {
    tv = limelight_target_data.getEntry("tv").getDouble(0.0);
    tx = limelight_target_data.getEntry("tx").getDouble(0.0);
    ty = limelight_target_data.getEntry("ty").getDouble(0.0);
    tvert = limelight_target_data.getEntry("tvert").getDouble(0.0);

    distance = (Constants.TARGET_TALLNESS_INCHES / 2) / Math.tan(Math.toRadians(tvert * Constants.DEGREES_PER_PIXEL) / 2);
    floorDistance = Math.sqrt(Math.pow(distance, 2) - Math.pow(Constants.HEIGHT_DIFFERENCE, 2));

    SmartDashboard.putNumber("Bearing", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("Target Valid?", tv);
    SmartDashboard.putNumber("Range (in)", distance);
    SmartDashboard.putNumber("Horiz. Range", floorDistance);
  }

  public double getTX() {
    return tx;
  }
  public double getTY() {
    return ty;
  }
  public double getTV() {
    return tv;
  }
  public double getDistanceToTarg() {
    return distance;
  }
  public double getFloorDistanceToTarg() {
    return floorDistance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
