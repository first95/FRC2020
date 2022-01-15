/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class FollowTrajectory extends SequentialCommandGroup {
  public FollowTrajectory() {
    addRequirements(Robot.drivebase);
  
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA), Constants.DRIVE_KINEMATICS, 10);
    //Generate trajectory config
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.MAX_SPEED_MPS, Constants.MAX_ACCELERATION_MPSPS)
        .setKinematics(Constants.DRIVE_KINEMATICS)
        .addConstraint(autoVoltageConstraint);
    
    //Generate a trajectory (replace with import)
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(2, 1, new Rotation2d(-90)), 
      config);

    //Create Ramsete follower:
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        Robot.drivebase::getPose,
        new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA),
        Constants.DRIVE_KINEMATICS,
        Robot.drivebase::getWheelSpeeds,
        new PIDController(Constants.KP, 0, 0),
        new PIDController(Constants.KP, 0, 0),
        Robot.drivebase::tankDriveVolts,
        Robot.drivebase);

    // Set robot starting position:
    Robot.drivebase.resetOdometry(exampleTrajectory.getInitialPose());

    addCommands(ramseteCommand);
  }

}
