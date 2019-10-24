/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.drivebase.PathFinderCommand;
import frc.robot.components.DrivePod;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Add your docs here.
 */
public class PathFinderSystem extends Subsystem
{
  private double Encoder_Pos = DrivePod.getPositionInches();
  private double Left_To_Right_Offset_Inches;
  public int Right_Encoder_Pos = 0;
  public int Left_Encoder_Pos = 0;
  private double wheel_diameter = Constants.WHEEL_DIAMETER*0.0254;
  public PathFinderSystem(boolean LeftOrRight) {
    super();

    Waypoint[] points = new Waypoint[] {
      new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
      new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
      new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
    };
  
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);

    // Wheelbase Width = 0.5m
    TankModifier modifier = new TankModifier(trajectory).modify(0.5);

    // Do something with the new Trajectories...
    //Trajectory left = modifier.getLeftTrajectory();
    //Trajectory right = modifier.getRightTrajectory();

    EncoderFollower encoderLeft = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower encoderRight = new EncoderFollower(modifier.getRightTrajectory());

    // Determine whether the encoder position is the left or right encoder position.
    if(LeftOrRight)
    {
      Left_Encoder_Pos = (int) (Math.round(Encoder_Pos) + Left_To_Right_Offset_Inches);
      Right_Encoder_Pos = (int) (Math.round(Encoder_Pos));
    }
    else
    {
      Left_Encoder_Pos = (int) (Math.round(Encoder_Pos));
      Right_Encoder_Pos = (int) (Math.round(Encoder_Pos) + Left_To_Right_Offset_Inches);
    }

    encoderLeft.configureEncoder(Left_Encoder_Pos, 1000, wheel_diameter);
    encoderRight.configureEncoder(Right_Encoder_Pos, 1000, wheel_diameter);

    encoderLeft.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.ROBOT_TOP_SPEED_LOW_GEAR_FPS, 0);
    encoderRight.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.ROBOT_TOP_SPEED_LOW_GEAR_FPS, 0);

    double output = encoderLeft.calculate(Left_Encoder_Pos);
  }


  @Override
  protected void initDefaultCommand()
  {
    setDefaultCommand(new PathFinderCommand(0));
  }
}
