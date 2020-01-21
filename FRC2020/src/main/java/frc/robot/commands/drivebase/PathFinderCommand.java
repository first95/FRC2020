/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.components.DrivePod;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class PathFinderCommand extends Command {
  // private double Encoder_Pos = DrivePod.getPositionInches();
  // private double Left_To_Right_Offset_Inches;
  // private double Top_Speed;
  // public int Right_Encoder_Pos = 0;
  // public int Left_Encoder_Pos = 0;
  // private double wheel_diameter = Constants.WHEEL_DIAMETER*0.0254;
  // public static double outputLeft, outputRight;
  // public static int whatPath;
  // public static int spin = 0;

  // // All of these are the possible path values
  // public static int ForwardTenFeet = 1;

  // private static boolean LeftOrRight;
  // private static boolean WhatGearAreWeIn;

  // public static EncoderFollower leftEncFollower;
  // public static EncoderFollower rightEncFollower;

  // public static Waypoint[] points;

  // public PathFinderCommand(boolean RightOrLeft, boolean WhichGearAreWeIn, double a, int whatPath) {
  //   //requires(Robot.drivebase);
  //   //requires(Robot.pathfinder);
  //   System.out.println("We are in PathfinderCommand");
  //   this.setInterruptible(false);
  //   //System.out.println("we are in the constructor");
  //   this.LeftOrRight = RightOrLeft;
  //   this.WhatGearAreWeIn = WhichGearAreWeIn;
  //   this.Left_To_Right_Offset_Inches = a;
  //   this.whatPath = whatPath;
  // }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  //   System.out.println("we are in initialize");

  //   if (whatPath == ForwardTenFeet)
  //   {
  //     points = new Waypoint[] {
  //       new Waypoint(0, 0, 0),
  //       new Waypoint(10, 0, 0)
  //     };
    }
   

  //   Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
  //   Trajectory trajectory = Pathfinder.generate(points, config);

  //   // Wheelbase Width = 0.5m
  //   TankModifier modifier = new TankModifier(trajectory).modify(0.5);

  //   // Do something with the new Trajectories...
  //   //Trajectory left = modifier.getLeftTrajectory();
  //   //Trajectory right = modifier.getRightTrajectory();

  //   leftEncFollower = new EncoderFollower(modifier.getLeftTrajectory());
  //   rightEncFollower = new EncoderFollower(modifier.getRightTrajectory());

  //   // Determine whether the encoder position is the left or right encoder position.
  //   if(LeftOrRight)
  //   {
  //     Left_Encoder_Pos = (int) (Math.round(Encoder_Pos) + Left_To_Right_Offset_Inches);
  //     Right_Encoder_Pos = (int) (Math.round(Encoder_Pos));
  //   }
  //   else
  //   {
  //     Left_Encoder_Pos = (int) (Math.round(Encoder_Pos));
  //     Right_Encoder_Pos = (int) (Math.round(Encoder_Pos) + Left_To_Right_Offset_Inches);
  //   }

  //   leftEncFollower.configureEncoder(Left_Encoder_Pos, 1000, wheel_diameter);
  //   rightEncFollower.configureEncoder(Right_Encoder_Pos, 1000, wheel_diameter);

  //   // Determine what gear we are in, then make the top speed here equal to the top speed for the gear.
  //   if(WhatGearAreWeIn)
  //   {
  //     Top_Speed = Constants.ROBOT_TOP_SPEED_HIGH_GEAR_FPS;
  //   }
  //   else
  //   {
  //     Top_Speed = Constants.ROBOT_TOP_SPEED_LOW_GEAR_FPS;
  //   }

  //   leftEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Top_Speed, 0);
  //   rightEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Top_Speed, 0);
  // }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // outputLeft = leftEncFollower.calculate(Left_Encoder_Pos);
    // outputRight = rightEncFollower.calculate(Right_Encoder_Pos);

    // System.out.println("we are in execute");
    // // Robot.drivebase.arcade(outputLeft, spin);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.println("are we finished?");
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("have we ended?");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("are we interrupted?");
  }
}
