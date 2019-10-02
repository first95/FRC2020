/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * Add your docs here.
 */
import frc.robot.commands.climber.ClimberHabLevelTwoCommand;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class ClimberHabLevelTwo extends Subsystem {

  //Solenoid (1) from the original climber
  private Solenoid solR;
  //Solenoid (5) for the new climber
  private Solenoid solF;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public ClimberHabLevelTwo(boolean thisIsABoolean) {
    super();

    if(thisIsABoolean == true) {
      solR = new Solenoid(Constants.CLIMBER_SOL_REAR);
      solF = new Solenoid(Constants.CLIMBER_SOL_FRONT);
    } else if(thisIsABoolean == false) {
      solR = null;
      solF = null;
    }
  }

  @Override
  public void initDefaultCommand() {
  // Set the default command for a subsystem here.
  setDefaultCommand(new ClimberHabLevelTwoCommand());
  }

  public void log(){
  }

  public void frontClimber(boolean deployFront) {
    if(solF != null) {
      solF.set(deployFront);
    }
  }

  public void toggleFrontClimber()
  {
    if(solF != null) {
      solF.set(!solF.get());
    }
  }

  public void rearClimber(boolean deployRear) {
    if(solR != null) {
      solR.set(deployRear);
    }
  }

  public void toggleRearClimber()
  {
    if(solR != null) {
      solR.set(!solR.get());
    }
  }
}
