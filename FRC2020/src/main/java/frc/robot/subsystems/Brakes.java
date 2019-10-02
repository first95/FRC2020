/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * Brake system controlled by a solenoid. Not to be confused with built-in braking
 * mode in talons for drive motors.
 */
import frc.robot.commands.brakes.BrakesCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Brakes extends Subsystem {

  //Solenoid for the brakes
  private Solenoid solB;
  public Brakes(boolean thisIsABoolean) {
    super();

    if(thisIsABoolean == true) {
      solB = new Solenoid(Constants.BRAKES_SOL);
    } else if(thisIsABoolean == false) {
      solB = null;
    }
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new BrakesCommand());
  }

  public void log(){
    SmartDashboard.putBoolean("Brake status", solB.get());
  }

  public void setBrakes(boolean deploy) {
    if(solB != null) {
      solB.set(deploy);
    }
  }

}