/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.ManuallyControlClimber;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {
    
    private Solenoid deploy;

    public Climber() {
        super();
        
        deploy = new Solenoid(Constants.CLIMBER_SOLENOID_NUM);

        // System.out.println("hardware is set");
    }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManuallyControlClimber());
  }
    
  /**  
  If climber is deployed, retract it; else, deploy it 
  */
  public void toggleClimberDeploy()
    {
        deploy.set(!deploy.get());
        System.out.println("climber deploy has been toggled");
    }
}
