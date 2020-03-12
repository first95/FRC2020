/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.GroundPickUpCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class GroundPickUp extends Subsystem {
    
    private TalonSRX rollers;
    private Solenoid deploy;

    public GroundPickUp() {
        super();
        
        rollers = new TalonSRX(Constants.GROUND_PICK_UP_TALON_ID);
        deploy = new Solenoid(Constants.GROUND_PICK_UP_SOLENOID_ID);

        // System.out.println("hardware is set");
    }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GroundPickUpCommand());
  }

  /** 
  Set the speed at which the roller rotates at
  */
  public void setRollerSpeed(double speed)
  {
      rollers.set(ControlMode.PercentOutput, speed);
      // System.out.println("rollers have a command");
  }
    
  /**  
  If ground pick-up is deployed, retract it; else, deploy it 
  */
  public void toggleGroundPickUpDeploy()
    {
        deploy.set(!deploy.get());
        // System.out.println("it has been deployed");
    }
}
