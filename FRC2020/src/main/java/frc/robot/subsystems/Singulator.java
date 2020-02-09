/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
// import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.SingulatorCommand;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Singulator extends Subsystem {

  private IMotorControllerEnhanced Singulator;
  private IMotorControllerEnhanced SingulatorIntake;

  public Singulator() {
    super();

    Singulator = new TalonSRX(Constants.INNER_SINGULATOR_TALON_ID);
    SingulatorIntake = new TalonSRX(Constants.SINGULATOR_INTAKE_TALON_ID);
  }

  public double getSingulatorCurrentSpike() {
    return Singulator.getOutputCurrent();
  }

  public void setSingulatorSpeed(double Speed) {
    Singulator.set(ControlMode.PercentOutput, Speed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new SingulatorCommand());
  }
}
