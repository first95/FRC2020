/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.PowerCellMoverCommand;

/**
 * Add your docs here.
 */
public class PowerCellMover extends Subsystem {

  // DigitalInput SingulatorSensor = new DigitalInput(Constants.SINGULATOR_SENSOR);
  // DigitalInput IndexerEntranceSensor = new DigitalInput(Constants.INDEXER_ENTRANCE_SENSOR);
  // DigitalInput IndexerLoadedSensor = new DigitalInput(Constants.INDEXER_POWERCELL_LOADED_SENSOR);
  // DigitalInput ShooterLoadedSensor = new DigitalInput(Constants.SHOOTER_LOADED_SENSOR);

  private CANSparkMax beltMotor;
  private IMotorControllerEnhanced Singulator, SingulatorIntake;
  private TalonSRX rollers;
  private Solenoid deploy;

  public PowerCellMover() {
    super();

    // SparkMax initialization
    beltMotor = new CANSparkMax(Constants.INDEXER_BELT_MOTOR_ID, MotorType.kBrushless);

    // Talon initialization
    Singulator = new TalonSRX(Constants.INNER_SINGULATOR_TALON_ID);
    SingulatorIntake = new TalonSRX(Constants.SINGULATOR_INTAKE_TALON_ID);
    rollers = new TalonSRX(Constants.GROUND_PICK_UP_TALON_ID);

    // Solenoid initialization
    deploy = new Solenoid(Constants.GROUND_PICK_UP_SOLENOID_ID);
  }

  // public boolean getSingulatorSensor() {
  //   // get the current state of the sensor watching for powercells in the singulator
  //   return SingulatorSensor.get();
  // }
  
  // public boolean getIndexerEntranceSensor() {
  //   // get the current state of the sensor watching for powercells where the indexer can first grab them
  //   return IndexerEntranceSensor.get();
  // }
  
  // public boolean getIndexerLoadedSensor() {
  //   // get the current state of the sensor watching for powercells in the first position fully within the indexer
  //   return IndexerLoadedSensor.get();
  // }
  
  // public boolean getShooterLoadedSensor() {
  //   // get the current state of the sensor watching for powercells in position to be fired
  //   return ShooterLoadedSensor.get();
  // }

  /**
     * Run the indexer belt
     * @param speed 0 for stationary, 1 for full forward
     */
    public void runIndexer(double speed) {
     // System.out.println("Setting indexer speed to " + speed);
      beltMotor.set(speed);
  }

  /**
   *  Set the speed at which the singulator rollers rotate
   */
  public void setSingulatorSpeed(double Speed) {
    Singulator.set(ControlMode.PercentOutput, Speed);
    
  }

  /** 
  Set the speed at which the roller rotates at
  */
  public void setRollerSpeed(double speed)
  {
      rollers.set(ControlMode.PercentOutput, speed);
      SingulatorIntake.set(ControlMode.PercentOutput, -speed);
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

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PowerCellMoverCommand());
  }
}