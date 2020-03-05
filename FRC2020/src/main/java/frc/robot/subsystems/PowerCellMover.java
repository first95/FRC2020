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
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.ManuallyControlPowerCellMovers;
import frc.robot.commands.AutoPowerCellMover;


/**
 * Add your docs here.
 */
public class PowerCellMover extends Subsystem {

  DigitalInput SingulatorSensor = new DigitalInput(Constants.SINGULATOR_SENSOR);
  DigitalInput IndexerEntranceSensor = new DigitalInput(Constants.INDEXER_ENTRANCE_SENSOR);
  DigitalInput IndexerLoadedSensor = new DigitalInput(Constants.INDEXER_POWERCELL_LOADED_SENSOR);
  DigitalInput ShooterLoadedSensor = new DigitalInput(Constants.SHOOTER_LOADED_SENSOR);

  private CANSparkMax beltMotor, leader, follower;
  public static CANPIDController shooterPIDController;
  private CANEncoder leaderEncoder;
  private IMotorControllerEnhanced Singulator, SingulatorIntake;
  private TalonSRX rollers;
  public Solenoid deploy;
  private static double kP, kI, kD, kIz, kFF, minOutput, maxOutput, maxRPM;

  public PowerCellMover() {
    super();

    // SparkMax initialization
    beltMotor = new CANSparkMax(Constants.INDEXER_BELT_MOTOR_ID, MotorType.kBrushless);
    leader = new CANSparkMax(Constants.LEADER_SHOOT, MotorType.kBrushless);
    follower = new CANSparkMax(Constants.FOLLOWER_SHOOT, MotorType.kBrushless);

    // Talon initialization
    Singulator = new TalonSRX(Constants.INNER_SINGULATOR_TALON_ID);
    SingulatorIntake = new TalonSRX(Constants.SINGULATOR_INTAKE_TALON_ID);
    rollers = new TalonSRX(Constants.GROUND_PICK_UP_TALON_ID);


    // Solenoid initialization
    deploy = new Solenoid(Constants.GROUND_PICK_UP_SOLENOID_ID);

    follower.follow(leader);

    shooterPIDController = leader.getPIDController();

    leaderEncoder = leader.getEncoder();

    // Initialize PID contstants
    kP = 0.0002;
    kI = 0;
    kD = 0;
    kIz = 0;
    // kFF = 0;
    maxOutput = 1;
    minOutput = -1;
    maxRPM = 2000;

    // set PID coefficients
    shooterPIDController.setP(kP, 0);
    System.out.println("p has been set");
    // shooterPIDController.setI(kI, 0);
    // shooterPIDController.setD(kD, 0);
    // shooterPIDController.setIZone(kIz, 0);
    // shooterPIDController.setFF(kFF, 0);
    // shooterPIDController.setOutputRange(minOutput, maxOutput, 0);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", maxOutput);
    SmartDashboard.putNumber("Min Output", minOutput);

    // init();
  }

   public boolean getSingulatorSensor() {
     // get the current state of the sensor watching for powercells in the singulator
     return (! SingulatorSensor.get());
   }
  
   public boolean getIndexerEntranceSensor() {
     // get the current state of the sensor watching for powercells where the indexer can first grab them
     return IndexerEntranceSensor.get();
   }
  
   public boolean getIndexerLoadedSensor() {
     // get the current state of the sensor watching for powercells in the first position fully within the indexer
     return IndexerLoadedSensor.get();
   }
  
   public boolean getShooterLoadedSensor() {
     // get the current state of the sensor watching for powercells in position to be fired
     return ShooterLoadedSensor.get();
   }

  private void init() {
		// leader.restoreFactoryDefaults();
    // follower.restoreFactoryDefaults();
  }

  public static double p()
  {
    return kP;
  }
  public static double i()
  {
    return kI;
  }
  public static double d()
  {
    return kD;
  }
  public static double iZ()
  {
    return kIz;
  }
  public static double fF()
  {
    return kFF;
  }
  public static double min()
  {
    return minOutput;
  }
  public static double max()
  {
    return maxOutput;
  }
  public static double RPM()
  {
    return maxRPM;
  }
  

  public void runShooterClosed(double setPoint, ControlType type)
  {
    shooterPIDController.setReference(setPoint, type);
    SmartDashboard.putNumber("ProcessVariable", leaderEncoder.getVelocity());
  }
   /**
     * Run the shooter
     * @param speed 0 for stationary, 1 for full forward, -1 for full reverse
     */
    public void runShooterOpen(double speed) {
      //System.out.println("Setting shooter speed to " + speed);
      leader.set(speed);
      follower.set(-1 * speed);
  }

   /**
     * Get speed from shooter (RPM)
     */
    public double getShooterSpeed() {
      return leaderEncoder.getVelocity();
  }

  /**
     * Run the indexer belt
     * @param speed 0 for stationary, 1 for full forward
     */
    public void runIndexer(double speed) {
     // System.out.println("Setting indexer speed to " + speed);
      beltMotor.set(speed);
      beltMotor.setIdleMode(IdleMode.kBrake);
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
      // System.out.println("rollers have a command");
  }
  public void setSingulatorIntakeSpeed(double speed) {
    SingulatorIntake.set(ControlMode.PercentOutput, -speed);
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
    setDefaultCommand(new AutoPowerCellMover());
  }
}
