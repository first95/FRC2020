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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
  public CANPIDController FollowerPIDController, LeaderPIDController;
  private CANEncoder FollowerEncoder, LeaderEncoder;
  private IMotorControllerEnhanced Singulator, SingulatorIntake;
  private TalonSRX rollers;
  private TalonSRX greenRingLight;
  private Solenoid deploy;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

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

    greenRingLight = new TalonSRX(Constants.TARGET_CAM_GREEN_RINGLIGHT_TALON_ID);

    // Solenoid initialization
    deploy = new Solenoid(Constants.GROUND_PICK_UP_SOLENOID_ID);

    // init();

    follower.follow(leader, true);

    LeaderPIDController = leader.getPIDController();
    FollowerPIDController = follower.getPIDController();

    LeaderEncoder = leader.getEncoder();
    FollowerEncoder = follower.getEncoder();

    kP = 0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 6000;

    System.out.println("10");

    LeaderPIDController.setP(kP);
    // FollowerPIDController.setP(kP);
    LeaderPIDController.setI(kI);
    // FollowerPIDController.setI(kI);
    LeaderPIDController.setD(kD);
    // FollowerPIDController.setP(kD);
    LeaderPIDController.setIZone(kIz);
    // FollowerPIDController.setIZone(kIz);
    LeaderPIDController.setFF(kFF);
    // FollowerPIDController.setFF(kFF);
    LeaderPIDController.setOutputRange(kMinOutput, kMaxOutput);
    // FollowerPIDController.setOutputRange(kMinOutput, kMaxOutput);
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
  
   /**
     * Run the shooter
     * @param speed 0 for stationary, 1 for full forward, -1 for full reverse
     */
    public void runShooterOpen(double speed) {
      //System.out.println("Setting shooter speed to " + speed);
      leader.set(speed);
      follower.set(-1 * speed);
  }

  public void runShooterClosed(double setPoint, ControlType controlType)
  {
    LeaderPIDController.setReference(setPoint, controlType);
    // FollowerPIDController.setReference(setPoint, controlType);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Leader velocity", LeaderEncoder.getVelocity());
    SmartDashboard.putNumber("Follower velocity", FollowerEncoder.getVelocity());
  }

  public double getShooterVelocity()
  {
    return LeaderEncoder.getVelocity();
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
   * Set green ring light state
   * @param PctOutput
   */
  public void setGreenRingLightOutput( double PctOutput) {
    greenRingLight.set(ControlMode.PercentOutput, PctOutput);
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
