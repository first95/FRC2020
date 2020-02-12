/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.ManuallyControlIndexer;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Indexer extends Subsystem {
    private CANSparkMax beltMotor;

    public Indexer() {
        beltMotor = new CANSparkMax(Constants.INDEXER_BELT_MOTOR_ID, MotorType.kBrushless);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ManuallyControlIndexer());
    }

    /**
     * Run the indexer belt
     * @param speed 0 for stationary, 1 for full forward
     */
    public void runIndexer(double speed) {
        beltMotor.set(speed);
    }
}
