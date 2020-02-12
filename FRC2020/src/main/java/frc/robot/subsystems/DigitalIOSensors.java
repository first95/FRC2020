/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commands.PublishDigitalIO;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DigitalIOSensors extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DigitalInput SingulatorSensor = new DigitalInput(Constants.SINGULATOR_SENSOR);
  DigitalInput IndexerEntranceSensor = new DigitalInput(Constants.INDEXER_ENTRANCE_SENSOR);
  DigitalInput IndexerLoadedSensor = new DigitalInput(Constants.INDEXER_POWERCELL_LOADED_SENSOR);
  DigitalInput ShooterLoadedSensor = new DigitalInput(Constants.SHOOTER_LOADED_SENSOR);


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new PublishDigitalIO());
  }

public boolean getSingulatorSensor() {
  // get the current state of the sensor watching for powercells in the singulator
  return SingulatorSensor.get();
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

}
