/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

import frc.robot.Robot;
import frc.robot.components.StartPosition;

/**
 * Add your docs here.
 */
public class StartPositionSetter
{
    boolean setPosition = true;

    public void StartPositionSetter()
    {
        if(setPosition)
        {
            Robot.startPosition = StartPosition.HAB_1_LEFT;
        }
        else {
            Robot.startPosition = StartPosition.UNKNOWN;
        }
    }
}
