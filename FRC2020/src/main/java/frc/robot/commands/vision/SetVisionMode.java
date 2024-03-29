/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.VisionProcessor;

/**
 * Command to set the main camera into one of its modes
 */
public class SetVisionMode extends Command {
    private VisionProcessor.VisionMode targetMode;

    /**
     * Command the camera to enter a mode
     * 
     * @param isHumanVisible true if the camera should be configured for human use,
     *                       or false to configure the camera for machine vision.
     */
    public SetVisionMode(VisionProcessor.VisionMode mode) {
        targetMode = mode;
        requires(Robot.vision);
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.vision.SetMode(targetMode);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }
}
