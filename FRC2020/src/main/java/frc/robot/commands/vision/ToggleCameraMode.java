/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command to set the main camera into one of its modes
 */
public class ToggleCameraMode extends Command {

    /**
     * Command the camera to enter whatever mode it is not in
     */
    public ToggleCameraMode() {
        requires(Robot.vision);
    }

    @Override
    public synchronized void start() {
        super.start();
        Robot.vision.setCameraIsHumanVisible(!Robot.vision.isCameraHumanVision());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }
}
