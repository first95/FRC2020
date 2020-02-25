/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.LinkedList;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.vision.SetCameraMode;

/**
 * A subsystem to read from the camera(s), process them, and output video to the smart dashboard
 */
public class VisionProcessor extends Subsystem {
    /** The camera aimed at the upper port */
    UsbCamera upperPortCam;
    /** The view exposed to the drivers for them to see through */
    MjpegServer fpsViewServer;
    /** The camera aimed upward at the climber/control panel */
    UsbCamera lookupCam;
    /** The view exposed to the drivers for them to see through */
    MjpegServer lookupServer;

    private double[] bearingsList = null;
    private double[] rangesList = null;

    public VisionProcessor() {
        super();
        // /dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0
        upperPortCam = new UsbCamera("Upper port cam", "/dev/video0");
        upperPortCam.setResolution(800, 600);
        fpsViewServer = new MjpegServer("First person view", 1181);
        fpsViewServer.setSource(upperPortCam);
        //  /dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0 
        lookupCam = new UsbCamera("Upward-facing cam", "/dev/video1");
        lookupCam.setResolution(800, 600);
        lookupServer = new MjpegServer("Lookup view", 1182);
        lookupServer.setSource(lookupCam);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new SetCameraMode());
    }


    public class VisionTargetInfo {
        VisionTargetInfo(double bearingDegrees, double rangeInches) {
            this.bearingDegrees = bearingDegrees;
            this.rangeInches = rangeInches;
        }

        public double bearingDegrees;
        public double rangeInches;
    }

    /**
     * Returns the last-seen list of vision targets.
     * @return the last-seen list of vision targets.
     */
    public LinkedList<VisionTargetInfo> getCurVisibleVisionTargets() {
        LinkedList<VisionTargetInfo> vvts = new LinkedList<VisionTargetInfo>();
        for(int i = 0; i < bearingsList.length; ++i) {
            vvts.add(new VisionTargetInfo(bearingsList[i], rangesList[i]));
        }
        return vvts;
    }


    /**
     * Get current camera configuration
     * @return true if the camera is configured for human use, 
     * or false if configured for machine vision.
     */
    public boolean isCameraHumanVision() {
        return false;
    }

    /**
     * Command the camera to enter a mode
     * @param isHumanVisible true if the camera should be configured for human use, 
     * or false to configure the camera for machine vision.
     */
    public void setCameraIsHumanVisible(boolean isHumanVisible) {
        
    }
}
