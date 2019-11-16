/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

/**
* Robot position at match start.
* Robot is assumed to have its bumper flush against the alliance wall
* or against the wall of the higher hab level in all these cases.
*/
public enum StartPosition
{
    HAB_1_LEFT,      // Robot is on hab level 1 on the left edge.
	HAB_1_CENTER,    // Robot is approxiamtely in the center on hab level 1.
    HAB_1_RIGHT,     // Robot is on hab level 1 on the right edge.
    HAB_2_LEFT,      // Robot is on the left hab level 2.
	HAB_2_RIGHT,     // Robot is on the right hab level 2.
    HAB_3,           // Robot is in the center of hab level 3.
    UNKNOWN           // Robot start position is unknown.
};
