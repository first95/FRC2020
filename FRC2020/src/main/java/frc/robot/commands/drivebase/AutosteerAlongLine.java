/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Leaves the driver in manual control of forward speed.
 * When the line is detected under a line sensor, steers the robot so it is driving parallel to the line.
 * If no line is detected, steering remains under manual control.
 */
public class AutosteerAlongLine extends Command {
    // How fast the robot should turn when the sensor at the extreme right or left sees a line
    public static final double MAX_TURN_RATE = 0.5;
    // How fast the robot should drive forward when the sensor in the center sees a line
    public static final double MAX_FWD_RATE = 0.5;
    
	public AutosteerAlongLine() {
		requires(Robot.drivebase);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        Robot.drivebase.setMaxSpeed(1);
        double turn;
        double fwd;
        if(Robot.drivebase.doesAnySensorSeeTheLine()) {
            System.out.println("Some sensor sees a line!");
            double[] newSpeeds = getLineFollowSpeeds();
            turn = newSpeeds[0];
            fwd = newSpeeds[1];
        } else {
            // Exponential response
            System.out.println("No sensor sees a line.");
            turn = Math.pow(Robot.oi.getTurnAxis(), 3);
            fwd  = Math.pow(Robot.oi.getForwardAxis(), 3);
        }

        // Put it into action
		Robot.drivebase.arcade(fwd, turn);
    }
    
    /**
     * Check the sensors and compute the appropriate turn rate [0] and fwd rate [1]
     * @return {turn rate (-1.0 for full left (CCW), +1.0 for full right (CW), forward rate (-1.0 for reverse, +1.0 for full fwd))}
     */
	private double[] getLineFollowSpeeds() {
        double turnRateAccum = 0;
        double fwdRateAccum = 0;
        int numSensors = Robot.drivebase.getLineSensorCount();
        int numSensorsTripped = 0;

        // Accumulation part of the average
        for(int sensor = 0; sensor < numSensors; sensor++) {
            if(Robot.drivebase.doesSensorSeeLine(sensor)) {
                turnRateAccum += getTurnRateForSingleSensor(sensor);
                fwdRateAccum += getFwdRateForSingleSensor(sensor);
                numSensorsTripped++;
            }
        }

        // Actually perform the average
        return new double[] {turnRateAccum/numSensorsTripped,fwdRateAccum/numSensorsTripped};
    }

    /**
     * @param sensor The index of the sensor.  Must be >= 0 and < Robot.drivebase.getLineSensorCount()
     * @return The rate at which the robot should turn if only this sensor is tripped.  -1 for full speed left (CCW from above), +1 for full right (CW from above)
     */
    private double getTurnRateForSingleSensor(int sensor) {
        // Index of the sensor in the center.  This is also the count of sensors to the right or left of center.
        int centerSensorIndex = Robot.drivebase.getCenterSensorIndex();
        return (((double)(sensor - centerSensorIndex)) / centerSensorIndex) * MAX_TURN_RATE;
    }

    /**
     * @param sensor The index of the sensor.  Must be >= 0 and < Robot.drivebase.getLineSensorCount()
     * @return The rate at which the robot should drive forward if only this sensor is tripped.  -1 for full speed left (CCW from above), +1 for full right (CW from above)
     */
    private double getFwdRateForSingleSensor(int sensor) {
        // Index of the sensor in the center.  This is also the count of sensors to the right or left of center.
        int centerSensorIndex = Robot.drivebase.getCenterSensorIndex();
        return ((double)(Math.abs(sensor - centerSensorIndex)-centerSensorIndex) / centerSensorIndex) * MAX_TURN_RATE;
    }

    // Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
        //System.out.println("We " + (Robot.drivebase.doAllForwardSensorsSeeWall()? "see":"do not see") + " see the wall.");
        if (Robot.drivebase.doAllForwardSensorsSeeWall()){
            return true;
        } else {
            return false; // This command runs until interrupted
        }
	}

}
