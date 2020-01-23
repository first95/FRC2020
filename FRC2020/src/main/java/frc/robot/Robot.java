package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBase.GearShiftMode;
import frc.robot.subsystems.VisionCoprocessor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {

	// Components of the robot
	public static DriveBase drivebase;
	public static Compressor compressor;
	public static OI oi;
	public static VisionCoprocessor vision;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		// Initialize all subsystems
		drivebase = new DriveBase();
		compressor = new Compressor();
		vision = new VisionCoprocessor();
        oi = new OI();

		// Show what command your subsystem is running on the SmartDashboard
		SmartDashboard.putData(drivebase);
		// Disable brakes on talons to make it
		// easier to push
		drivebase.brake(false);

	}

	@Override
	public void autonomousInit() {

    }

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	public void disabledInit() {
		drivebase.brake(false);
	}

	public void disabledPeriodic() {
	}

	@Override
	public void robotPeriodic() {
		Scheduler.getInstance().run(); // Runs all active commands
		drivebase.pullPidConstantsFromSmartDash();
		oi.visit();
		drivebase.visit();

		// Depending if you want all output or just limited
		// use either debugLog() or just log()
		// debugLog();
		log();
	}

	@Override
	public void teleopInit() {

		// Unlock the auto shifter
		drivebase.setShiftMode(GearShiftMode.AUTOSHIFT);

		drivebase.brake(true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	private void log() {
		debugLog();
	}

	private void debugLog() {
		drivebase.log();
	}
}
