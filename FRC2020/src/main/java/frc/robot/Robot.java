package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveBase;
// import frc.robot.subsystems.GroundPickUp;
// import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PowerCellMover;
// import frc.robot.subsystems.Singulator;
// import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.VisionProcessor;
import frc.robot.Constants;
import frc.robot.commands.AutoPowerCellMover;
import frc.robot.commands.autocommands.AutoMoves;

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
	public static VisionProcessor vision;
	// public static GroundPickUp groundPickUp;
	// public static Indexer indexer;
	// public static Singulator singulator;
	public static PowerCellMover powerCellMover;
	// public static Shooter shooter;
	public static Climber climber;
	public static double AutoDriveSpeed = 0.3;
	public static double PSAutoDriveSpeed = 0.2;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		// Initialize all subsystems
		drivebase = new DriveBase();
		compressor = new Compressor(Constants.PCM_NUM);
		vision = new VisionProcessor();
		oi = new OI();
		// groundPickUp = new GroundPickUp();
		// indexer = new Indexer();
		// singulator = new Singulator();
		powerCellMover = new PowerCellMover();
		// shooter = new Shooter();
		climber = new Climber();

		// Show what command your subsystem is running on the SmartDashboard
		SmartDashboard.putData(drivebase);
		// Show git build information from Jar Manifest
		SmartDashboard.putString("BuildHost-BranchName", Robot.class.getPackage().getImplementationTitle());
		SmartDashboard.putString("GitCommitID-BuildTimestamp", Robot.class.getPackage().getImplementationVersion());

		SmartDashboard.putNumber("Pre-Shoot Automode Drive speed", PSAutoDriveSpeed);
		SmartDashboard.putNumber("Automode Drive speed (neg for backwards)", AutoDriveSpeed);

		// Disable brakes on talons to make it
		// easier to push
		drivebase.brake(false);

	}

	@Override
	public void autonomousInit() {
		var commandgroup = new AutoMoves();
		Scheduler.getInstance().add(commandgroup);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();

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
		SmartDashboard.putBoolean("SingulatorOccupied",Robot.powerCellMover.getSingulatorSensor());
		SmartDashboard.putBoolean("IndexerEntranceOccupied",Robot.powerCellMover.getIndexerEntranceSensor());
		SmartDashboard.putBoolean("IndexerPosition1Occupied",Robot.powerCellMover.getIndexerLoadedSensor());
		SmartDashboard.putBoolean("ShooterLoaded",Robot.powerCellMover.getShooterLoadedSensor());
		
		SmartDashboard.putBoolean("dummy", AutoPowerCellMover.dummy);
	}

	@Override
	public void robotPeriodic() {
		Scheduler.getInstance().run(); // Runs all active commands
		oi.visit();
		drivebase.visit();

		SmartDashboard.putBoolean("SingulatorOccupied",Robot.powerCellMover.getSingulatorSensor());
		SmartDashboard.putBoolean("IndexerEntranceOccupied",Robot.powerCellMover.getIndexerEntranceSensor());
		SmartDashboard.putBoolean("IndexerPosition1Occupied",Robot.powerCellMover.getIndexerLoadedSensor());
		SmartDashboard.putBoolean("ShooterLoaded",Robot.powerCellMover.getShooterLoadedSensor());

		SmartDashboard.putBoolean("Shooter is loaded", AutoPowerCellMover.shooterIsLoadedCheck);
		SmartDashboard.putBoolean("Moving into singulator", AutoPowerCellMover.movingFromSingulator);
		SmartDashboard.putBoolean("Moving into indexer", AutoPowerCellMover.movingIntoIndexer);
		SmartDashboard.putBoolean("Is in indexer", AutoPowerCellMover.isInIdexer);

		SmartDashboard.putNumber("Shooter speed (RPM)", powerCellMover.getShooterSpeed());

		// SmartDashboard.getNumber("Automode Drive speed (neg for backwards)", AutoDriveSpeed);
		// SmartDashboard.putNumber("Pre-Shoot Automode Drive speed", PSAutoDriveSpeed);

	}

	@Override
	public void teleopInit() {

		// Unlock the auto shifter
		// drivebase.setShiftMode(GearShiftMode.AUTOSHIFT);

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

}
