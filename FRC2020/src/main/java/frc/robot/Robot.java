package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHabLevelTwo;
import frc.robot.subsystems.HatchGroundLoader;
import frc.robot.subsystems.HatchScorer;
import frc.robot.subsystems.PathFinderSystem;
import frc.robot.commands.drivebase.PathFinderCommand;
import frc.robot.subsystems.VisionCoprocessor;
import frc.robot.SelectedStrategy;
import frc.robot.components.StartPosition;
import frc.robot.subsystems.Brakes;
import frc.robot.subsystems.DriveBase.GearShiftMode;
import frc.robot.commands.compound.ForwardTenFeet;
//import jaci.pathfinder.Pathfinder;
import frc.robot.commands.drivebase.PathFinderCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {

	private StartPosition robotStartSide; // The location where the robot began
	private String gameData;
	Command autonomousCommand;

	// Components of the robot
	public static DriveBase drivebase;
	public static Elevator elevator;
	public static HatchScorer hScorer;
	public static HatchGroundLoader hGroundLoader;
	public static CargoHandler cargoHandler;
	public static Climber climber;
	public static ClimberHabLevelTwo climber2;
	public static Compressor compressor;
	public static OI oi;
	public static VisionCoprocessor vision;
	public static Brakes brakes;
	public static PathFinderSystem pathfinder;
	public static StartPosition startPosition;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		// Initialize all subsystems
		drivebase = new DriveBase(true);
		elevator = new Elevator(true);
		hScorer = new HatchScorer(true);
		hGroundLoader = new HatchGroundLoader(true);
		cargoHandler = new CargoHandler(true);
		climber = new Climber(false);
		climber2 = new ClimberHabLevelTwo(false);
		compressor = new Compressor();
		vision = new VisionCoprocessor();
		brakes = new Brakes(true);
		pathfinder = new PathFinderSystem(true, false, 0, 1);
        oi = new OI();

		// Show what command your subsystem is running on the SmartDashboard
		SmartDashboard.putData(drivebase);
		SmartDashboard.putData(elevator);
		SmartDashboard.putData(hGroundLoader);
		SmartDashboard.putData(climber);

		// Disable brakes on talons to make it
		// easier to push
		drivebase.brake(false);
		elevator.brake(false);

	}

	@Override
	public void autonomousInit()
	{
		System.out.println("made it");
		int maxTime_sec = 8;
		double startTime_sec = Timer.getFPGATimestamp();
		double elapTime_sec = Timer.getFPGATimestamp() - startTime_sec;
		gameData = "";
		// while ((gameData.length() < 3) & (elapTime_sec < maxTime_sec)) {
		// 	gameData = DriverStation.getInstance().getGameSpecificMessage();
		// 	elapTime_sec = Timer.getFPGATimestamp() - startTime_sec;
		// }
		if (gameData == "") {
			
			gameData = "UUU";
		} else {
			System.out.println("Time to get game data was "+elapTime_sec+" seconds.");
		}
		//System.out.println("Plate assignments are " + gameData);

		robotStartSide = oi.getRobotStartPosition();
		System.out.println("Robot start side: " + robotStartSide);
		System.out.println("Strategy #"  + getWhichStratIsSelected() + " has been selected.");
		
		autonomousCommand = new PathFinderCommand(true, false, 20, 1);
		autonomousCommand.start();
    }

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		robotPeriodic();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	public void disabledInit() {
		drivebase.brake(false);
		elevator.brake(false);
	}

	public void disabledPeriodic() {
		robotPeriodic();
	}

	@Override
	public void robotPeriodic() {
		Scheduler.getInstance().run(); // Runs all active commands
		// elevator.checkAndApplyHomingSwitch();
		// drivebase.pullPidConstantsFromSmartDash();
		// oi.visit();
		// drivebase.visit();

		// // Depending if you want all output or just limited
		// // use either debugLog() or just log()
		// // debugLog();
		// log();
	}

	@Override
	public void teleopInit() {

		// Unlock the auto shifter
		drivebase.setShiftMode(GearShiftMode.AUTOSHIFT);

		drivebase.brake(true);
		elevator.brake(true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		robotPeriodic();
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
		//brakes.log();
		elevator.log();
		//cargoHandler.log();
		hGroundLoader.log();
		// oi.log();
	}

	public StartPosition getRobotStartSide() {
		return robotStartSide;
	}

	private StartPosition sideFromChar(char side)
	{
		if(side == 'A') {
			return StartPosition.HAB_1_LEFT;
		} else if(side == 'B') {
			return StartPosition.HAB_1_CENTER;
		} else if(side == 'C') {
			return StartPosition.HAB_1_RIGHT;
		} else if(side == 'D') {
			return StartPosition.HAB_2_LEFT;
		} else if(side == 'E') {
			return StartPosition.HAB_2_RIGHT;
		} else if(side == 'F') {
			return StartPosition.HAB_3;
		} else {
			return StartPosition.UNKNOWN;
		}
	}

	public StartPosition getWhichStratIsSelected()
	{
		return sideFromChar(gameData.charAt(0));
	}
}
