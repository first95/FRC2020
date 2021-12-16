package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.commands.drivebase.AutoCollect;
import frc.robot.oi.XBox360Controller;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * One can use SmartDashboard with a controller plugged in to see which values corrospond
 * to which controls.
 */
public class OI {

	public static boolean auto_shooting = false;
	public static boolean auto_collector_deploy = false;
	public static double auto_shooting_speed = 2100;
	public static double auto_collect_speed = 0;
	public static boolean toggle_shooter_hood = false;
	public static boolean shooter_hood_state = false;

	// Controllers
	private Joystick driverController = new Joystick(0);
	private Joystick weaponsController = new Joystick(1);
	
	// Buttons on weapons controller
	private JoystickButton runIndexer; 
	public static final int GROUND_PICK_UP_DEPLOY = XBox360Controller.Button.X.Number(); 
	public static final int SINGULATOR_BUTTON = XBox360Controller.Button.A.Number();
	public static final int SINGULATOR_INTAKE_BUTTON = XBox360Controller.Button.B.Number();
	public static final int SHOOTER_BUTTON = XBox360Controller.Button.Y.Number();
	public static final int BACKWARDS_BUTTON = XBox360Controller.Button.RIGHT_BUMPER.Number();


	// Buttons on drive controller
	//public static final int CLIMB_SKIDS_BUTTON = 0;// XBox360Controller.Button.LEFT_BUMPER.Number();
	public static final int AUTO_COLLECT_BUTTON = XBox360Controller.Button.START.Number();
	public static final int BUTTON_FORCE_LOW_GEAR = XBox360Controller.Button.LEFT_BUMPER.Number();
	public static final int BUTTON_FORCE_HIGH_GEAR = XBox360Controller.Button.RIGHT_BUMPER.Number();
	public static final int BUTTON_CLIMBER_TOGGLE = XBox360Controller.Button.X.Number();
	public static final int BUTTON_VISION_AIM_A = XBox360Controller.Button.Y.Number();
	public static final int BUTTON_VISION_AIM_B = XBox360Controller.Button.B.Number();
	public static final int BUTTON_VISION_AIM_C = XBox360Controller.Button.A.Number();

	// Axes on drive controller
	public static final int DRIVE_FORWARD_AXIS = XBox360Controller.Axis.LEFT_STICK_Y.Number();
	public static final int DRIVE_TURN_AXIS = XBox360Controller.Axis.RIGHT_STICK_X.Number();
	public static final int SUCKER_AXIS = XBox360Controller.Axis.RIGHT_TRIGGER.Number();
	
	// Axes on weapons controller
	public static final int GROUND_PICK_UP_ROLLER_AXIS = XBox360Controller.Axis.LEFT_TRIGGER.Number();
	public static final int HUMAN_PLAYER_PICKUP_ROLLER_AXIS = XBox360Controller.Axis.LEFT_TRIGGER.Number();

	/** Describes which of the controlleres you're referring to */
	public enum Controller {
		DRIVER,
		WEAPONS, // Weapons operator
	}

	// System timestamps after which we want each rumbler to be turned off
	private double driverLeftRumbleStopTime = 0;
	private double driverRightRumbleStopTime = 0;
	private double weaponsLeftRumbleStopTime = 0;
	private double weaponsRightRumbleStopTime = 0;

	public OI() {

		// // Create some buttons

		runIndexer = new JoystickButton(weaponsController, XBox360Controller.Button.B.Number());

		JoystickButton visionAimRangeA = new JoystickButton(driverController, BUTTON_VISION_AIM_A);
		visionAimRangeA.whileHeld(new AutoAim(Constants.VISION_RANGE_A_INCH));

		JoystickButton visionAimRangeB = new JoystickButton(driverController, BUTTON_VISION_AIM_B);
		visionAimRangeB.whileHeld(new AutoAim(Constants.VISION_RANGE_B_INCH));

		JoystickButton visionAimRangeC = new JoystickButton(driverController, BUTTON_VISION_AIM_C);
		visionAimRangeC.whileHeld(new AutoAim(Constants.VISION_RANGE_C_INCH));

		JoystickButton autocollect = new JoystickButton(driverController, AUTO_COLLECT_BUTTON);
		autocollect.whileHeld(new AutoCollect());

		
	}

	// There are a few things the OI wants to revisit every time around
	public void visit() {

		// Cancel joystick rumble if necessary
		if(Timer.getFPGATimestamp() > driverLeftRumbleStopTime) {
			driverController.setRumble(RumbleType.kLeftRumble, 0);
		}
		if(Timer.getFPGATimestamp() > driverRightRumbleStopTime) {
			driverController.setRumble(RumbleType.kRightRumble, 0);
		}
		if(Timer.getFPGATimestamp() > weaponsLeftRumbleStopTime) {
			weaponsController.setRumble(RumbleType.kLeftRumble, 0);
		}
		if(Timer.getFPGATimestamp() > weaponsRightRumbleStopTime) {
			weaponsController.setRumble(RumbleType.kRightRumble, 0);
		}
	}

    /**
     * Get the forward travel rate commanded by the driver
     * @return -1 for full speed backward, +1 for full speed forward
     */
	public double getForwardAxis() {
		return driverController.getRawAxis(DRIVE_FORWARD_AXIS);
	}

    /**
     * Get the turn rate commanded by the driver
     * @return -1 for full turn leftward (CCW when looking down at the robot), +1 for full turn rightward (CW when looking down at the robot), 0 for no turn
     */
	public double getTurnAxis() {
		return driverController.getRawAxis(DRIVE_TURN_AXIS);
	}

	/**
	 * Ask if the driver wants the robot to be in high gear
	 * @return
	 */
	public boolean getHighGear() {
		return driverController.getRawButton(BUTTON_FORCE_HIGH_GEAR);
	}

	/**
	 * Ask if the driver wants the robot to be in low gear
	 * @return
	 */
	public boolean getLowGear() {
		return driverController.getRawButton(BUTTON_FORCE_LOW_GEAR);
	}

	/**
     * Get the roller rotation speed commanded by the driver
     * @return -1 for full speed backward, +1 for full speed forward
     */
	public double getGroundPickUpRollerAxis() {
		if (auto_collect_speed == 0) {
			return weaponsController.getRawAxis(GROUND_PICK_UP_ROLLER_AXIS);
		} else {
			return auto_collect_speed;
		}
	}
	public double getHumanPlayerStationPickUpRollerAxis() {
		return weaponsController.getRawAxis(HUMAN_PLAYER_PICKUP_ROLLER_AXIS);
	}

	/**
	 * Ask if the driver wants ground pick-up to be deployed
	 * @return
	 */
	public boolean getGroundPickUpDeployed() {
		// System.out.println("button has been pressed");
		if (auto_collector_deploy) {
			auto_collector_deploy = false;
			return true;
		} else {
			return weaponsController.getRawButtonPressed(GROUND_PICK_UP_DEPLOY);
		}
	}

	/**
	 * Ask if the driver wants climber deploy toggled
	 * @return
	 */
	public boolean getClimberDeployed() {
		return driverController.getRawButtonPressed(BUTTON_CLIMBER_TOGGLE);
	}

	/**
	 * Ask if the indexer should be moving power cells onward
	 * @return
	 */
	public boolean getRunIndexer() {
		return runIndexer.get();
	}
	public boolean getSingulatorButton() {
		return weaponsController.getRawButton(SINGULATOR_BUTTON);
	}

	public boolean getShooterButton() {
		if (auto_shooting) {
			return true;
		}
		else {
			return weaponsController.getRawButton(SHOOTER_BUTTON);
		}
	}
	
	public boolean getBackwardsButtonPressed() {
		return weaponsController.getRawButton(BACKWARDS_BUTTON);
	}

	/**
	 * Rumble a controller.
	 * Note that you may have overlapping low- and high-pitched rumbles
	 * @param controller which controller to rumble
	 * @param side right of left side.  Note that the left side has a lower RPM and what feels like a heavier weight compared to the right.
	 * @param severity how strongly to rumble, between 0.0 and 1.0
	 * @param duration how long, in seconds, the rumble should last
	 */
	public void Rumble(Controller controller, Joystick.RumbleType side, double severity, double duration) {
		Joystick stick = null;
		switch(controller) {
			case DRIVER: 
				stick = driverController; 
				switch(side) {
					case kRightRumble:
						driverRightRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
					case kLeftRumble:
						driverLeftRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
				}
				break;
			case WEAPONS: 
				stick = weaponsController;
				switch(side) {
					case kRightRumble:
						weaponsRightRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
					case kLeftRumble:
						weaponsLeftRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
				}
				break;
		}

		stick.setRumble(side, severity);
	}
	/**
	 * Cease all rumbling
	 */
	public void CancelRumble() {
		CancelRumble(Controller.DRIVER);
		CancelRumble(Controller.WEAPONS);
	}
	/**
	 * Cease all rumbling on a controller
	 */
	public void CancelRumble(Controller controller) {
		Joystick stick = null;
		switch(controller) {
			case DRIVER: 
				stick = driverController; 
				driverRightRumbleStopTime = Timer.getFPGATimestamp() - 1;
				driverLeftRumbleStopTime = Timer.getFPGATimestamp() - 1;
				break;
			case WEAPONS: 
				stick = weaponsController;
				weaponsRightRumbleStopTime = Timer.getFPGATimestamp() - 1;
				weaponsLeftRumbleStopTime = Timer.getFPGATimestamp() - 1;
				break;
		}

		stick.setRumble(Joystick.RumbleType.kRightRumble, 0);
		stick.setRumble(Joystick.RumbleType.kRightRumble, 0);
    }

	/**
	 * Get the amount to power the sucker
	 * @return 0 for off, 1 for full power
	 */
	public double getSuckerAxis() {
		return driverController.getRawAxis(SUCKER_AXIS);
	}
}
