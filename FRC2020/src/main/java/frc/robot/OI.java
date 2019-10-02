package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.compound.AutosteerThenRumble;
import frc.robot.commands.drivebase.DriveToVT;
import frc.robot.commands.drivebase.Pivot;
import frc.robot.commands.vision.ToggleCameraMode;
import frc.robot.commands.hgroundloader.AutoAcquire;
import frc.robot.commands.hgroundloader.SetIntakeThrottle;
import frc.robot.commands.hgroundloader.SetWristAngle;
import frc.robot.commands.hgroundloader.WaitForHatchDetected;
import frc.robot.oi.JoystickAxisButton;
import frc.robot.oi.JoystickPovButton;
import frc.robot.oi.XBox360Controller;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * One can use SmartDashboard with a controller plugged in to see which values corrospond
 * to which controls.
 */
public class OI {

	// Controllers
	private Joystick driverController = new Joystick(0);
	private Joystick weaponsController = new Joystick(1);

	// Buttons on drive controller
	//public static final int CLIMB_SKIDS_BUTTON = 0;// XBox360Controller.Button.LEFT_BUMPER.Number();
	public static final int SWITCH_CAM_VIEW_BUTTON = XBox360Controller.Button.START.Number();
	// Features not presently in use - getRawButton(0) always returns false
	public static final int BUTTON_FORCE_LOW_GEAR = XBox360Controller.Button.LEFT_BUMPER.Number();
	public static final int BUTTON_FORCE_HIGH_GEAR = XBox360Controller.Button.RIGHT_BUMPER.Number();
	public static final int CLIMB2_TOGGLE_FRONT = XBox360Controller.Button.A.Number();
	public static final int CLIMB2_TOGGLE_REAR = XBox360Controller.Button.Y.Number();
	public static final int BRAKES_DEPLOY = XBox360Controller.Button.X.Number();

	// Axes on drive controller
	public static final int DRIVE_FORWARD_AXIS = XBox360Controller.Axis.LEFT_STICK_Y.Number();
	public static final int DRIVE_TURN_AXIS = XBox360Controller.Axis.RIGHT_STICK_X.Number();
	//public static final int CLIMBER_UP_AXIS = XBox360Controller.Axis.LEFT_TRIGGER.Number();
    //public static final int CLIMBER_DOWN_AXIS = XBox360Controller.Axis.RIGHT_TRIGGER.Number();

	// Buttons on weapons controller
	public static final int ELEV_PRESET_HATCH_LOAD = XBox360Controller.Button.A.Number();
	public static final int ELEV_PRESET_HATCH_LOW = XBox360Controller.Button.B.Number();
	public static final int ELEV_PRESET_HATCH_MID = XBox360Controller.Button.X.Number(); 
	public static final int ELEV_PRESET_HATCH_HIGH = XBox360Controller.Button.Y.Number();
	public static final int HS_CLOSE_HOLD = XBox360Controller.Button.LEFT_BUMPER.Number();
	public static final int HS_PUSH_HOLD = XBox360Controller.Button.RIGHT_BUMPER.Number();
	public static final int ELEV_YOU_ARE_HOME = XBox360Controller.Button.BACK.Number();
	public static final int HGL_RETRACT_WRIST = XBox360Controller.PovDir.UP.Degrees();
	public static final int HGL_AUTO_COLLECT = XBox360Controller.Button.START.Number(); // XBox360Controller.PovDir.DOWN.Degrees();
	public static final int CH_WRIST_UP = XBox360Controller.PovDir.LEFT.Degrees();
	public static final int CH_WRIST_COLLECT = XBox360Controller.PovDir.RIGHT.Degrees();
	
	// Axes on weapons controller
	public static final int HGL_INTAKE_AXIS = XBox360Controller.Axis.LEFT_TRIGGER.Number();
	public static final int HGL_OUTSPIT_AXIS = XBox360Controller.Axis.RIGHT_TRIGGER.Number();
	public static final int CARGO_HANDLER_INTAKE_AXIS = XBox360Controller.Axis.LEFT_TRIGGER.Number();
	public static final int CARGO_HANDLER_OUTSPIT_AXIS = XBox360Controller.Axis.RIGHT_TRIGGER.Number();
	public static final int CARGO_HANDLER_WRIST_AXIS = XBox360Controller.Axis.LEFT_STICK_Y.Number();
	public static final int ELEVATOR_AXIS = XBox360Controller.Axis.RIGHT_STICK_Y.Number();

	private static final double ELEVATOR_UPDOWN_DEADBAND = 0.18;
	private static final double CARGO_INTAKE_DEADBAND = 0.1;


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
		JoystickButton cameraViewSwitcher = new JoystickButton(driverController, SWITCH_CAM_VIEW_BUTTON);
        cameraViewSwitcher.whenPressed(new ToggleCameraMode());
		cameraViewSwitcher.close(); // Don't need this one anymore?
		
		JoystickButton hglAutoCollect = new JoystickButton(weaponsController, HGL_AUTO_COLLECT);
		hglAutoCollect.whileHeld(new AutoAcquire(true));
        hglAutoCollect.close(); // Don't need this one anymore?		

        JoystickButton lineFollowButton = new JoystickButton(driverController, BUTTON_FORCE_HIGH_GEAR);
        lineFollowButton.whileHeld(new AutosteerThenRumble());
		lineFollowButton.close();

		// // For testing 
        // JoystickAxisButton testRumble = new JoystickAxisButton(driverController, XBox360Controller.Axis.LEFT_TRIGGER.Number());
        // testRumble.whenPressed(new RumbleCommand(Controller.DRIVER, RumbleType.kLeftRumble ,1.0 , 1.0, false));
        // testRumble.close();

		// Sendable Chooser for single commands
		// These are only for testing Purposes
		// Rotations
		SmartDashboard.putData("Drive to vision target", new DriveToVT());
		// SmartDashboard.putData("Pivot 90 degrees CW", new Pivot(90));
		
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

	// If anything needs to be posted to the SmartDashboard, place it here
	public void log() {
		
	}


	// Hatch scorer
	/**
	 * Check if the Open Hatch Grabber button was pressed since last check
	 * @return true if the Open Hatch Grabber button was pressed since last check
	 */
	public boolean isGrabHatchButtonHeld() {
		return weaponsController.getRawButton(HS_CLOSE_HOLD);
	}
	/**
	 * Check if the Push Hatch Grabber button was pressed since last check
	 * @return true if the Push Hatch Grabber button was pressed since last check
	 */
	public boolean isPushHatchButtonHeld() {
		return weaponsController.getRawButton(HS_PUSH_HOLD);
	}	

	/**
	 * Get speed at which the intake rollers of the hatch ground loader should run
	 * @return -1.0 for fully outward, 1.0 for fully inward, 0.0 for stationary
	 */
	public double getHGLIntakeSpeed() {
		return weaponsController.getRawAxis(HGL_INTAKE_AXIS) - weaponsController.getRawAxis(HGL_OUTSPIT_AXIS);
	}

	/**
	 * Get speed at which the wrist of of the hatch ground loader should turn
	 * @return -1.0 for fully downward, 1.0 for fully upward, 0.0 for stationary
	 */
	public double getHGLWristSpeed() {
		return 0; //weaponsController.getRawAxis(HGL_WRIST_AXIS);
	}

	/**
	 * Get whether HGL wrist UP button is pressed
	 * @return true to bring HGL wrist up, false otherwise
	 */	
	public boolean isHGLWristUpButtonPressed() {
		return weaponsController.getPOV() == HGL_RETRACT_WRIST;
	}	

	/**
	 * Get speed at which the motor of the climber should move
	 * @return -1.0 for fully downward, 1.0 for fully upward, 0.0 for stationary
	 */
	public double getClimberSpeed() {
		return 0; //driverController.getRawAxis(CLIMBER_UP_AXIS) - driverController.getRawAxis(CLIMBER_DOWN_AXIS);
	}

	/**
	 * Get deploy state for skids
	 * @return true to deploy and false to retract
	 */	
	public boolean isDeploySkidsToggled() {
		return false; //driverController.getRawButtonPressed(CLIMB_SKIDS_BUTTON);
	}

	public boolean isDeployFrontClimberToggled() {
		return driverController.getRawButtonPressed(CLIMB2_TOGGLE_FRONT);//return false;
	}

	public boolean isDeployRearClimberToggled() {
		return driverController.getRawButtonPressed(CLIMB2_TOGGLE_REAR);//return false;
	}

	/**
	 * Get speed at which the intake rollers of the cargo handler should run
	 * @return -1.0 for fully outward, 1.0 for fully inward, 0.0 for stationary
	 */
	public double getCargoHandlerIntakeSpeed() {
		return weaponsController.getRawAxis(CARGO_HANDLER_INTAKE_AXIS) - weaponsController.getRawAxis(CARGO_HANDLER_OUTSPIT_AXIS);
	}

	/**
	 * Get speed at which the wrist of of the cargo handler  should turn
	 * @return -1.0 for fully downward, 1.0 for fully upward, 0.0 for stationary
	 */
	public double getCargoHandlerWristSpeed() {
		return weaponsController.getRawAxis(CARGO_HANDLER_WRIST_AXIS);
	}

	/**
	 * Get whether cargo handler wrist UP button is pressed
	 * @return true to bring cargo handler wrist up, false otherwise
	 */	
	public boolean isCHWristUpButtonPressed() {
		return weaponsController.getPOV() == CH_WRIST_UP;
	}

	/**
	 * Get whether cargo handler wrist COLLECT button is pressed
	 * @return true to bring cargo handler wrist to collect position, false otherwise
	 */	
	public boolean isCHWristCollectButtonPressed() {
		return weaponsController.getPOV() == CH_WRIST_COLLECT;
	}	

	// Brakes
	/**
	 * Check if the Brakes button is currently held
	 * @return true if the Brakes button is currently held
	 */
	public boolean isBrakesButtonHeld() {
		return driverController.getRawButton(BRAKES_DEPLOY);
	}

	// Elevator controls
	public double getElevatorSpeed() {

		double elevatorCtrl = weaponsController.getRawAxis(ELEVATOR_AXIS);
		double elevatorSpeed = 0;

		if ((elevatorCtrl > ELEVATOR_UPDOWN_DEADBAND)
				|| (elevatorCtrl < -ELEVATOR_UPDOWN_DEADBAND)) {
			elevatorSpeed = elevatorCtrl;
		}

		// The Y axis on thet controller is reversed, so that positive is down
		SmartDashboard.putNumber("Elevator axis value",elevatorCtrl);
		SmartDashboard.putNumber("Elevator speed throttle",-elevatorSpeed);
		return -elevatorSpeed * 1.0;
	}

	public Elevator.ElevatorHoldPoint getCommandedHoldPoint() {
		// Prioritize lower setpoints if the user holds more than one button
		if(weaponsController.getRawButton(ELEV_PRESET_HATCH_LOAD)) {
			return Elevator.ElevatorHoldPoint.HATCH_COVER_LOAD;
		} else if(weaponsController.getRawButton(ELEV_PRESET_HATCH_LOW)) {
			return Elevator.ElevatorHoldPoint.HATCH_COVER_LOW;
		} else if(weaponsController.getRawButton(ELEV_PRESET_HATCH_MID)) {
			if(this.getCargoHandlerIntakeSpeed()>CARGO_INTAKE_DEADBAND) {
				return Elevator.ElevatorHoldPoint.CARGO_MID;
			} else {
				return Elevator.ElevatorHoldPoint.HATCH_COVER_MID;
			}
		} else if(weaponsController.getRawButton(ELEV_PRESET_HATCH_HIGH)) {
			if(this.getCargoHandlerIntakeSpeed()>CARGO_INTAKE_DEADBAND) {
				return Elevator.ElevatorHoldPoint.CARGO_HIGH;
			} else {
				return Elevator.ElevatorHoldPoint.HATCH_COVER_HIGH;
			}
		} else {
			return Elevator.ElevatorHoldPoint.NONE;
		}
	}

	public boolean getElevatorHomeButtonPressed() {
		return weaponsController.getRawButton(ELEV_YOU_ARE_HOME);
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
		return false; //return driverController.getRawButton(BUTTON_FORCE_HIGH_GEAR);
	}

	/**
	 * Ask if the driver wants the robot to be in low gear
	 * @return
	 */
	public boolean getLowGear() {
		return driverController.getRawButton(BUTTON_FORCE_LOW_GEAR);
	}

	/**
	 * Rumble a controller.
	 * Note that you may have overlapping low- and high-pitched rumbles
	 * @param controller which controller to rumble
	 * @param side right of left side
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
}
