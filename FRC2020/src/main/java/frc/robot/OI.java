package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.StartPosition;
import frc.robot.commands.Nothing;
import frc.robot.commands.RumbleCommand;
import frc.robot.SelectedStrategy.selectedStrategy;
import frc.robot.commands.compound.AutosteerThenRumble;
import frc.robot.commands.compound.ForwardTenFeet;
import frc.robot.commands.drivebase.DriveToVT;
import frc.robot.commands.drivebase.Pivot;
import frc.robot.commands.vision.ToggleCameraMode;
import frc.robot.commands.hgroundloader.AutoAcquire;
import frc.robot.commands.hgroundloader.SetIntakeThrottle;
import frc.robot.commands.hgroundloader.SetWristAngle;
import frc.robot.commands.hgroundloader.WaitForHatchDetected;
import frc.robot.oi.JoystickAxisButton;
import frc.robot.oi.JoystickPovButton;
import frc.robot.oi.MutableSendableChooser;
import frc.robot.oi.XBox360Controller;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the 
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

	StartPosition startPosition;


	/** Describes which of the controlleres you're referring to */
	public enum Controller {
		DRIVER,
		WEAPONS, // Weapons operator
	}

	
	SendableChooser<StartPosition> robotStartingPosition = new SendableChooser<>();

	// All of these commands assume we are scoring a hatch.
	MutableSendableChooser<Command> attackFLRocket = new MutableSendableChooser<>(); // Scores on the left rocket in the front.
	//MutableSendableChooser<Command> attackFL2Rocket = new MutableSendableChooser<>(); // Scores on the left rocket in the front on the second level.
	//MutableSendableChooser<Command> attackFL3Rocket = new MutableSendableChooser<>(); // Scores on the left rocket in the front on the third level.
	MutableSendableChooser<Command> attackFRRocket = new MutableSendableChooser<>(); // Scores on the right rocket in the front.
	//MutableSendableChooser<Command> attackFR2Rocket = new MutableSendableChooser<>(); // Scores on the right rocket in the front on the second level.
	//MutableSendableChooser<Command> attackFR3Rocket = new MutableSendableChooser<>(); // Scores on the right rocket in the front on the third level.
	MutableSendableChooser<Command> attackBLRocket = new MutableSendableChooser<>(); // Scores on the left rocket in the back.
	//MutableSendableChooser<Command> attackBL2Rocket = new MutableSendableChooser<>(); // Scores on the left rocket in the back on the second level.
	//MutableSendableChooser<Command> attackBL3Rocket = new MutableSendableChooser<>(); // Scores on the left rocket in the back on the third level.
	MutableSendableChooser<Command> attackBRRocket = new MutableSendableChooser<>(); // Scores on the right rocket in the back.
	//MutableSendableChooser<Command> attackBR2Rocket = new MutableSendableChooser<>(); // Scores on the right rocket in the back on the second level.
	//MutableSendableChooser<Command> attackBR3Rocket = new MutableSendableChooser<>(); // Scores on the right rocket in the back on the thrid level.
	MutableSendableChooser<Command> attackFLCargoship = new MutableSendableChooser<>(); // Scores on the front left side of the cargoship.
	MutableSendableChooser<Command> attackFRCargoship = new MutableSendableChooser<>(); // Scores on the front right side of the cargoship.
	MutableSendableChooser<Command> attackLCargoship = new MutableSendableChooser<>(); // Scores on the left side of the cargoship.
	//MutableSendableChooser<Command> attackL2Cargoship = new MutableSendableChooser<>(); // Scores on the left side of the cargoship second from the front.
	//MutableSendableChooser<Command> attackL3Cargoship = new MutableSendableChooser<>(); // Scores on the left side of the cargoship furthest from the front.
	MutableSendableChooser<Command> attackRCargoship = new MutableSendableChooser<>(); // Scores on the right side of the cargoship.
	//MutableSendableChooser<Command> attackR2Cargoship = new MutableSendableChooser<>(); // Scores on the right side of the cargoship second from the front.
	//MutableSendableChooser<Command> attackR3Cargoship = new MutableSendableChooser<>(); // Scores on the right side of the cargoship furthest from the front.
	StartPosition lastSelectedPosition = null;

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
		//hglAutoCollect.whileHeld(new AutoAcquire(true));
        hglAutoCollect.close(); // Don't need this one anymore?		

        JoystickButton lineFollowButton = new JoystickButton(driverController, BUTTON_FORCE_HIGH_GEAR);
        lineFollowButton.whileHeld(new AutosteerThenRumble());
		lineFollowButton.close();

		robotStartingPosition.addObject("Hab 1 Left", StartPosition.HAB_1_LEFT);
		robotStartingPosition.addObject("Hab 1 Center", StartPosition.HAB_1_CENTER);
		robotStartingPosition.addObject("Hab 1 Right", StartPosition.HAB_1_RIGHT);
		robotStartingPosition.addObject("Hab 1 Left", StartPosition.HAB_2_LEFT);
		robotStartingPosition.addObject("Hab 2 Right", StartPosition.HAB_2_RIGHT);
		robotStartingPosition.addObject("Hab 3", StartPosition.HAB_3);

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

		//updateSmartChoosers();

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

	public StartPosition getRobotStartPosition()
	{
		return robotStartingPosition.getSelected();
	}

	public void updateSmartChoosers() {
		StartPosition curPos = robotStartingPosition.getSelected();

		if (curPos != lastSelectedPosition) {
			System.out.println("Updating auto move choices list");
			updateAttackFLRocket(curPos);
			updateAttackFRRocket(curPos);
			updateAttackBLRocket(curPos);
			updateAttackBRRocket(curPos);
			updateAttackFLCargoship(curPos);
			updateAttackFRCargoship(curPos);
			updateAttackLCargoship(curPos);
			updateAttackRCargoship(curPos);
		}
		lastSelectedPosition = curPos;
	}

	public Command getSelectedCommand(StartPosition startPosition)
	{
		if (startPosition == StartPosition.UNKNOWN)
		{
			return attackFLRocket.getSelected();
		}
		else if ((startPosition == StartPosition.HAB_1_LEFT) || (startPosition == StartPosition.HAB_2_LEFT)
			|| (startPosition == StartPosition.HAB_1_CENTER) || (startPosition == StartPosition.HAB_3))
		{
			return attackFLRocket.getSelected();
		}
		else if ((startPosition == StartPosition.HAB_1_RIGHT) || (startPosition == StartPosition.HAB_1_CENTER)
			|| (startPosition == StartPosition.HAB_2_RIGHT) || (startPosition == StartPosition.HAB_3))
		{
			return attackFRRocket.getSelected();
		}

		// else if (whichStratIsSelected == 2)
		// {
		// 	if (startPosition == StartPosition.HAB_1_LEFT) || startPosition == StartPosition.HAB_2_LEFT)
		// 		|| startPosition == StartPosition.HAB_1_CENTER) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackFL2Rocket.getSelected();
		// 	}
		// 	else if (startPosition == StartPosition.HAB_1_RIGHT) || startPosition == StartPosition.HAB_1_CENTER)
		// 		|| startPosition == StartPosition.HAB_2_RIGHT) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackFR2Rocket.getSelected();
		// 	}
		// 	else
		// 	{
		// 		return new Nothing();
		// 	}
			
		// }
		// else if (whichStratIsSelected == 3)
		// {
		// 	if (startPosition == StartPosition.HAB_1_LEFT) || startPosition == StartPosition.HAB_2_LEFT)
		// 		|| startPosition == StartPosition.HAB_1_CENTER) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackFL3Rocket.getSelected();
		// 	}
		// 	else if (startPosition == StartPosition.HAB_1_RIGHT) || startPosition == StartPosition.HAB_1_CENTER)
		// 		|| startPosition == StartPosition.HAB_2_RIGHT) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackFR3Rocket.getSelected();
		// 	}
		// 	else
		// 	{
		// 		return new Nothing();
		// 	}
			
		// }

			if ((startPosition == StartPosition.HAB_1_LEFT) || (startPosition == StartPosition.HAB_2_LEFT)
				|| (startPosition == StartPosition.HAB_1_CENTER) || (startPosition == StartPosition.HAB_3))
			{
				return attackBLRocket.getSelected();
			}
			else if ((startPosition == StartPosition.HAB_1_RIGHT) || (startPosition == StartPosition.HAB_1_CENTER)
				|| (startPosition == StartPosition.HAB_2_RIGHT) || (startPosition == StartPosition.HAB_3))
			{
				return attackBRRocket.getSelected();
			}
			
		// else if (whichStratIsSelected == 5)
		// {
		// 	if (startPosition == StartPosition.HAB_1_LEFT) || startPosition == StartPosition.HAB_2_LEFT)
		// 		|| startPosition == StartPosition.HAB_1_CENTER) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackBL2Rocket.getSelected();
		// 	}
		// 	else if (startPosition == StartPosition.HAB_1_RIGHT) || startPosition == StartPosition.HAB_1_CENTER)
		// 		|| startPosition == StartPosition.HAB_2_RIGHT) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackBR2Rocket.getSelected();
		// 	}
		// 	else
		// 	{
		// 		return new Nothing();
		// 	}
			
		// }
		// else if (whichStratIsSelected == 6)
		// {
		// 	if (startPosition == StartPosition.HAB_1_LEFT) || startPosition == StartPosition.HAB_2_LEFT)
		// 		|| startPosition == StartPosition.HAB_1_CENTER) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackBL3Rocket.getSelected();
		// 	}
		// 	else if (startPosition == StartPosition.HAB_1_RIGHT) || startPosition == StartPosition.HAB_1_CENTER)
		// 		|| startPosition == StartPosition.HAB_2_RIGHT) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackBR3Rocket.getSelected();
		// 	}
		// 	else
		// 	{
		// 		return new Nothing();
		// 	}
			
		// }

			else if ((startPosition == StartPosition.HAB_1_LEFT) || (startPosition == StartPosition.HAB_2_LEFT)
				|| (startPosition == StartPosition.HAB_1_CENTER) || (startPosition == StartPosition.HAB_3))
			{
				return attackFLCargoship.getSelected();
			}
			else if ((startPosition == StartPosition.HAB_1_RIGHT) || (startPosition == StartPosition.HAB_1_CENTER)
				|| (startPosition == StartPosition.HAB_2_RIGHT) || (startPosition == StartPosition.HAB_3))
			{
				return attackFRCargoship.getSelected();
			}
			if ((startPosition == StartPosition.HAB_1_LEFT) || (startPosition == StartPosition.HAB_2_LEFT)
				|| (startPosition == StartPosition.HAB_1_CENTER) || (startPosition == StartPosition.HAB_3))
			{
				return attackLCargoship.getSelected();
			}
			else if ((startPosition == StartPosition.HAB_1_RIGHT) || (startPosition == StartPosition.HAB_1_CENTER)
				|| (startPosition == StartPosition.HAB_2_RIGHT) || (startPosition == StartPosition.HAB_3))
			{
				return attackRCargoship.getSelected();
			}
			else
			{
				return new Nothing();
			}
			
		// else if (whichStratIsSelected == 9)
		// {
		// 	if (startPosition == StartPosition.HAB_1_LEFT) || startPosition == StartPosition.HAB_2_LEFT)
		// 		|| startPosition == StartPosition.HAB_1_CENTER) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackL2Cargoship.getSelected();
		// 	}
		// 	else if (startPosition == StartPosition.HAB_1_RIGHT) || startPosition == StartPosition.HAB_1_CENTER)
		// 		|| startPosition == StartPosition.HAB_2_RIGHT) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackR2Cargoship.getSelected();
		// 	}
		// 	else
		// 	{
		// 		return new Nothing();
		// 	}
			
		// }
		// else if (whichStratIsSelected == 10)
		// {
		// 	if (startPosition == StartPosition.HAB_1_LEFT) || startPosition == StartPosition.HAB_2_LEFT)
		// 		|| startPosition == StartPosition.HAB_1_CENTER) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackL3Cargoship.getSelected();
		// 	}
		// 	else if (startPosition == StartPosition.HAB_1_RIGHT) || startPosition == StartPosition.HAB_1_CENTER)
		// 		|| startPosition == StartPosition.HAB_2_RIGHT) || startPosition == StartPosition.HAB_3))
		// 	{
		// 		return attackR3Cargoship.getSelected();
		// 	}
	}

	// AUTO MOVE CHOOSERS

	private void updateAttackFLRocket(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack FL Rocket: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackFLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackFLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackFLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackFLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackFLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackFLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case UNKNOWN:
			attackFLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
		default:
			break;
		}
	}

	private void updateAttackFRRocket(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack FR Rocket: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackFRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackFRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackFRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackFRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackFRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackFRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		default:
			break;
		}
	}

	private void updateAttackBLRocket(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack BL Rocket: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackBLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackBLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackBLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackBLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackBLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackBLRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		default:
			break;
		}
	}

	private void updateAttackBRRocket(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack BR Rocket: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackBRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackBRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackBRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackBRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackBRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackBRRocket.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		default:
			break;
		}
	}

	private void updateAttackFLCargoship(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack FL Cargoship: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		default:
			break;
		}
	}

	private void updateAttackFRCargoship(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack FR Cargoship: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackFLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		default:
			break;
		}
	}

	private void updateAttackLCargoship(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack L Cargoship: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackLCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		default:
			break;
		}
	}

	private void updateAttackRCargoship(StartPosition robotStartPosition) {
		// Clear chooser before updating
		attackFLRocket.clear();

		// Default move || The closest thing we have to a label
		attackFLRocket.addDefault("Attack R Cargoship: Nothing", new Nothing());

		switch (robotStartPosition) {
		case HAB_1_LEFT:
			attackRCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_CENTER:
			attackRCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_1_RIGHT:
			attackRCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_LEFT:
			attackRCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_2_RIGHT:
			attackRCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		case HAB_3:
			attackRCargoship.addObject("Forward 10 Feet", new ForwardTenFeet());
			break;
		default:
			break;
		}
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
