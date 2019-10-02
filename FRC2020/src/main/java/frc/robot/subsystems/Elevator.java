package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.elevator.ManuallyControlElevator;
import frc.robot.components.FakeTalon;
import frc.robot.components.AdjustedTalon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {
	private static final double K_F = 0.0; // Don't use in position mode.
	private double K_P = 0.3 * 1023.0 / 900.0; // Respond to an error of 900 with 40% throttle
	private double K_I = 0.01 * K_P;
	private double K_D = 0; // 40.0 * K_P;
	private static final int I_ZONE = 200; // In closed loop error units
	private final String pLabel = "Winch P";
	private final String iLabel = "Winch I";
	private final String dLabel = "Winch D";
	public static final double INCHES_FULL_RANGE = 74-9.5 ;// Measured on 2019-4-12
	public static final double ENCODER_TICKS_FULL_RANGE = 72299.0; // Measured 2019-4-12
	private static final double TICKS_PER_INCH = ENCODER_TICKS_FULL_RANGE / INCHES_FULL_RANGE;
	private static final double TICKS_PER_FOOT = TICKS_PER_INCH * 12;
    //private static final double SOFT_FWD_LIMIT = ENCODER_TICKS_FULL_RANGE * 0.96;
    private static final double HATCH_SCORER_HEIGHT_IN = 12.3; // This is the height off the ground that the hatch scorer is at when the elevator is at 0 inches
    private static final double HATCH_LOW_OFFSET_IN = 2.0; 
	private static final double HATCH_MID_HIGH_OFFSET_IN = -2.0; 
	private static final double CARGO_OFFSET_IN = -8.5;

	private IMotorControllerEnhanced followerDriver, leaderDriver;
	private DigitalInput homeSwitch;

	public enum ElevatorHoldPoint {
		NONE(0),			// Not commanded to any specific position
		HERE(0),			// Stay exactly where you are
		HATCH_HANDOFF(3.9),  // The point at which we need to position the elevator to retrieve a hatch from the ground loader
		HATCH_COVER_LOAD(19 - HATCH_SCORER_HEIGHT_IN),// The point at which we need to position the elevator to retrieve a hatch from the loading station
		HATCH_COVER_LOW(19 - HATCH_SCORER_HEIGHT_IN + HATCH_LOW_OFFSET_IN),// The point at which we need to position the elevator to score a hatch cover on the low position
		HATCH_COVER_MID(47 - HATCH_SCORER_HEIGHT_IN + HATCH_MID_HIGH_OFFSET_IN),// The point at which we need to position the elevator to score a hatch cover on the middle position
		HATCH_COVER_HIGH(70 - HATCH_SCORER_HEIGHT_IN + HATCH_MID_HIGH_OFFSET_IN),// The point at which we need to position the elevator to score a hatch cover on the high position
		CARGO_MID(47 - HATCH_SCORER_HEIGHT_IN + HATCH_MID_HIGH_OFFSET_IN + CARGO_OFFSET_IN),// The point at which we need to position the elevator to score a cargo on the middle position
		CARGO_HIGH(70 - HATCH_SCORER_HEIGHT_IN + HATCH_MID_HIGH_OFFSET_IN + CARGO_OFFSET_IN),// The point at which we need to position the elevator to score a cargo on the high position		
		;
        public final double heightInches;
        private ElevatorHoldPoint(double heightInches) {
            this.heightInches = heightInches;
        }
	};

	public Elevator(boolean realHardware) {
		super();
		// Set up the digital IO object to read the home switch
		homeSwitch = new DigitalInput(Constants.ELEVATOR_HOME_SWITCH_DIO_NUM);
		
		if(realHardware) {
			followerDriver = new AdjustedTalon(Constants.ELEV_FOLLOWER_DRIVER);
			leaderDriver = new AdjustedTalon(Constants.ELEV_LEADER_DRIVER);
		} else {
			followerDriver = new FakeTalon();
			leaderDriver = new FakeTalon();
		}

		// Configure the left talon to follow the right talon, but backwards
		followerDriver.setInverted(false); // Inverted here refers to the output.  It flips the voltage
		followerDriver.set(ControlMode.Follower, Constants.ELEV_LEADER_DRIVER);
		
		// Configure the right talon for closed loop control
		leaderDriver.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_IDX,
				Constants.CAN_TIMEOUT_MS);
		leaderDriver.setSensorPhase(false);
		leaderDriver.config_kF(Constants.PID_IDX, K_F, Constants.CAN_TIMEOUT_MS);
		leaderDriver.config_kP(Constants.PID_IDX, K_P, Constants.CAN_TIMEOUT_MS);
		leaderDriver.config_kI(Constants.PID_IDX, K_I, Constants.CAN_TIMEOUT_MS);
		leaderDriver.config_kD(Constants.PID_IDX, K_D, Constants.CAN_TIMEOUT_MS);
		// Prevent Integral Windup.
		// Whenever the control loop error is outside this zone, zero out the I term
		// accumulator.
		leaderDriver.config_IntegralZone(Constants.PID_IDX, I_ZONE, Constants.CAN_TIMEOUT_MS);

		// Configure soft limit at top
		//leaderDriver.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT_MS);
		//leaderDriver.configForwardSoftLimitThreshold((int) SOFT_FWD_LIMIT, Constants.CAN_TIMEOUT_MS);
		//leaderDriver.configReverseSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);
		
		//Tell talon a limit switch is connected
		// leaderDriver.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.CAN_TIMEOUT_MS);
		leaderDriver.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, Constants.CAN_TIMEOUT_MS);
		leaderDriver.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, Constants.CAN_TIMEOUT_MS);

		// Send the initial PID constant values to the smartdash
		// SmartDashboard.putNumber(pLabel, K_P);
		// SmartDashboard.putNumber(iLabel, K_I);
		// SmartDashboard.putNumber(dLabel, K_D);
		leaderDriver.set(ControlMode.PercentOutput, 0);
	}

	public void checkAndApplyHomingSwitch() {
		// Pin floats high by default, due to an internal pull-up resistor.
		// When the magnet gets close enough to the reed switch, the pin is
		// connected to ground. Thus, get() starts returning false.
		if (elevatorIsHome() || Robot.oi.getElevatorHomeButtonPressed()) {
			setCurrentPosToZero();
		}
	}
	
	private boolean elevatorIsHome() {
		return !homeSwitch.get();
	}

	/**
	 * Update the Talon's definition of zero to be its present position.
	 * 
	 * SAFETY NOTE: The talon tracks position changes as long as it has power, no
	 * matter how many times you restart the code. You can easily turn a motor a
	 * zillion times, and the talon is counting each and every one of those
	 * revolutions.
	 * 
	 * If you then command the motor to seek position 0, it will make all haste to
	 * turn it back as many revolutions as you've turned the shaft since power-on.
	 * 
	 * This method needs to be called at the beginning of a match when the elevator
	 * is known to be at its bottom, and it should be called whenever the sensor
	 * confirms that it's hit bottom.
	 * 
	 */
	private void setCurrentPosToZero() {
		leaderDriver.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.CAN_TIMEOUT_MS);
	}
	
	public void brake(boolean isEnabled) {
		leaderDriver.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		followerDriver.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new ManuallyControlElevator());
	}

	public void log() {
		SmartDashboard.putNumber("Elevator Speed", Robot.oi.getElevatorSpeed());
		SmartDashboard.putBoolean("Elevator is home?", elevatorIsHome());
		SmartDashboard.putNumber("Elevator encoder value:",
				leaderDriver.getSelectedSensorPosition(Constants.PID_IDX));
		SmartDashboard.putNumber("Elevator height in feet:", getElevatorHeightFeet());
	}

	/**
	 * Command the elevator to run at a specific speed.
	 * Won't drive downward if the homing switch is tripped.
	 * 
	 * @param value
	 *            - the throttle value to apply to the motors, between -1 and +1
	 */
	public void setElevatorSpeed(double value) {
		if(!elevatorIsHome() || value > 0) {
			// Either the elevator is above the deck, or being driven upward.
			// This is the normal state
			leaderDriver.set(ControlMode.PercentOutput, value);
			SmartDashboard.putNumber("Elevator speed drive", value);
		} else {
			// The elevator is on the deck and they're trying to drive down.
			// Don't do that.
			leaderDriver.set(ControlMode.PercentOutput, 0);
			SmartDashboard.putNumber("Elevator speed drive", 0);
		}
	}

	/**
	 * Command the elevator to a specific height
	 * 
	 * @param feet
	 *            - the target height in feet up from lowest possible position
	 */
	public void setElevatorHeight(double feet) {
		leaderDriver.set(ControlMode.Position, feet * TICKS_PER_FOOT);
	}

	/**
	 * Command the elevator to a specific position
	 * @param point
	 */
	public void seekHoldPoint(ElevatorHoldPoint point) {
		if(point == ElevatorHoldPoint.NONE) {
			// No action commanded
		} else if (point == ElevatorHoldPoint.HERE) {
			// Hold wherever the elevator is right now
			setElevatorHeight(getElevatorHeightFeet());
		} else {
			setElevatorHeight(point.heightInches / 12);
		}
	}

	/**
	 * Gets the elevator's present actual height, in feet. This may not be the same
	 * as the last height commanded to the elevator.
	 * 
	 * @return 0 for against the floor, about 5.91 for its highest extent.
	 */
	public double getElevatorHeightFeet() {
		//System.out.println("The elevator height encoder ticks are " + leaderDriver.getSelectedSensorPosition(Constants.PID_IDX));
		return leaderDriver.getSelectedSensorPosition(Constants.PID_IDX) / TICKS_PER_FOOT;
	}

	/**
	 * Gets the elevator's target height, in feet. This may not be the same as the
	 * actual height of the elevator.
	 * 
	 * @return 0 for against the floor, about 5.91 for its highest extent.
	 */
	public double getTargetHeightFeet() {
		if (leaderDriver instanceof AdjustedTalon) {
			return ((AdjustedTalon) leaderDriver).getClosedLoopTarget(Constants.PID_IDX) / TICKS_PER_FOOT;
		} else {
			return 0;
		}
	}

	/**
	 * Commands the motor to stop driving itself, but not to disable. Turns off
	 * closed-loop control completely.
	 */
	public void stopMotor() {
		leaderDriver.set(ControlMode.PercentOutput, 0.0);
	}

	/**
	 * Retrieve the values of P, I and D from the smartdashboard and apply them to
	 * the motor controllers.
	 */
	public void pullPidConstantsFromSmartDash() {
		// Retrieve
		K_P = SmartDashboard.getNumber(pLabel, K_P);
		K_I = SmartDashboard.getNumber(iLabel, K_I);
		K_D = SmartDashboard.getNumber(dLabel, K_D);

		// Apply
		leaderDriver.config_kP(Constants.PID_IDX, K_P, Constants.CAN_TIMEOUT_MS);
		leaderDriver.config_kI(Constants.PID_IDX, K_I, Constants.CAN_TIMEOUT_MS);
		leaderDriver.config_kD(Constants.PID_IDX, K_D, Constants.CAN_TIMEOUT_MS);
	}

	public boolean isOnTarget() {
		// leader.configNeutralDeadband(percentDeadband, timeoutMs);
		return Math.abs(getElevatorHeightFeet()
				- getTargetHeightFeet()) < (Constants.ELEVATOR_ON_TARGET_THRESHOLD_INCHES / 12.0);
	}
}
