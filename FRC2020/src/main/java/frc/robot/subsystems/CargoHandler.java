package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.cargohandler.CombinedControlCargoHandler;
import frc.robot.components.AdjustedTalon;
import frc.robot.components.FakeTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoHandler extends Subsystem {
	// Control parameters for wrist
	private static final double K_F = 0.0; // Don't use in position mode.
	private double K_P = 0.5 * 1023.0 / 200.0; // Respond to an error of 200 with 50% throttle
	private double K_I = 0; //0.01 * K_P;
	private double K_D = 0; // 40.0 * K_P;
	private static final int I_ZONE = 200; // In closed loop error units
	private final String pLabel = "CL_Wrist P";
	private final String iLabel = "CL Wrist I";
	private final String dLabel = "CL Wrist D";
	public static final double DEGREES_FULL_RANGE = 135; // needs to be measured
	public static final double ENCODER_TICKS_FULL_RANGE = 1500; // needs to be measured
	private static final double TICKS_PER_DEG = ENCODER_TICKS_FULL_RANGE / DEGREES_FULL_RANGE;

	
	// Motor controller for the intake rollers
	private IMotorControllerEnhanced intakeDriver;
	// Pitch up/down motor
	private IMotorControllerEnhanced wristDriver;

	public CargoHandler(boolean realHardware) {
		super();
		
		if(realHardware) {
			intakeDriver = new AdjustedTalon(Constants.CARGO_HANDLER_INTAKE);
			wristDriver  = new AdjustedTalon(Constants.CARGO_HANDLER_WRIST);
		} else {
			intakeDriver = new FakeTalon();
			wristDriver  = new FakeTalon();
		}
		
		// Configure the wrist talon for closed loop control
		wristDriver.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_IDX,
				Constants.CAN_TIMEOUT_MS);
		wristDriver.setSensorPhase(false);
		wristDriver.config_kF(Constants.PID_IDX, K_F, Constants.CAN_TIMEOUT_MS);
		wristDriver.config_kP(Constants.PID_IDX, K_P, Constants.CAN_TIMEOUT_MS);
		wristDriver.config_kI(Constants.PID_IDX, K_I, Constants.CAN_TIMEOUT_MS);
		wristDriver.config_kD(Constants.PID_IDX, K_D, Constants.CAN_TIMEOUT_MS);
		// Prevent Integral Windup.
		// Whenever the control loop error is outside this zone, zero out the I term
		// accumulator.
		wristDriver.config_IntegralZone(Constants.PID_IDX, I_ZONE, Constants.CAN_TIMEOUT_MS);

		// Set the wrist initially to 0 percent output and assume starting position is 0 (fully up)
		wristDriver.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.CAN_TIMEOUT_MS);
		wristDriver.set(ControlMode.Position, 0);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new CombinedControlCargoHandler());
	}
	
	public void log() {
		SmartDashboard.putNumber("Cargo intake current (A)",intakeDriver.getOutputCurrent());
		SmartDashboard.putNumber("Cargo intake current spike",getIntakeCurrentSpike() ? 1: 0);
		SmartDashboard.putNumber("Cargo handler wrist position (deg)",getWristAngleDeg());
		SmartDashboard.putNumber("Cargo handler wrist angle (ticks)",getWristAngleDeg()*TICKS_PER_DEG);
		SmartDashboard.putString("Cargo handler wrist mode",wristDriver.getControlMode().toString());
	}	

	/**
	 * Query current for intake and determine if greater than threshold
	 */	
	public boolean getIntakeCurrentSpike() {
		return intakeDriver.getOutputCurrent() > Constants.CARGO_HANDLER_INTAKE_CUR_SPIKE_AMPS;
	}

	/**
	 * Set speed of the intake rollers
	 * @param inwardThrottle 1.0 for fully inward, -1.0 for fully outward, 0.0 for stationary
	 */
	public void setIntakeSpeed(double inwardThrottle) {
		intakeDriver.set(ControlMode.PercentOutput, inwardThrottle);
	}

	/**
	 * Set speed of the wrist movement
	 * @param upwardSpeed 1.0 for fully upward, -1.0 for fully downward, 0.0 for stationary
	 */
	public void setWristPitchSpeed(double upwardSpeed) {
		// Slow it way the hell down for starters
		// and reverse the direction so up is up and down is down
		wristDriver.set(ControlMode.PercentOutput, -upwardSpeed * 0.8);	
	}

	/**
	 * Gets the wrist's present actual angle, in degrees. 
	 * 
	 * @return 0 for fully vertical and ~135 for all the way down.
	 */
	public double getWristAngleDeg() {
		return wristDriver.getSelectedSensorPosition(Constants.PID_IDX) / TICKS_PER_DEG;
	}

	/**
	 * Command the wrist to a particular angle
	 * 
	 * @param degrees
	 *            - the target in degrees from vertical
	 */
	public void setWristAngleDeg(double degrees) {
		wristDriver.set(ControlMode.Position, degrees * TICKS_PER_DEG);
	}	

	/**
	 * Commands the wrist motor to stop driving itself, but not to disable. Turns off
	 * closed-loop control completely.
	 */
	public void stopWrist() {
        //System.out.println("CargoHandler.stopWrist()");
		wristDriver.set(ControlMode.PercentOutput, 0.0);
	}

	/**
	 * Gets the wrist's target angle, in degrees.
	 * 
	 * @return 0 for vertical, about 135 degrees for fully down
	 */
	public double getTargetAngleDeg() {
		if (wristDriver instanceof AdjustedTalon) {
			return ((AdjustedTalon) wristDriver).getClosedLoopTarget(Constants.PID_IDX) / TICKS_PER_DEG;
		} else {
			return 0;
		}
	}

	public boolean isOnTarget() {
		return Math.abs(getWristAngleDeg()
				- getTargetAngleDeg()) < (Constants.CARGO_HANDLER_ON_TARGET_DEG);
	}	

}