package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.hgroundloader.CombinedControlHGroundLoader;
import frc.robot.components.AdjustedTalon;
import frc.robot.components.FakeTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchGroundLoader extends Subsystem {
	// Control parameters for wrist
	private static final double K_F = 0.0; // Don't use in position mode.
	private double K_P = 0.5 * 1023.0 / 200.0; // Respond to an error of 200 with 50% throttle
	private double K_I = 0; //0.01 * K_P;
	private double K_D = 0; // 40.0 * K_P;
	private static final int I_ZONE = 200; // In closed loop error units
	private final String pLabel = "HGL_Wrist P";
	private final String iLabel = "HGL Wrist I";
	private final String dLabel = "HGL Wrist D";
	public static final double DEGREES_FULL_RANGE = 90;
	public static final double ENCODER_TICKS_FULL_RANGE = 1075; // measured 2019-2-19
	private static final double TICKS_PER_DEG = ENCODER_TICKS_FULL_RANGE / DEGREES_FULL_RANGE;

	public static double HANDOFF_DEG = -10.0;
	public static double UP_DEG = 0.0;
	public static double COLLECT_DEG = 90.0;
	
	// Motor controller for the intake rollers
	private IMotorControllerEnhanced intakeDriver;
	// Pitch up/down motor
	private IMotorControllerEnhanced wristDriver;

	public HatchGroundLoader(boolean realHardware) {
		super();
		
		if(realHardware) {
			intakeDriver = new AdjustedTalon(Constants.HGL_INTAKE);
			wristDriver  = new AdjustedTalon(Constants.HGL_WRIST);
			wristDriver.enableCurrentLimit(true);
			wristDriver.configContinuousCurrentLimit((int)Constants.HGL_WRIST_CUR_LIMIT_AMPS,Constants.CAN_TIMEOUT_MS);			
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
		wristDriver.set(ControlMode.PercentOutput, 0);
		wristDriver.setSelectedSensorPosition(0, Constants.PID_IDX, Constants.CAN_TIMEOUT_MS);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new CombinedControlHGroundLoader());
	}
	
	public void log() {
		SmartDashboard.putNumber("HGL intake current (A)",intakeDriver.getOutputCurrent());
		SmartDashboard.putNumber("HGL intake current spike",getIntakeCurrentSpike() ? 1: 0);
		SmartDashboard.putNumber("HGL wrist position (deg)",getWristAngleDeg());
		SmartDashboard.putNumber("HGL wrist angle (ticks)",getWristAngleDeg()*TICKS_PER_DEG);
		SmartDashboard.putString("HGL wrist mode",wristDriver.getControlMode().toString());
		SmartDashboard.putNumber("HGL wrist current (A)",wristDriver.getOutputCurrent());
	}	

	/**
	 * Query current for intake and determine if greater than threshold
	 */	
	public boolean getIntakeCurrentSpike() {
		return intakeDriver.getOutputCurrent() > Constants.HGL_INTAKE_CUR_SPIKE_AMPS;
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
		wristDriver.set(ControlMode.PercentOutput, upwardSpeed * 0.2);	
	}

	/**
	 * Gets the wrist's present actual angle, in degrees. 
	 * 
	 * @return 0 for fully vertical and -90 for all the way down.
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
		wristDriver.set(ControlMode.PercentOutput, 0.0);
	}

	/**
	 * Gets the wrist's target angle, in degrees.
	 * 
	 * @return 0 for vertical, about -90 degrees for fully down
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
				- getTargetAngleDeg()) < (Constants.HGL_ON_TARGET_DEG);
	}	

}