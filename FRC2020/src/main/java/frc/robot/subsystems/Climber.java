package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.climber.SpeedControlClimber;
import frc.robot.components.AdjustedTalon;
import frc.robot.components.FakeTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

    // Motor controller for the climber
    private IMotorControllerEnhanced driver;
    // Solenoid for the climber
    private Solenoid sol;

    public Climber(boolean realHardware) {
        super();

        if (realHardware) {
            sol = new Solenoid(Constants.CLIMBER_SOL_REAR);
            driver = new AdjustedTalon(Constants.CLIMBER_DRIVER);
        } else {
            sol = null;
            driver = new FakeTalon();
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SpeedControlClimber());
    }

    public void log() {

    }

    /**
     * Set speed of the driver
     * 
     * @param throttle 1.0 for fully down (?), -1.0 for fully up (?), 0.0 for
     *                 stationary
     */
    public void setSpeed(double throttle) {
        driver.set(ControlMode.PercentOutput, throttle);
    }

    /**
     * Set whether deploy or retract climber skids
     * 
     * @param deploy true to extend skids and false to retract them
     */
    public void deploySkids(boolean deploy) {
        if (sol != null) {
            sol.set(deploy);
        }
    }

    /**
     * Toggle skids between deploy and retract
     */
    public void toggleSkids() {
        if (sol != null) {
            sol.set(!sol.get());
        }
    }
}