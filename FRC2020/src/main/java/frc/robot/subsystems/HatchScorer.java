package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.hscorer.ManuallyControlHatchScorer;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls mechanism to score hatches.
 */
public class HatchScorer extends Subsystem {

    // The solenoids for the cylinders that open the scorer and that push the scorer
    // Named to allow for future possibility of adding one more solenoid per motion
    // to have slow/fast options
    private Solenoid openA, pushA;

    public HatchScorer(boolean realHardware) {
        if(realHardware) {
            // False means closed and true means open so default is closed when robot off
            openA = new Solenoid(Constants.HS_OPEN_A);

            // False means retracted, and true means extended
            pushA = new Solenoid(Constants.HS_PUSH_A);
        } else {
            openA = null;
            pushA = null;
        }
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManuallyControlHatchScorer());
    }

    /**
     * Set the Hatch scorer to open (grasping) or closed (not grasping)
     * @param open true to grasp, false to release
     */
    public void openHS(boolean open) {
        if(openA != null) {
            openA.set(open);
        }
    }

    /**
     * Toggle the open/closed state of the hatch scorer
     */
    public void toggleOpenHS() {
        if(openA != null) {
            openA.set(!openA.get());
        }
    }

    /**
     * Set the Hatch Scorer to extended or retracted
     * @param push true for extended, false for retracted
     */
    public void pushHS(boolean push) {
        if(pushA != null) {
            pushA.set(push);
        }
    }

    /**
     * Toggle the extend/retract state of the hatch scorer
     */
    public void togglePushHS() {
        if(pushA != null) {
            pushA.set(!pushA.get());
        }
    }
}
