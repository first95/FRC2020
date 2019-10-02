package frc.robot.commands.hgroundloader;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.OI.Controller;
import frc.robot.commands.Nothing;
import frc.robot.commands.Pause;
//import frc.robot.commands.RumbleCommand;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.hscorer.GrabIt;
import frc.robot.commands.hscorer.PushIt;
import frc.robot.subsystems.HatchGroundLoader;
import frc.robot.subsystems.Elevator.ElevatorHoldPoint;

public class AutoAcquire extends CommandGroup {
    public static final double AUTO_ACQUIRE_INTAKE_THROTTLE = 1.0;
    public static final double RUMBLE_TIME_S = 0.5;
    public static final double SPINUP_DURATION_S = 0.5;
    public static final double DEBOUNCE_DURATION_S = 0.25;

    public AutoAcquire() {
        this(true);
        //CombinedControlHGroundLoader.setInterrupt(true);
    }
    public AutoAcquire(boolean buzz) {
        super();
        // Spin it 
        addSequential(new SetIntakeThrottle(AUTO_ACQUIRE_INTAKE_THROTTLE));        
        // Drop it        
        addSequential(new SetWristAngle(HatchGroundLoader.COLLECT_DEG, true));
        //         like it's hot
        addSequential(new SetElevatorHeight(ElevatorHoldPoint.HATCH_HANDOFF));
        // Pull it
        addSequential(new PushIt(false));
        addSequential(new GrabIt(false));
        // Wait for it
        addSequential(new WaitForHatchDetected());
        // Stop it
        addSequential(new SetIntakeThrottle(0));
        if(buzz) {
            // Buzz it
            // addParallel(new RumbleCommand(Controller.WEAPONS, RumbleType.HIGH_PITCH, 1.0, RUMBLE_TIME_S, true));
            // addParallel(new RumbleCommand(Controller.DRIVER, RumbleType.LOW_PITCH, 1.0, RUMBLE_TIME_S, true));
        }
        // Lift it
        addSequential(new SetWristAngle(HatchGroundLoader.HANDOFF_DEG, true));
        // Wait for it
        addSequential(new Pause(DEBOUNCE_DURATION_S));
        // Grab it
        addSequential(new PushIt(true, 0.0)); // Only run execute once, making this nearly concurrent with the next
        addSequential(new GrabIt(true));
        // Lift it
        addSequential(new SetElevatorHeight(ElevatorHoldPoint.HATCH_COVER_LOW));
        addSequential(new PushIt(false));
        // Stop it (seriously, if you don't put this at the end, and you bind this sequence as a whilePressed, it will repeat while the button is held)
        addSequential(new Nothing());
    }

    @Override
    public synchronized void start() {
        System.out.println("AutoAcquire.start()");
        super.start();
    }

    @Override
    protected void execute() {
        System.out.println("AutoAcquire.execute()");
        super.execute();
    }

    @Override
    protected boolean isFinished() {
        boolean fin = super.isFinished();
        //System.out.println("AutoAcquire.isFinished() - " + fin);
        return fin;
    }
    @Override
    protected void initialize() {
        //System.out.println("AutoAcquire.initialize()");
        super.initialize();
    }

    @Override
    protected void end() {
        //System.out.println("AutoAcquire.end()");
        super.end();
    }
    @Override
    public synchronized void cancel() {
        //System.out.println("AutoAcquire.cancel()");
        super.cancel();
        Scheduler.getInstance().add(new SetWristAngle(HatchGroundLoader.UP_DEG, false));
        Scheduler.getInstance().add(new SetIntakeThrottle(0));
    }
}