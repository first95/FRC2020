package frc.robot.commands.compound;

// Import your needed commands!
import frc.robot.commands.Nothing;
import edu.wpi.first.wpilibj.command.CommandGroup;

// Then extend command group as it is not a single command but a group!
public class CompoundExample extends CommandGroup {

    // Constructor that will run commands!
	public CompoundExample() {
        // A sequential command is one after another, only one will run at a time
        addSequential(new Nothing());

        // A parallell command is something that can run with another command simutaniously
        addParallel(new Nothing());
	}
    
    // Once all commands above have been executed then it will run end!
    // Here you can reset robot postion, etc...
    // I belive there is something called stop(), which does other things than end()
	@Override
	protected void end() {
		super.end();
	}
	
}
