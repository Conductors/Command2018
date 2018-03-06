package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MiddleLine extends CommandGroup {

    public MiddleLine() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	addSequential(new GoDistance(RobotMap.botLength*2));
    	addSequential(new Rotate(-90.0));
    	addSequential(new GoDistance(264.0/2.0 - RobotMap.botLength/2.0));
    	addSequential(new Rotate(90.0));
    	addSequential(new GoDistance(140.0));
    }
}
