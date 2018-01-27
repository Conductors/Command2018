package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rotateneg90 extends Command {

    public Rotateneg90() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveBase.resetNavx();
    	Robot.driveBase.PIDRotate(-90);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Robot.driveBase.isDone()) {
    		Robot.driveBase.PIDEnable(false);
    		return true;
    	} return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
