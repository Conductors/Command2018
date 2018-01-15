package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.Robot;
import org.usfirst.frc.team4580.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Baseline extends Command {
	Encoder left;
	Encoder right;
	double distance;
    public Baseline() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	right = new Encoder(2,3,true,EncodingType.k4X);
    	left = new Encoder(0, 1,true,EncodingType.k4X);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//CORRECT THESE
    	left.reset();
    	right.reset();
    	right.setDistancePerPulse(.06981311);
    	left.setDistancePerPulse(.06981311);
    	distance = RobotMap.baseLength - RobotMap.botLength;
    	
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveBase.tankDrive(1.0, 1.0); 
    	SmartDashboard.putNumber("Left Encoder Dist", left.getDistance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (left.getDistance() > distance) return true;
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
