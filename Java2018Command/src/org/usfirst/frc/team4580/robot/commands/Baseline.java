package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.Robot;
import org.usfirst.frc.team4580.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Baseline extends Command implements PIDOutput{
	Encoder left;
	Encoder right;
	double distance;
	PIDController distController;
	static final double lP = 0.03;
    static final double lI = 0.00;
    static final double lD = 0.00;
    static final double lF = 0.00;
    static final double lTolerancePercent = .5;
    public Baseline() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	right = new Encoder(2,3,true,EncodingType.k2X);
    	left = new Encoder(0, 1,true,EncodingType.k2X);
    	distController = new PIDController(lP,lI,lD,lF,right,this);
		distController.setInputRange(-500.0f, 500.0f);
		distController.setOutputRange(-1.0, 1.0);
    	distController.setPercentTolerance(lTolerancePercent);
    	distController.setContinuous(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//CORRECT THESE
    	left.reset();
    	right.reset();
    	right.setDistancePerPulse(0.01308995833);
    	left.setDistancePerPulse(0.01308995833);
    	distance = RobotMap.baseLength - RobotMap.botLength;

    	
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveBase.tankDrive(1.0, 1.0); 
    	SmartDashboard.putNumber("Left Encoder Dist", left.getDistance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (distController.onTarget()) {
    		distController.disable();
    		return true;
    	} return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }
	public boolean isDone() {
		return distController.onTarget();
	}
    public void PIDWrite(double output) {
    	Robot.driveBase.arcadeDrive(output, 0);
    }
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		Robot.driveBase.arcadeDrive(output, 0);
	}
}
