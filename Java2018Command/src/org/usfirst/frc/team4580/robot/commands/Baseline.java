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
	static final double lP = 0.3;
    static final double lI = 0.00;
    static final double lD = 0.00;
    static final double lF = 0.00;
    static final double lAbsolute = .05;
	static final double encoderDPP = RobotMap.encoderDPP;
    public Baseline() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	right = new Encoder(2,3,true,EncodingType.k4X);
    	left = new Encoder(0, 1,true,EncodingType.k4X);
    	left.reset();
    	right.reset();
    	right.setDistancePerPulse(6.0*Math.PI/1440.0);
    	left.setDistancePerPulse(6.0*Math.PI/1440.0);
    	distController = new PIDController(lP,lI,lD,lF,left,this);
		distController.setInputRange(-500.0f, 500.0f);
		distController.setOutputRange(-1.0, 1.0);
    	distController.setAbsoluteTolerance(lAbsolute);
    	distController.setContinuous(true);
    	distance = RobotMap.baseLength - RobotMap.botLength;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//CORRECT THESE
    	left.reset();
    	right.reset();
    	right.setDistancePerPulse(encoderDPP);
    	left.setDistancePerPulse(encoderDPP);
    	SmartDashboard.putData(distController);
    	
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	distController.setSetpoint(distance);
    	distController.enable();

    	SmartDashboard.putNumber("Left Encoder Dist", left.getDistance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (distController.onTarget()) {
    		return true;
    	} return false;
    }
    public void PIDEnable(boolean enable) {
    	if (enable) {
    		distController.enable();
    	} else {
    		distController.disable();
    	}
    }
    // Called once after isFinished returns true
    protected void end() {
    	//distController.disable();
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
