package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.Robot;
import org.usfirst.frc.team4580.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class GoDistance extends Command implements PIDOutput{

	double distance;
	PIDController distController;
	static final double lP = 1.4;
    static final double lI = 0.00;
    static final double lD = 0.00;
    static final double lF = 0.00;
    static final double lAbsolute = .1;
    Encoder left;
    Encoder right;
    public GoDistance(double dist) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	distance = (dist - RobotMap.botLength)/3.0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	left = Robot.driveBase.getLeftRaw();
    	right = Robot.driveBase.getRightRaw();
    	distController = new PIDController(lP,lI,lD,lF,right,this);
		distController.setInputRange(-500.0f, 500.0f);
		distController.setOutputRange(-.8, .8);
    	distController.setAbsoluteTolerance(lAbsolute);
    	distController.setContinuous(true);
    	
    	left.reset();
    	right.reset();
    	distController.enable();
    	SmartDashboard.putData(distController);
    	
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	distController.setSetpoint(distance);
    	SmartDashboard.putNumber("Right Encoder Dist", right.getDistance());
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
    	distController.disable();
    	Robot.driveBase.arcadeDrive(0, 0);
    	System.out.println("Driving done");
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
    	end();
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		Robot.driveBase.arcadeDrive(output, 0);
	} 
}
