package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Rotate extends Command implements PIDOutput {
	double angle;
    double rotateToAngleRate;
	static final double kP = 0.022;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kAbsoluteTol = .08;
    PIDController turnController;
    AHRS navx;
    public Rotate(double goAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	angle = goAngle;
    	navx = Robot.driveBase.getNavx();
    	navx.reset();
    	turnController = new PIDController(kP, kI, kD, kF, navx, this);
		turnController.setInputRange(-500.0f, 500.0f);
    	turnController.setOutputRange(-.8, .8);
    	turnController.setPercentTolerance(kAbsoluteTol);
    	turnController.setContinuous(true);
    	
    	

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	navx.reset();
    	turnController.disable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	turnController.setSetpoint(angle);
    	turnController.enable();
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return turnController.onTarget();
    }
    // Called once after isFinished returns true
    protected void end() {
    	turnController.disable();
    	navx.reset();
    	System.out.println(navx.getAngle());
    	Robot.driveBase.arcadeDrive(0, 0);
    	System.out.println("Rotation done");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    public void pidWrite(double output) {
    	rotateToAngleRate = output;
    	Robot.driveBase.arcadeDrive(0, rotateToAngleRate);
    }
}
