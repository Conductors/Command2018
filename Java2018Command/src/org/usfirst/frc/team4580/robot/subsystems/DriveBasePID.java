/*package org.usfirst.frc.team4580.robot.subsystems;

import org.usfirst.frc.team4580.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 
public class DriveBase extends PIDSubsystem {

    // Initialize your subsystem here
    public DriveBase() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return navx.getAngle();
    }


	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	RobotDrive myRobot = new RobotDrive(RobotMap.leftFrontTal,RobotMap.leftBackTal,RobotMap.rightFrontTal,RobotMap.rightBackTal);
	myRobot.SetInvertedMotor(RobotDrive.kFrontLeftMotor, true);	// invert left side motors
    myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	AHRS navx = new AHRS(SerialPort.Port.kMXP);
	static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
	public void tankDrive(double firstVal,double secondVal) {
    	leftFront.set(ControlMode.Velocity, firstVal);
    	rightFront.set(ControlMode.Velocity, secondVal);
    	leftBack.set(ControlMode.Velocity, firstVal);
    	rightBack.set(ControlMode.Velocity, secondVal); 
    	myRobot.tankDrive(firstVal, secondVal);
    }
    public void rotate(double angle) {
    	try {
    		while (!(navx.getAngle() + 1 > angle && navx.getAngle() - 1 < angle)) {
	    		if (angle > navx.getAngle()) {
	    			tankDrive(RobotMap.rotateVelocity,-1 * RobotMap.rotateVelocity);
	    		} else {
	    			tankDrive(-1 * RobotMap.rotateVelocity,RobotMap.rotateVelocity);
	    		}
    		}
    		
    	} catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    }
    public void PIDRotate(double angle) {

    	try {

    	} catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    }
    public void resetNavx() {
    	try {
    		navx.reset();
    	} catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    }
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	my
    }
}
*/