package org.usfirst.frc.team4580.robot.subsystems;

import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveBase extends Subsystem implements PIDOutput {
	static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 2.0f;
    double rotateToAngleRate;
    PIDController turnController;
    WPI_TalonSRX leftFront;
	WPI_TalonSRX leftBack;
	WPI_TalonSRX rightFront;
	WPI_TalonSRX rightBack;
	SpeedControllerGroup leftSide;
	SpeedControllerGroup rightSide;
	DifferentialDrive myRobot;
	AHRS navx;
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	public DriveBase() {
		leftFront = new WPI_TalonSRX(RobotMap.leftFrontTal);
		leftBack = new WPI_TalonSRX(RobotMap.leftBackTal);
		rightFront = new WPI_TalonSRX(RobotMap.rightFrontTal);
		rightBack = new WPI_TalonSRX(RobotMap.rightBackTal);
		leftSide = new SpeedControllerGroup(leftFront, leftBack);
		rightSide = new SpeedControllerGroup(rightFront, rightBack);
		myRobot = new DifferentialDrive(leftSide, rightSide );
		myRobot.setSafetyEnabled(false);
		myRobot.setExpiration(.1);
		turnController = new PIDController(kP, kI, kD, kF, navx, this);
    	turnController.setInputRange(-180.0f, 180.0f);
    	turnController.setOutputRange(-1.0, 1.0);
    	turnController.setAbsoluteTolerance(kToleranceDegrees);
    	turnController.setContinuous(true);
    	try {
    		navx = new AHRS(SerialPort.Port.kMXP);
    	} catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
	}

	public void tankDrive(double firstVal,double secondVal) {
		myRobot.tankDrive(firstVal, secondVal);
	}
	public void arcadeDrive(double firstVal, double secondVal) {
		myRobot.arcadeDrive(firstVal, secondVal);
	}
    public void rotate(double angle) {
    		navx.reset();
    		while (!(navx.getAngle() + 4 > angle && navx.getAngle() - 4 < angle)) {
	    		if (angle > navx.getAngle()) {
	    			tankDrive(RobotMap.rotateVelocity,-1 * RobotMap.rotateVelocity);
	    			SmartDashboard.putNumber("Angle", navx.getAngle());
	    		} else {
	    			tankDrive(-1 * RobotMap.rotateVelocity,RobotMap.rotateVelocity);
	    			SmartDashboard.putNumber("Angle", navx.getAngle());
	    		}
    		}
    		myRobot.tankDrive(0, 0);
    }
    public void PIDRotate(double angle) {
    	turnController.setSetpoint(angle);
    	turnController.enable();
    	LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
    	
    	
    }
    public void resetNavx() {
    	try {
        	navx = new AHRS(SerialPort.Port.kMXP);
    		navx.reset();
    	} catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    }
    public void pidWrite(double output) {
    	rotateToAngleRate = output;
    	arcadeDrive(0, rotateToAngleRate);
    }
    public double getAngle() {
    	return navx.getAngle();
    }
}
