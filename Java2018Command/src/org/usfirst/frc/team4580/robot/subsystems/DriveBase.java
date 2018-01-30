package org.usfirst.frc.team4580.robot.subsystems;

import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveBase extends Subsystem implements PIDOutput {
	static final double kP = 0.06;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kTolerancePercent = .5;

    double rotateToAngleRate;
    PIDController turnController;
    PIDController distController;
    WPI_TalonSRX leftFront;
	WPI_TalonSRX leftBack;
	WPI_TalonSRX rightFront;
	WPI_TalonSRX rightBack;
	SpeedControllerGroup leftSide;
	SpeedControllerGroup rightSide;
	DifferentialDrive myRobot;
	Encoder right;
	Encoder left;
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
    	navx = new AHRS(I2C.Port.kMXP);
		turnController = new PIDController(kP, kI, kD, kF, navx, this);
    	navx.reset();
		turnController.setInputRange(-500.0f, 500.0f);
    	turnController.setOutputRange(-1.0, 1.0);
    	turnController.setPercentTolerance(kTolerancePercent);
    	turnController.setContinuous(true);

    	
    	
	}

	public void tankDrive(double firstVal,double secondVal) {
		myRobot.tankDrive(firstVal, secondVal);
	}
	public void arcadeDrive(double firstVal, double secondVal) {
		myRobot.arcadeDrive(firstVal, secondVal);
    }
    public void PIDRotate(double angle) {
    	navx.reset();
    	turnController.setSetpoint(angle);
    	turnController.enable();
    	SmartDashboard.putNumber("NavX", navx.getAngle());
    	SmartDashboard.putData("Left Encoder PID",turnController);
    	
    }
    public void PIDEnable(boolean enable) {
    	if (enable) {
    		turnController.enable();
    	} else {
    		turnController.disable();
    	}
    }
    public boolean isDone() {
    	return turnController.onTarget();
    }
    public void resetNavx() {
    		navx.reset();
    }
    public void pidWrite(double output) {
    	rotateToAngleRate = output;
    	arcadeDrive(0, rotateToAngleRate);
    	SmartDashboard.putNumber("NavX", navx.getAngle());
    }
    public double getAngle() {
    	return navx.getAngle();
    }
}
