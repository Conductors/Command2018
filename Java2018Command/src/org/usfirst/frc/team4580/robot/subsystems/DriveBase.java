package org.usfirst.frc.team4580.robot.subsystems;

import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveBase extends Subsystem {

	static final double encoderDPP = RobotMap.encoderDPP;
AHRS navx;
    PIDController distController;
    WPI_TalonSRX leftFront;
	WPI_TalonSRX leftBack;
	WPI_TalonSRX rightFront;
	WPI_TalonSRX rightBack;
	SpeedControllerGroup leftSide;
	SpeedControllerGroup rightSide;
	//Spark upDown;
	DifferentialDrive myRobot;
	Encoder right;
	Encoder left;
	DoubleSolenoid shift;
	ADXRS450_Gyro gyro;
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	public DriveBase() {

		leftFront = new WPI_TalonSRX(RobotMap.leftFrontTal);
		leftBack = new WPI_TalonSRX(RobotMap.leftBackTal);
		rightFront = new WPI_TalonSRX(RobotMap.rightFrontTal);
		rightBack = new WPI_TalonSRX(RobotMap.rightBackTal);
		leftSide = new SpeedControllerGroup(leftFront,leftBack);
		rightSide = new SpeedControllerGroup(rightFront,rightBack);
		myRobot = new DifferentialDrive(leftSide, rightSide );
		//upDown = new Spark(RobotMap.upDown);
    	right = new Encoder(2,3,true,EncodingType.k4X);
    	left = new Encoder(0, 1,false,EncodingType.k4X);
    	left.reset();
    	right.reset();
		myRobot.setSafetyEnabled(false);
		myRobot.setExpiration(.5);
		navx = new AHRS(SPI.Port.kMXP,(byte)100);
    	right.setDistancePerPulse(encoderDPP);
    	left.setDistancePerPulse(encoderDPP);
    	//gyro = new ADXRS450_Gyro();
    	shift = new DoubleSolenoid(2,4);
	}

	public void tankDrive(double firstVal,double secondVal) {
		myRobot.tankDrive(firstVal, secondVal);
	}
	public void arcadeDrive(double firstVal, double secondVal) {
		myRobot.arcadeDrive(firstVal, secondVal);
    }
	/*public void setUpDown(double val) {
		upDown.set(val);
	} */
	public DoubleSolenoid getShift() {
		return shift;
	}
    public void resetEnc(char side) {
    	if (side == 'r') {
    		right.reset();
    	}else left.reset();
    	
    }
    public Encoder getLeftRaw() {
    	return left;
    }
    public Encoder getRightRaw() {
    	return right;
    }
    public double encoderPos(char side) {
    	if (side == 'r') {
    		return right.getDistance();
    	}
    	return left.getDistance();
    }
    public AHRS getNavx() {
    	return navx;
    }
    /*public ADXRS450_Gyro getGyro() {
    	return gyro;
    } */
}
