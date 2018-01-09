package org.usfirst.frc.team4580.robot.subsystems;

import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class DriveBase extends Subsystem {
	AHRS navx = new AHRS(SerialPort.Port.kMXP);
	static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
	WPI_TalonSRX leftFront = new WPI_TalonSRX(RobotMap.leftFrontTal);
	WPI_TalonSRX leftBack = new WPI_TalonSRX(RobotMap.leftBackTal);
	WPI_TalonSRX rightFront = new WPI_TalonSRX(RobotMap.rightFrontTal);
	WPI_TalonSRX rightBack = new WPI_TalonSRX(RobotMap.rightBackTal);
	SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftBack);
	SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightBack);
	DifferentialDrive myRobot = new DifferentialDrive(leftSide, rightSide);
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }


	public void tankDrive(double firstVal,double secondVal) {
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
    	//PIDController turnControl = new PIDController(kP, kI, kD, kF, navx, myRobot);
    	
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
}
