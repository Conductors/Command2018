package org.usfirst.frc.team4580.robot.subsystems;

import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveBase extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	TalonSRX leftFront = new TalonSRX(RobotMap.leftFrontTal);
	TalonSRX leftBack = new TalonSRX(RobotMap.leftBackTal);
	TalonSRX rightFront = new TalonSRX(RobotMap.rightFrontTal);
	TalonSRX rightBack = new TalonSRX(RobotMap.rightBackTal);
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }
    public void tankDrive(double firstVal,double secondVal) {
    	leftFront.set(ControlMode.Velocity, firstVal);
    	rightFront.set(ControlMode.Velocity, secondVal);
    	leftBack.set(ControlMode.Velocity, firstVal);
    	rightBack.set(ControlMode.Velocity, secondVal);
    }
}

