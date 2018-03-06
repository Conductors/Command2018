package org.usfirst.frc.team4580.robot.subsystems;

import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    DigitalInput topSwitch;
    DigitalInput bottomSwitch;
    WPI_TalonSRX lift1;
    WPI_TalonSRX lift2;
    public Lift() {
    	topSwitch = new DigitalInput(4);
    	bottomSwitch = new DigitalInput(5);
    	lift1 = new WPI_TalonSRX(RobotMap.elevator1);
    	lift2 = new WPI_TalonSRX(RobotMap.elevator2);
    }
    public void fullRaise() {
    	while (!topSwitch.get()) {
    		lift1.set(1);
    		lift2.set(1);
    	}
    	lift1.set(0);
    	lift2.set(0);
    }
    public void fullLower() {
    	while (!bottomSwitch.get() && !topSwitch.get()) {
    		lift1.set(-1);
    		lift2.set(-1);
    	}
    	lift1.set(0);
    	lift2.set(0);
    }
}

