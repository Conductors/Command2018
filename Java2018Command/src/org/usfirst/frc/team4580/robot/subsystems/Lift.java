package org.usfirst.frc.team4580.robot.subsystems;

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
    DigitalInput stopSwitch;
    WPI_TalonSRX lift;
    public Lift() {
    	stopSwitch = new DigitalInput(0);
    	//lift = new WPI_TalonSRX(RobotMap.lift)
    }
    public void fullRaise() {
    	
    }
}

