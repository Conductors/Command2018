package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.OI;
import org.usfirst.frc.team4580.robot.Robot;
import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TankDriveTele extends Command {
    DoubleSolenoid shift;
    DoubleSolenoid test1;
    DigitalInput topSwitch;
    DigitalInput bottomSwitch;
    WPI_TalonSRX intake;
    WPI_TalonSRX elevateOne;
    WPI_TalonSRX elevateTwo;
    Encoder left;
    Encoder right;
    AHRS navx;
    boolean fliparoo;
    boolean fliplock;
    //NetworkTable vision;	
	public TankDriveTele() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveBase);
        //intake = new WPI_TalonSRX(RobotMap.rightBackTal);
        shift = new DoubleSolenoid(2,3);
        navx = Robot.driveBase.getNavx();
        elevateOne = new WPI_TalonSRX(RobotMap.elevator1);
        elevateTwo = new WPI_TalonSRX(RobotMap.elevator2);
        left = Robot.driveBase.getLeftRaw();
        right = Robot.driveBase.getRightRaw();
        //topSwitch = new DigitalInput(4);
        //bottomSwitch = new DigitalInput(5);
        fliparoo = false;
        fliplock = true;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	//liftSwitch = new DigitalInput(0);
    	//intake = new WPI_TalonSRX(RobotMap.intake);
    	
    	Scheduler.getInstance().run();    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveBase.tankDrive(OI.getLeftSpeed() *-1, OI.getRightSpeed()*-1);
    	if (OI.getButton(1,5)) shift.set(DoubleSolenoid.Value.kForward);
    	if (OI.getButton(1,6)) shift.set(DoubleSolenoid.Value.kReverse); 
    	if (OI.getButton(2, 3) && fliplock) {
    		 fliplock = false;
    		 fliparoo = !fliparoo;
    	} else fliplock = true;
    	if (OI.getButton(1,1) && !topSwitch.get()) {
    		elevateOne.set(.3);
    		elevateTwo.set(.3);
    	} else if (OI.getButton(1,2) && !bottomSwitch.get()) {
    		elevateOne.set(-.3);
    		elevateTwo.set(-.3);
    	} else {
    		elevateOne.set(0); 
    		elevateTwo.set(0);
    	}
    	
    	//System.out.println(Robot.driveBase.getAngle());
    	//double[] defaultValue = new double[0];
    	//System.out.println(vision.getEntry("centerX").getDoubleArray(defaultValue)[0]);
    	//SmartDashboard.putNumberArray("CENTERX", vision.getEntry("centerX").getDoubleArray(defaultValue));
    	SmartDashboard.putNumber("Navx", navx.getAngle());
    	SmartDashboard.putNumber("left", left.getDistance());
    	SmartDashboard.putNumber("right", right.getDistance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
