package org.usfirst.frc.team4580.robot.commands;

import java.lang.management.ClassLoadingMXBean;

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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TankDriveTele extends Command {
    DoubleSolenoid shift;
    DoubleSolenoid intakeOpenClose;
    DoubleSolenoid intakeUpDown;
    DigitalInput topSwitch;
    //DigitalInput bottomSwitch;
    WPI_TalonSRX intakeR;
    WPI_TalonSRX intakeL;
    WPI_TalonSRX elevateOne;
    WPI_TalonSRX elevateTwo;
    WPI_TalonSRX climbOne;
    WPI_TalonSRX climbTwo;
    //Encoder left;
    //Encoder right;
    AHRS navx;
    boolean fliparoo;
    boolean fliplock;
    boolean fliplock2;
    boolean fliparoo2;
    //NetworkTable vision;	
	public TankDriveTele() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveBase);
        intakeR = new WPI_TalonSRX(RobotMap.intakeRight);
        intakeL = new WPI_TalonSRX(RobotMap.intakeLeft);
        shift = Robot.driveBase.getShift();
        //navx = Robot.driveBase.getNavx();
        elevateOne = new WPI_TalonSRX(RobotMap.elevator1);
        elevateTwo = new WPI_TalonSRX(RobotMap.elevator2);
        climbOne = new WPI_TalonSRX(RobotMap.climb1);
        climbTwo = new WPI_TalonSRX(RobotMap.climb2);
        intakeOpenClose = new DoubleSolenoid(3, 6);
        intakeUpDown = new DoubleSolenoid(1,5);
        //left = Robot.driveBase.getLeftRaw();
        //right = Robot.driveBase.getRightRaw();
        fliparoo = false;
        fliplock = true;
        fliparoo2 = false;
        fliplock2 = true;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	
    	Scheduler.getInstance().run();    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveBase.tankDrive(OI.getLeftSpeed() *-1, OI.getRightSpeed()*-1);
    	if (OI.getButton(0,3)) shift.set(DoubleSolenoid.Value.kForward);
    	if (OI.getButton(0,4)) shift.set(DoubleSolenoid.Value.kReverse); 
    	if (OI.getButton(2,4) ) {
    		climbOne.set(.5);
    		climbTwo.set(.5);
    	} else {
    		climbOne.set(0); 
    		climbTwo.set(0);
    	}
    	if (OI.getButton(2,1) ) {
    		intakeL.set(.50);
    		intakeR.set(-.50);
    	} else if (OI.getButton(2,2) ) {
    		intakeL.set(-1.0);
    		intakeR.set(1.0);
    	} else {
    		intakeL.set(0); 
    		intakeR.set(0);
    	}
    	//intakeL.set(OI.getTempAx());
    	//intakeR.set(OI.getTempAx());
    	/* if (OI.getButton(2, 3) && fliplock) {
    		 fliplock = false;
    		 fliparoo = !fliparoo;
    		 if (fliparoo) {
    			 intakeOpenClose.set(DoubleSolenoid.Value.kForward);
    		 } else {
    			 intakeOpenClose.set(DoubleSolenoid.Value.kReverse);
    		 }
    	} */
    	/*if (OI.getButton(2,5) ) {
    		Robot.driveBase.setUpDown(1.0);
    	} else if (OI.getButton(2,6) ) {
    		Robot.driveBase.setUpDown(-1.0);
    	} else {
    		Robot.driveBase.setUpDown(0);
    	} */
    	if (!OI.getButton(2, 3))fliplock = true; 
    	if (OI.getButton(1, 2) && fliplock2) {
   		 fliplock2 = false;
   		 fliparoo2 = !fliparoo2;
   		 if (fliparoo2) {
   			 //intakeUpDown.set(DoubleSolenoid.Value.kForward);
   		 } else {
   			 intakeUpDown.set(DoubleSolenoid.Value.kReverse);
   		 } 
    	}
    	if (!OI.getButton(1, 2))fliplock2 = true; 
    	if (OI.getButton(1,3)) {
    		elevateOne.set(.70);
    		elevateTwo.set(.70);
    	} else if (OI.getButton(1,4)) { //&& topSwitch.get()) {
    		elevateOne.set(-.70);
    		elevateTwo.set(-.70);
    	} else {
    		elevateOne.set(0); 
    		elevateTwo.set(0);
    	}
    	
    	//System.out.println(Robot.driveBase.getAngle());
    	//double[] defaultValue = new double[0];
    	//System.out.println(vision.getEntry("centerX").getDoubleArray(defaultValue)[0]);
    	//SmartDashboard.putNumberArray("CENTERX", vision.getEntry("centerX").getDoubleArray(defaultValue));
    	//SmartDashboard.putNumber("Navx", navx.getAngle());
    	//SmartDashboard.putNumber("left", left.getDistance());
    	//SmartDashboard.putNumber("right", right.getDistance());
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
