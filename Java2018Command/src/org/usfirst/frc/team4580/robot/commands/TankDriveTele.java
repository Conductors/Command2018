package org.usfirst.frc.team4580.robot.commands;

import org.usfirst.frc.team4580.robot.OI;
import org.usfirst.frc.team4580.robot.Robot;
import org.usfirst.frc.team4580.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TankDriveTele extends Command {
    DoubleSolenoid shift;
    DoubleSolenoid test1;
    DigitalInput liftSwitch;
    WPI_TalonSRX intake;
    //NetworkTable vision;	
	public TankDriveTele() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveBase);
        //vision =  NetworkTableInstance.create().getTable("GRIP/vision");
        
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	shift = new DoubleSolenoid(0,1);
    	test1 = new DoubleSolenoid(2,3);
    	Robot.driveBase.PIDEnable(false);
    	liftSwitch = new DigitalInput(0);
    	intake = new WPI_TalonSRX(RobotMap.intake);
    	Scheduler.getInstance().run();    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveBase.tankDrive(OI.getRightSpeed() *1, OI.getLeftSpeed()*1);
    	if (OI.getButton(5)) shift.set(DoubleSolenoid.Value.kForward);
    	if (OI.getButton(6)) shift.set(DoubleSolenoid.Value.kReverse);
    	if (OI.getButton(3)) test1.set(DoubleSolenoid.Value.kForward);
    	if (OI.getButton(4)) test1.set(DoubleSolenoid.Value.kReverse);
    	if (OI.getButton(1)) {
    		intake.set(1);
    	} else if (OI.getButton(2)) {
    		intake.set(-1);
    	} else intake.set(0);
    	
    	//if (!liftSwitch.get())  
    	//System.out.println(Robot.driveBase.getAngle());
    	//double[] defaultValue = new double[0];
    	//System.out.println(vision.getEntry("centerX").getDoubleArray(defaultValue)[0]);
    	//SmartDashboard.putNumberArray("CENTERX", vision.getEntry("centerX").getDoubleArray(defaultValue));
    
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
