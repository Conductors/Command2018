/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4580.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4580.robot.commands.GoDistance;
import org.usfirst.frc.team4580.robot.commands.LeftLeftScale;
import org.usfirst.frc.team4580.robot.commands.LeftLine;
import org.usfirst.frc.team4580.robot.commands.LeftRightScale;
import org.usfirst.frc.team4580.robot.commands.MiddleLeftScale;
import org.usfirst.frc.team4580.robot.commands.MiddleLine;
import org.usfirst.frc.team4580.robot.commands.MiddleRightScale;
import org.usfirst.frc.team4580.robot.commands.RightLeftScale;
import org.usfirst.frc.team4580.robot.commands.RightLine;
import org.usfirst.frc.team4580.robot.commands.RightRightScale;
import org.usfirst.frc.team4580.robot.commands.TeleCommands;
import org.usfirst.frc.team4580.robot.subsystems.DriveBase;
import org.usfirst.frc.team4580.robot.subsystems.Lift;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI m_oi;
	Command m_autonomousCommand;
	SendableChooser<String> m_chooser = new SendableChooser<>();
	SendableChooser<String> typeChoose = new SendableChooser<>();
	public static DriveBase driveBase;
	public static TeleCommands teleCommands;
	public static LeftLeftScale leftLeftScale;
	public static LeftRightScale leftRightScale;
	public static LeftLine leftLine;
	public static RightLeftScale rightLeftScale;
	public static RightRightScale rightRightScale;
	public static RightLine rightLine;
	public static MiddleLeftScale middleLeftScale;
	public static MiddleRightScale middleRightScale;
	public static MiddleLine middleLine;
	public static Lift lift;
	//public static GoDistance gogo;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		driveBase = new DriveBase();
		lift = new Lift();
		teleCommands = new TeleCommands();
		leftLeftScale = new LeftLeftScale();
		leftRightScale = new LeftRightScale();
		leftLine = new LeftLine();
		rightLeftScale = new RightLeftScale();
		rightRightScale = new RightRightScale();
		rightLine = new RightLine();
		middleLeftScale = new MiddleLeftScale();
		middleRightScale = new MiddleRightScale();
		middleLine = new MiddleLine();
		m_chooser.addDefault("Left",  "left");
		m_chooser.addObject("Middle", "middle");
		m_chooser.addObject("Right)", "right");
		typeChoose.addDefault("Scale", "scale");
		typeChoose.addObject("Baseline","line");
		SmartDashboard.putData("Side", m_chooser);
		SmartDashboard.putData("Type", typeChoose);
		try {
			CameraServer.getInstance().startAutomaticCapture();
		} catch (Exception e) {
			DriverStation.reportError("CAMERA SERVER ERROR: " + e.getMessage(), true);
		}
		Compressor compressor = new Compressor(0);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		/* m_autonomousCommand = m_chooser.getSelected();

		*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 *
		*/
		// schedule the autonomous command (example)

		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (m_chooser.getSelected().equals("left")) {
			if (typeChoose.getSelected().equals("scale")) {
				if (gameData.charAt(1) == 'L') {
					m_autonomousCommand = leftLeftScale;
				} else {
					m_autonomousCommand = leftRightScale;
				} 
			} else if (typeChoose.getSelected().equals("line")) {
				m_autonomousCommand = leftLine;
			}
		} else if (m_chooser.getSelected().equals("middle")) {
			if (typeChoose.getSelected().equals("scale")) {
				if (gameData.charAt(1) == 'L') {
					m_autonomousCommand = middleLeftScale;
				} else {
					m_autonomousCommand = middleRightScale;
				} 
			} else if (typeChoose.getSelected().equals("line")) {
				m_autonomousCommand = middleLine;
			}
		} else if (m_chooser.getSelected().equals("right")) {
			if (typeChoose.getSelected().equals("scale")) {
				if (gameData.charAt(1) == 'L') {
					m_autonomousCommand = rightLeftScale;
				} else {
					m_autonomousCommand = rightRightScale;
				} 
			} else if (typeChoose.getSelected().equals("line")) {
				m_autonomousCommand = rightLine;
			}
		}
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		//baseline.PIDEnable(false);
		if (m_autonomousCommand != null) {
			//m_autonomousCommand.cancel();
		}
		
		teleCommands.start();
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
