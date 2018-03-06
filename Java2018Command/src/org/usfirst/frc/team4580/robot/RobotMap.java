/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4580.robot;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	public static int leftFrontTal = 6;
	public static int leftBackTal = 1;
	public static int rightFrontTal = 2;
	public static int rightBackTal = 5;
	/*
	public static int leftFrontTal = 2;
	public static int leftBackTal = 1;
	public static int rightFrontTal = 4;
	public static int rightBackTal = 3;
	*/
	public static int intakeRight = 6;
	public static int intakeLeft = 5; 
	public static int elevator1 = 15;
	public static int elevator2 = 16;
	public static int climb1 = 13;
	public static int climb2 = 14;
	public static int stick1 = 0;
	public static int stick2 = 1;
	public static int leftStick = 1;
	public static int rightStick = 2;
	public static double rotateVelocity = .5;
	public static double botLength = 32;
	public static double wheelSize = 6.0;
	public static double encoderPPR = 270;
	public static double encoderDPP = wheelSize*Math.PI/encoderPPR;
}
