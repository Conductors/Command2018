package org.usfirst.frc.team4580.robot.commands;

import edu.wpi.first.wpilibj.PIDOutput;

public class PIDDistTurn implements PIDOutput{
	GoDistance instanc;
	double turnMod;
	public PIDDistTurn () {
		turnMod = 0.0;
	}
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		turnMod = output;
	}
	public double getTurnMod() {
		return turnMod;
	}
}
