package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;

public class WaitForHeadingAction implements Action{
	Swerve swerve;
	double lowThreshold;
	double highThreshold;
	
	public WaitForHeadingAction(double lowThreshold, double highThreshold){
		swerve = Swerve.getInstance();
		this.lowThreshold = lowThreshold;
		this.highThreshold = highThreshold;
	}
	
	@Override
	public boolean isFinished(){
		double heading = swerve.getPose().getRotation().getDegrees();
		return heading >= lowThreshold && heading <= highThreshold;
	}
	
	@Override
	public void start(){
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){
	}
}
