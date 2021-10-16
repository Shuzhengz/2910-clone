package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;

public class WaitToPassYCoordinateAction implements Action{
	double startingYCoordinate;
	double targetYCoordinate;
	Swerve swerve;
	
	public WaitToPassYCoordinateAction(double y){
		targetYCoordinate = y;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public boolean isFinished() {
		return Math.signum(startingYCoordinate - targetYCoordinate) !=
				Math.signum(swerve.getPose().getTranslation().getY() - targetYCoordinate);
	}

	@Override
	public void start() {
		startingYCoordinate = swerve.getPose().getTranslation().getY();
	}

	@Override
	public void update() {
		
	}

	@Override
	public void done() {
		
	}
}
