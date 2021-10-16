package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.math.RigidTransform2;

public class ResetPoseAction extends RunOnceAction{
	private RigidTransform2 newPose;
	boolean leftStartingSide = true;

	Swerve swerve;
	
	public ResetPoseAction(RigidTransform2 newPose){
		this.newPose = newPose;
		swerve = Swerve.getInstance();
	}

	public ResetPoseAction(boolean slalom){
		newPose = slalom ? Constants.kSlalomStartingPose : Constants.kAutoNavStartingPose;
		swerve = Swerve.getInstance();
	}

	@Override
	public void runOnce() {
		//swerve.setStartingPose(newPose);
		//swerve.zeroSensors(newPose); // TODO: Fix zeroSensors method
	}

}
