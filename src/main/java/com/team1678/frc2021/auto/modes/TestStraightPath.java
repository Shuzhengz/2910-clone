package com.team1678.frc2021.auto.modes;

import com.team1678.frc2021.auto.AutoModeBase;
import com.team1678.frc2021.auto.AutoModeEndedException;
import com.team1678.frc2021.auto.actions.ResetPoseAction;
import com.team2910.lib.control.Trajectory;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class TestStraightPath extends AutoModeBase {

    final boolean left;
    final double directionFactor;

    @Override
    public List<Trajectory> getPaths() {
        //return Arrays.asList(trajectories.testStraightTrajectory);
        return null;
    }

	public TestStraightPath(boolean left) {
        
        this.left = left;
        directionFactor = left ? -1.0 : 1.0;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        System.out.println("RUNNING STRAIGHT PATH AUTO!!!");

        runAction(new ResetPoseAction(true));
        // runAction(new SetTrajectoryAction(trajectories.testStraightTrajectory, 0.0, 1.0));
        // runAction(new SetTrajectoryAction(trajectories.straightPath2, 0.0, 1.0));

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
    }
}
