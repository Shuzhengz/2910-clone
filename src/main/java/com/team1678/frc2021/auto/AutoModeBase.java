package com.team1678.frc2021.auto;

import com.team1678.frc2021.auto.actions.Action;
import com.team2910.lib.autos.AutonomousTrajectories;
import com.team2910.lib.control.Trajectory;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto modes (which are routines that do actions).
 */
public abstract class AutoModeBase {
    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;

    // protected static TrajectoryGenerator.TrajectorySet trajectories =
    // TrajectoryGenerator.getInstance().getTrajectorySet();
    protected static AutonomousTrajectories trajectories = /*AutonomousTrajectories.getTrajectorySet()*/ null;

    public List<Trajectory> getPaths(){
        return new ArrayList<>();
    }

    protected double startTime = 0.0;
    protected double currentTime(){
        return Timer.getFPGATimestamp() - startTime;
    }

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        m_active = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            System.out.println("Auto mode done, ended early");
            return;
        }

        done();
        System.out.println("Auto mode done");
    }

    public void done() {
    }

    public void stop() {
        m_active = false;
    }

    public boolean isActive() {
        return m_active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        action.start();

        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (m_update_rate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

}