package com.team1678.frc2021.subsystems.superstructure;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Indexer;
import com.team1678.frc2021.subsystems.Intake;

public class FollowSetPoint extends Superstructure{

    public FollowSetPoint() {
        super();
    }

    synchronized void followSetpoint() {

        if (SuperstructureConstants.kUseSmartdashboard) {
            mShooterSetpoint = getShootingSetpointRpm(0);
            mHoodSetpoint = getHoodSetpointAngle(0);
        }

        if (mWantsTuck || !mEnableIndexer) {
            mHood.setSetpointPositionPID(Constants.kHoodConstants.kMinUnitsLimit, 0);
        } else if (mWantsFendor) {
            mHood.setSetpointMotionMagic(39.5);
            mAimSetpoint = 180.0;
        } else if (mWantsTestSpit) {
            mHood.setSetpointMotionMagic(Constants.kHoodConstants.kMinUnitsLimit);
        } else {
            mHood.setSetpointMotionMagic(mHoodSetpoint);
        }

        Indexer.WantedAction indexerAction = Indexer.WantedAction.PREP;
        double real_trigger = 0.0;
        double real_shooter = 0.0;
        boolean real_popout = false;

        if (Intake.getInstance().getState() == Intake.State.INTAKING) {
            mIndexShouldSpin = true;
            indexerAction = Indexer.WantedAction.PREP;
            real_trigger = -600.0;
        }

        if (mWantsSpinUp) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.PREP;
            real_trigger = -600.0;
            enableIndexer(true);
        } else if (mWantsPreShot) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.HELLA_ZOOM;
            real_trigger = Constants.kTriggerRPM;
            real_popout = false;
        } else if (mWantsShoot) {
            real_shooter = mShooterSetpoint;

            if (mLatestAimingParameters.isPresent()) {
                if (mLatestAimingParameters.get().getRange() > 240.) {
                    indexerAction = Indexer.WantedAction.SLOW_ZOOM;
                } else {
                    indexerAction = Indexer.WantedAction.INDEX;
                }
            } else {
                indexerAction = Indexer.WantedAction.INDEX;
            }
            real_trigger = Constants.kTriggerRPM;

            if (mGotSpunUp) {
                real_popout = true;
                enableIndexer(true);
            }

            if (mShooter.spunUp()) {
                mGotSpunUp = true;
            }
        } else if(mWantsTestSpit){
            real_shooter = 1200;
            indexerAction = Indexer.WantedAction.SLOW_ZOOM;
            real_trigger = 4000.0;
            real_popout = true;
            enableIndexer(true);
        }


        if (mWantsUnjam) {
            indexerAction = Indexer.WantedAction.PREP;
            real_popout = true;
            real_trigger = -5000;
        }

        if (mEnableIndexer && mIndexShouldSpin) {
            mIndexer.setState(indexerAction);
        } else {
            mIndexer.setState(Indexer.WantedAction.PREP);
        }

        mTrigger.setPopoutSolenoid(real_popout);
        mTrigger.setVelocity(real_trigger);
        if (Math.abs(real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0);
        } else if (mWantsFendor) {
            mShooter.setVelocity(1500);
        } else if (mWantsTestSpit) {
            mShooter.setVelocity(1200);
            System.out.println("is doing the test spit");
        } else {
            mShooter.setVelocity(real_shooter);
        }

        if (mLatestAimingParameters.isPresent() && !mWantsHoodScan) {
            if (mManualZoom) {
                Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
            } else if (mLatestAimingParameters.get().getRange() > kZoomedInRange
                    && Limelight.getInstance().getPipeline() == Limelight.kDefaultPipeline) {
                Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
            } else if (mLatestAimingParameters.get().getRange() < kZoomedOutRange
                    && Limelight.getInstance().getPipeline() == Limelight.kZoomedInPipeline) {
                Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);
            }
        } else if (mManualZoom) {
            Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
        } else {
            Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);
        }

//        Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);

        if (mTurretMode == TurretControlModes.OPEN_LOOP || !mEnableIndexer) {
            mTurret.setOpenLoop(0);
            // } else if (mTurretMode == TurretControlModes.VISION_AIMED) {
            // mTurret.setSetpointPositionPID(mTurretSetpoint, mTurretFeedforwardV);
        } else {
            mTurret.setSetpointMotionMagic(mTurretSetpoint);
        }
        //mTurret.setOpenLoop(0);
        // mHood.setOpenLoop(0);
        estim_popout = trigger_popout.update(real_popout, 0.2);
    }

}
