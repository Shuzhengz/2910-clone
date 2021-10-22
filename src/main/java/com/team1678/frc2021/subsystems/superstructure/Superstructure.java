package com.team1678.frc2021.subsystems.superstructure;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.RobotState;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team1678.frc2021.subsystems.*;
import com.team1678.lib.util.InterpolatingDouble;
import com.team1678.lib.util.UnitConversion;
import com.team1678.lib.util.Util;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.vision.AimingParameters;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Rotation2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class Superstructure extends Subsystem {

    // Instances

    private static Superstructure mInstance;

    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    private boolean mUseInnerTarget = false;
    private boolean mWantsUnjam = false;
    private boolean mWantsHoodScan = false;
    private boolean mEnforceAutoAimMinDistance = false;
    private boolean mIndexShouldSpin = false;
    private boolean mWantsTuck = false;
    private boolean mWantsFendor = false;
    private boolean mWantsTestSpit = false;
    private boolean mWantsSpinUp = false;
    private boolean mWantsPreShot = false;
    private boolean mWantsShoot = false;

    private double mAngleAdd = 0.0;

    private double mAimSetpoint = 0.0;
    private double mTurnAimSetpoint = 0.0;
    private double mHoodSetpoint = 70.5;         // TODO correct
    private double mShooterSetpoint = 4000.0;    // TODO correct

    private double mCurrentAim = 0.0;
    private double mCurrentHood = 0.0;

    private boolean mGotSpunUp = false;
    private boolean mEnableIndexer = false;
    private boolean mManualZoom = false;
    private boolean mDisableLimelight = false;

    private static final double kZoomedOutRange = 190.0;
    private static final double kZoomedInRange = 220.0;

    /**
     * Shooting things
     */
    private double mHoodFeedforwardV = 0.0;
    private double mCorrectedRangeToTarget = 0.0;
    private double mAutoAimMinDistance = 500;
    private double mAimingThrottle = 0.0;
    protected Optional<AimingParameters> mLatestAimingParameters = Optional.empty();

    private Rotation2 mFieldRelativeAimingGoal = null;

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    enum AimingControlModes {
        FIELD_RELATIVE, VISION_AIMED, OPEN_LOOP, JOGGING
    }

    private AimingControlModes mAimingMode = AimingControlModes.FIELD_RELATIVE;

    protected Superstructure() {
        // The superstructure class
    }

    public synchronized static Superstructure getInstance(){
        if(mInstance == null)
            mInstance = new Superstructure();
        return mInstance;
    }

    private double rotation2ToDegrees(Rotation2 angle) {
        return angle.toDegrees();
    }

    public synchronized void enableIndexer(boolean indexer) {
        mEnableIndexer = indexer;
    }

    public synchronized boolean spunUp() {
        return mGotSpunUp;
    }

    public boolean getWantShoot() {
        return mWantsShoot;
    }

    public void disabledState(){
    }

    public void neutralState(){
    }

    public synchronized boolean isAimed() {
        return mOnTarget;
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }


    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mAimingMode = AimingControlModes.FIELD_RELATIVE;
                    if (SuperstructureConstants.kUseSmartdashboard) {
                        SmartDashboard.putNumber("Shooting RPM", mShooterSetpoint);
                        SmartDashboard.putNumber("Hood Angle", mHoodSetpoint);
                    }
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateCurrentState();
                    maybeUpdateGoalFromVision(timestamp);
                    maybeUpdateGoalFromFieldRelativeGoal(timestamp);
                    maybeUpdateGoalFromHoodScan(timestamp);
                    followSetpoint();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void updateHoodState() {
        mCurrentHood = mHood.getAngle();
    }

    double getShootingSetpointRpm(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (SuperstructureConstants.kUseFlywheelAutoAimPolynomial) {
            return SuperstructureConstants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    double getHoodSetpointAngle(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Hood Angle", 0);
        } else if (SuperstructureConstants.kUseHoodAutoAimPolynomial) {
            return SuperstructureConstants.kHoodAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value + mAngleAdd;
        }
    }

    public synchronized void setWantHoodScan(boolean scan) {
        if (scan != mWantsHoodScan) {
            if (scan) {
                mHoodSetpoint = Constants.kHoodConstants.kMinUnitsLimit + 10;
            } else {
                mHoodSetpoint = mHood.getAngle();
            }
        }
        mWantsHoodScan = scan;
    }

    public synchronized void setGoal(double shooter, double hood, double aim) {
        if ((mAimingMode == AimingControlModes.VISION_AIMED && mHasTarget)) {
            // Keep current setpoints
        } else {
            mAimSetpoint = aim;
            mHoodSetpoint = hood;
            mShooterSetpoint = shooter;
        }
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    public synchronized boolean getDisableLimelight() {
        return mDisableLimelight;
    }

    /**
     * Gets the turn needed by the drivetrain to aim
     * TODO add this to Swerve so the aiming works
     * @return how much to turn
     */
    public synchronized double getTurnNeeded() {
        return mTurnAimSetpoint;
    }

    public synchronized void updateCurrentState() {
        //mCurrentAim = getGyroDegrees();
        mCurrentAim = 0;
        mCurrentHood = mHood.getAngle();
    }

    //private double getGyroDegrees() {
    //    return mSwerve.getYaw().getDegrees();
    //}

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    public synchronized double getAngleAdd() {
        return mAngleAdd;
    }

    public synchronized void setAngleAdd(double add) {
        mAngleAdd -= add;
    }

    public synchronized AimingControlModes getAimingControlMode() {
        return mAimingMode;
    }

    public synchronized boolean getScanningHood() {
        return mWantsHoodScan;
    }

    public void safetyReset() {
        Limelight.getInstance().setLed(Limelight.LedMode.OFF);
        mDisableLimelight = true;

        if (mHoodSetpoint < Constants.kHoodConstants.kMinUnitsLimit) {
            // logic for when hood fully in]
            System.out.println("running safety reset?");
            mHoodSetpoint = Constants.kHoodConstants.kMinUnitsLimit;
        }
        if (mHoodSetpoint > Constants.kHoodConstants.kMaxUnitsLimit) {
            mHoodSetpoint = Constants.kHoodConstants.kMaxUnitsLimit;
            // logic for when hood fully extended
        }
    }

    public synchronized void maybeUpdateGoalFromHoodScan(double timestamp) {
        if (!mWantsHoodScan) {
            return;
        }

        if (Util.epsilonEquals(mHood.getAngle(), Constants.kHoodConstants.kMinUnitsLimit + 10, 10.0)) {
            mHoodSetpoint = Constants.kHoodConstants.kMaxUnitsLimit - 10;
        } else if (Util.epsilonEquals(mHood.getAngle(), Constants.kHoodConstants.kMaxUnitsLimit - 10, 10.0)) {
            mHoodSetpoint = Constants.kHoodConstants.kMinUnitsLimit + 10;
        }
    }

    public synchronized void maybeUpdateGoalFromVision(double timestamp) {

        if (mAimingMode != AimingControlModes.VISION_AIMED) {
            resetAimingParameters();
            return;
        }

        if (mWantsShoot && mGotSpunUp) {
            mLatestAimingParameters = mRobotState.getAimingParameters(mUseInnerTarget, 0, Constants.kMaxGoalTrackAge);
            //System.out.println("ye we settin it to trackid");
        } else {
            mLatestAimingParameters = mRobotState.getAimingParameters(mUseInnerTarget, 1, Constants.kMaxGoalTrackAge);
            //System.out.println("ye we got it to a track -1");
        }

        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            RigidTransform2 robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(Constants.kAutoAimPredictionTime));
            RigidTransform2 predicted_aim_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getTurretToGoal());
            mCorrectedRangeToTarget = predicted_aim_to_goal.getTranslation().norm();

            System.out.println("has current aiming parameters");

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                System.out.println("Not meeting aiming recs");
                return;
            }

            final double shooting_setpoint = getShootingSetpointRpm(mCorrectedRangeToTarget);
            mShooterSetpoint = shooting_setpoint;

            final double aiming_setpoint = getHoodSetpointAngle(mCorrectedRangeToTarget);

            if (!mWantsHoodScan) {
                mHoodSetpoint = aiming_setpoint;
            }

            final Rotation2 aiming_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getTurretToGoalRotation());

            mAimSetpoint = mCurrentAim + /* - */ aiming_error.toDegrees(); // might switch to subtraction of error
            final Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            final double tangential_component = mLatestAimingParameters.get().getTurretToGoalRotation().sin
                    * velocity.dx / mLatestAimingParameters.get().getRange();
            final double angular_component = UnitConversion.radians_to_degrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local
            // frame.
            //mAimFeedforwardV = -(angular_component + tangential_component);

            safetyReset();

            mHasTarget = true;
            final double hood_error = mCurrentHood - mHoodSetpoint;

            if (Util.epsilonEquals(aiming_error.toDegrees(), 0.0, 3.0) && Util.epsilonEquals(hood_error, 0.0, 3.0)) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }

        } else {

            //System.out.println("No aiming paramenters :O");
            mHasTarget = false;
            mOnTarget = false;
        }
    }

    public synchronized void maybeUpdateGoalFromFieldRelativeGoal(double timestamp) {
        if (mAimingMode != AimingControlModes.FIELD_RELATIVE && mAimingMode != AimingControlModes.VISION_AIMED) {
            mFieldRelativeAimingGoal = null;
            return;
        }
        if (mAimingMode == AimingControlModes.VISION_AIMED && !mLatestAimingParameters.isEmpty()) {
            // Vision will control the aim.
            return;
        }
        if (mFieldRelativeAimingGoal == null) {
            return;
        }
        final double kLookaheadTime = 4.0;
        Rotation2 aim_error = mRobotState.getPredictedFieldToVehicle(timestamp + kLookaheadTime) // getPredictedFieldToVehicle
                .transformBy(mRobotState.getVehicleToTurret(timestamp)).getRotation().inverse()
                .rotateBy(mFieldRelativeAimingGoal);
        // System.out.println("aim Error" + aim_error);
        mAimSetpoint =  /* - */ aim_error.toDegrees();
        // System.out.println("aim error " + aim_error.toDegrees());
        safetyReset();
    }

    // Aim like a God
    public synchronized void setAimOpenLoop(double throttle) {
        mAimingMode = AimingControlModes.OPEN_LOOP;
        mAimingThrottle = throttle;
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

        Indexer.spinMotor();
        double real_shooter = 0.0;
        boolean real_popout = false;

        if (Intake.getInstance().getState() == Intake.State.INTAKING) {
            mIndexShouldSpin = true;
            indexerAction = Indexer.WantedAction.PREP;
        }

        if (mWantsSpinUp) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.PREP;
            enableIndexer(true);
        } else if (mWantsPreShot) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.HELLA_ZOOM;
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
            real_popout = true;
            enableIndexer(true);
        }


        if (mWantsUnjam) {
            indexerAction = Indexer.WantedAction.PREP;
            real_popout = true;
        }

        if (mEnableIndexer && mIndexShouldSpin) {
            mIndexer.setState(indexerAction);
        } else {
            mIndexer.setState(Indexer.WantedAction.PREP);
        }


        if (Math.abs(real_shooter) < Util.K_EPSILON) {
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

        if (mAimingMode == AimingControlModes.OPEN_LOOP || !mEnableIndexer) {
            // Noting happnes, hull aimed
        } else {
            mTurnAimSetpoint = mAimSetpoint;
        }
        // mHood.setOpenLoop(0);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Shooting", mWantsShoot);
        SmartDashboard.putBoolean("Spinning Up", mWantsSpinUp);
        SmartDashboard.putBoolean("Pre Shot", mWantsPreShot);
        SmartDashboard.putNumber("Limelight Range", mCorrectedRangeToTarget);

        SmartDashboard.putBoolean("Test Spit", mWantsTestSpit);
        SmartDashboard.putBoolean("Fendor Shot", mWantsFendor);
        SmartDashboard.putBoolean("Tuck", mWantsTuck);

        SmartDashboard.putNumber("Angle Add", -mAngleAdd);

        SmartDashboard.putNumber("Hood Goal", mHoodSetpoint);
    }

}
