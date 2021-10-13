package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.Ports;
import com.team1678.frc2021.RobotState;
import com.team1678.frc2021.SuperstructureConstants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team1678.lib.util.InterpolatingDouble;
import com.team1678.lib.util.Util;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;

public class Superstructure extends Subsystem {

    // Instances

    private static Superstructure mInstance;

    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();

    private boolean mWantsShoot = false;
    private boolean mWantsSpinUp = false;
    private boolean mWantsTuck = false;
    private boolean mWantsFendor = false;
    private boolean mWantsTestSpit = false;
    private boolean mUseInnerTarget = false;
    private boolean mWantsPreShot = false;
    private boolean mWantsUnjam = false;
    private boolean mWantsHoodScan = false;

    private double mCurrentHood = 0.0;

    private double mAngleAdd = 0.0;
    private double mHoodSetpoint = 70.5;         // TODO correct
    private double mShooterSetpoint = 4000.0;    // TODO correct
    private double mCorrectedRangeToTarget = 0.0;

    private boolean mGotSpunUp = false;
    private boolean mEnableIndexer = false;
    private boolean mManualZoom = false;

    private Rotation2d mFieldRelativeTurretGoal = null;

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private Superstructure() {
        // The superstructure class
    }

    public static synchronized Superstructure getInstance(){
        if(mInstance == null)
            mInstance = new Superstructure();
        return mInstance;
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            stop();
        }

        @Override
        public void onLoop(double timestamp) {

        }

        @Override
        public void onStop(double timestamp) {
            disabledState();
        }
    };

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
    public void registerEnabledLoops(ILooper enabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mTurretMode = TurretControlModes.FIELD_RELATIVE;
                    if (SuperstructureConstants.kUseSmartdashboard) {
                        SmartDashboard.putNumber("Shooting RPM", mShooterSetpoint);
                        SmartDashboard.putNumber("Hood Angle", mHoodSetpoint);
                    }
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateHoodState();
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

    private double getShootingSetpointRpm(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (SuperstructureConstants.kUseFlywheelAutoAimPolynomial) {
            return SuperstructureConstants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    private double getHoodSetpointAngle(double range) {
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

    public synchronized void setGoal(double shooter, double hood, double turret) {
        if ((mTurretMode == TurretControlModes.VISION_AIMED && mHasTarget)) {
            // Keep current setpoints
        } else {
            mTurretSetpoint = turret;
            mHoodSetpoint = hood;
            mShooterSetpoint = shooter;
        }
    }

    ublic synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardV = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    public synchronized boolean getDisableLimelight() {
        return mDisableLimelight;
    }

    public synchronized void updateCurrentState() {
        mCurrentTurret = mTurret.getAngle();
        mCurrentHood = mHood.getAngle();
    }

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    public synchronized double getAngleAdd() {
        return mAngleAdd;
    }

    public synchronized void setAngleAdd(double add) {
        mAngleAdd -= add;
    }

    public synchronized TurretControlModes getTurretControlMode() {
        return mTurretMode;
    }

    public synchronized boolean getScanningHood() {
        return mWantsHoodScan;
    }

    public void safetyReset() {
        if (mTurretSetpoint < Constants.kTurretConstants.kMinUnitsLimit) {
            mTurretSetpoint += SuperstructureConstants.kTurretDOF;
            Limelight.getInstance().setLed(Limelight.LedMode.OFF);
            mDisableLimelight = true;
        } else if (mTurretSetpoint > Constants.kTurretConstants.kMaxUnitsLimit) {
            mTurretSetpoint -= SuperstructureConstants.kTurretDOF;
            Limelight.getInstance().setLed(Limelight.LedMode.OFF);
            mDisableLimelight = true;
        } else {
            mDisableLimelight = false;
            Limelight.getInstance().setLed(Limelight.LedMode.ON);
        }

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
