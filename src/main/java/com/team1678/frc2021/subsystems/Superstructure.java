package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Ports;
import com.team1678.frc2021.RobotState;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;

public class Superstructure extends Subsystem {

    private Swerve swerve;
    private Compressor compressor;
    private RobotState robotstate;

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

    private double mHoodSetpoint = 70.5;
    private double mShooterSetpoint = 4000.0;
    private boolean mGotSpunUp = false;
    private boolean mEnableIndexer = true;
    private boolean mManualZoom = false;

    private double mAngleAdd = 0.0;

    public static Superstructure instance = null;

    public Superstructure() {
        swerve = Swerve.getInstance();
        robotstate = RobotState.getInstance();
        compressor = new Compressor(Ports.DRIVEBASE_PCM);
    }

    public static synchronized Superstructure getInstance(){
        if(instance == null)
            instance = new Superstructure();
        return instance;
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

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }


    @Override
    public boolean checkSystem() {
        return false;
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


    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Shooting", mWantsShoot);
        SmartDashboard.putBoolean("Spinning Up", mWantsSpinUp);
        SmartDashboard.putBoolean("Pre Shot", mWantsPreShot);

        SmartDashboard.putBoolean("Test Spit", mWantsTestSpit);
        SmartDashboard.putBoolean("Fendor Shot", mWantsFendor);
        SmartDashboard.putBoolean("Tuck", mWantsTuck);

        SmartDashboard.putNumber("Angle Add", -mAngleAdd);

        SmartDashboard.putNumber("Hood Goal", mHoodSetpoint);
    }

}
