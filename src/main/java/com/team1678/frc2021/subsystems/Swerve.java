package com.team1678.frc2021.subsystems;

import java.util.Optional;

import javax.annotation.concurrent.GuardedBy;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1323.lib.util.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.team1678.frc2021.Constants;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team1678.frc2021.Ports;
import com.team2910.lib.control.HolonomicMotionProfiledTrajectoryFollower;
import com.team2910.lib.control.PidConstants;
import com.team2910.lib.kinematics.ChassisVelocity;
import com.team2910.lib.kinematics.SwerveKinematics;
import com.team2910.lib.kinematics.SwerveOdometry;

import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Rotation2;
import com.team2910.lib.math.Vector2;

import com.team2910.lib.robot.Gyroscope;
import com.team2910.lib.robot.UpdateManager;
import com.team2910.lib.robot.Pigeon.Axis;
import com.team2910.lib.robot.Pigeon;
import com.team2910.lib.util.DrivetrainFeedforwardConstants;
import com.team2910.lib.util.HolonomicDriveSignal;
import com.team2910.lib.util.HolonomicFeedforward;

public class Swerve implements Subsystem, UpdateManager.Updatable {

	private static Swerve instance = null;

	public static Swerve getInstance(){
		if(instance == null)
			instance = new Swerve();
		return instance;
	}

    public static final double TRACKWIDTH = 19.5;
    public static final double WHEELBASE = 19.5;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.86;

    double mPercentOutput = 1.0;
    ChassisVelocity currentVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);

	private final SwerveDriveModule[] modules;

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         // Front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        // Front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),        // Back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        // Back right
    );

    private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(Constants.kDriveTranslationKp, Constants.kDriveTranslationKi, Constants.kDriveTranslationKd);
    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(Constants.kDriveSteeringKp, Constants.kDriveSteeringKi, Constants.kDriveSteeringKd);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)
    );

    private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            FOLLOWER_TRANSLATION_CONSTANTS,
            FOLLOWER_ROTATION_CONSTANTS,
            FOLLOWER_FEEDFORWARD_CONSTANTS
    );
    
    private final Object sensorLock = new Object();
    private static final Object sensorLockStatic = new Object();

    @GuardedBy("sensorLock")
    private Gyroscope gyroscope = new Pigeon(Ports.PIGEON_TALON);

    @GuardedBy("sensorLockStatic")
    private static Gyroscope gyroStatic = new Pigeon(Ports.PIGEON_TALON);

    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;

    private final Object stateLock = new Object();

    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    // Logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;

    private final NetworkTableEntry[] moduleAngleEntries;
    private final NetworkTableEntry[] moduleDriveCurrentEntries;
    private final NetworkTableEntry[] moduleDriveMotorMode;

    Pigeon pigeon = Pigeon.getInstance();

    double distanceTraveled;

    public Swerve() {
        synchronized (sensorLock) {
            gyroscope.setInverted(false);
        }

        TalonFX frontLeftSteeringMotor = new TalonFX(Constants.kFrontLeftRotationId);
        TalonFX backLeftSteeringMotor = new TalonFX(Constants.kBackLeftRotationId);

        TalonFX frontLeftDriveMotor = new TalonFX(Constants.kFrontLeftDriveId);
        TalonFX frontRightDriveMotor = new TalonFX(Constants.kFrontRightDriveId);
        TalonFX backLeftDriveMotor = new TalonFX(Constants.kBackLeftDriveId);
        TalonFX backRightDriveMotor = new TalonFX(Constants.kBackRightDriveId);
		
        frontRightDriveMotor.setInverted(true);
        backRightDriveMotor.setInverted(true);
        frontLeftDriveMotor.setInverted(true);
        backLeftDriveMotor.setInverted(true);

        // Limit speed (testing only)
        configTalon(frontLeftDriveMotor);
        configTalon(frontRightDriveMotor);
        configTalon(backLeftDriveMotor);
        configTalon(backRightDriveMotor);

        SwerveDriveModule frontLeftModule = new SwerveDriveModule(new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                Constants.kFrontLeftEncoderStartingPos,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                frontLeftSteeringMotor,
                frontLeftDriveMotor,
                new CANCoder(Constants.kFrontLeftEncoderId));

        SwerveDriveModule frontRightModule = new SwerveDriveModule(new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                Constants.kFrontRightEncoderStartingPos,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                new TalonFX(Constants.kFrontRightRotationId),
                frontRightDriveMotor,
                new CANCoder(Constants.kFrontRightEncoderId));

        SwerveDriveModule backLeftModule = new SwerveDriveModule(new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                Constants.kBackLeftEncoderStartingPos,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                backLeftSteeringMotor,
                backLeftDriveMotor,
                new CANCoder(Constants.kBackLeftEncoderId));

        SwerveDriveModule backRightModule = new SwerveDriveModule(new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                Constants.kBackRightEncoderStartingPos,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                new TalonFX(Constants.kBackRightRotationId),
                backRightDriveMotor,
                new CANCoder(Constants.kBackRightEncoderId));

        modules = new SwerveDriveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        moduleAngleEntries = new NetworkTableEntry[modules.length];
        moduleDriveCurrentEntries = new NetworkTableEntry[modules.length];
        moduleDriveMotorMode = new NetworkTableEntry[modules.length];

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        ShuffleboardLayout[] moduleLayouts = {
                tab.getLayout("Front Left Module", BuiltInLayouts.kList),
                tab.getLayout("Front Right Module", BuiltInLayouts.kList),
                tab.getLayout("Back Left Module", BuiltInLayouts.kList),
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        };
        for (int i = 0; i < modules.length; i++) {
            ShuffleboardLayout layout = moduleLayouts[i]
                    .withPosition(2 + i * 2, 0)
                    .withSize(2, 4);
            moduleAngleEntries[i] = layout.add("Angle", 0.0).getEntry();
            moduleDriveCurrentEntries[i] = layout.add("Drive percent output", 0.0).getEntry();
            moduleDriveMotorMode[i] = layout.add("Drive motor mode", NeutralMode.Coast.toString()).getEntry();
        }
        tab.addNumber("Rotation Voltage", () -> {
            HolonomicDriveSignal signal;
            synchronized (stateLock) {
                signal = driveSignal;
            }

            if (signal == null) {
                return 0.0;
            }

            return signal.getRotation() * RobotController.getBatteryVoltage();
        });
        setModuleBrakeMode();
    }

    RigidTransform2 startingPose = Constants.kRobotLeftStartingPose;
	public void setStartingPose(RigidTransform2 newPose){
		startingPose = newPose;
    }
    
    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Gyroscope getGyroscope() {
        return gyroscope;
    }

	public synchronized void zeroSensors() {
		zeroSensors(Constants.kRobotLeftStartingPose);
    }
    
    public void set10VoltRotationMode(boolean voltage){
		for (SwerveDriveModule module : modules) {
            module.set10VoltRotationMode(voltage);
        }
    }
    
	/** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
	public synchronized void zeroSensors(RigidTransform2 startingPose){
        pigeon.setAngle(startingPose.getRotation().toDegrees());
        for (SwerveDriveModule module : modules) {
            module.zeroSensors(startingPose);
        }
		pose = startingPose;
		distanceTraveled = 0;
	}

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
    }

    public static Rotation2 getGyroAngle() {
        synchronized (sensorLockStatic) {
            return gyroStatic.getAngle();
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    public void resetWheelAngles() {
        for (SwerveDriveModule module : modules) {
            module.resetAngleOffsetWithAbsoluteEncoder();
        }
    }

    public void setModuleBrakeMode(){
        for(SwerveDriveModule module:modules){
            module.getDriveMotor().setNeutralMode(NeutralMode.Brake);
        }
    }

    public void setModuleCoastMode(){
        for(SwerveDriveModule module:modules){
            module.getDriveMotor().setNeutralMode(NeutralMode.Coast);
        }
    }

    public void disableRotationMotors(){
        for(SwerveDriveModule module:modules){
            module.getRotationMotor().setNeutralMode(NeutralMode.Brake);
        }
    }

    private void configTalon(TalonFX talon) {
        talon.configPeakOutputForward(mPercentOutput, 30);
        talon.configPeakOutputReverse(-mPercentOutput, 30);
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
        }

        synchronized (kinematicsLock) {
            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
        }
    }

    private void updateModules(double time, double dt) {
        updateOdometry(dt);

        Optional<HolonomicDriveSignal> optSignal = follower.update(
            getPose(), 
            currentVelocity.getTranslationalVelocity(),
            currentVelocity.getAngularVelocity(), 
            time, 
            dt);

        HolonomicDriveSignal localSignal;

        if (optSignal.isPresent()) {

            localSignal = optSignal.get();

        } else {
            localSignal = this.driveSignal;
        }

        ChassisVelocity chassisVelocity;
        Boolean drive = true;
        if (localSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (localSignal.getTranslation().equals(Vector2.ZERO, 0.1) && localSignal.getRotation() == 0.0){
            drive = false;
            chassisVelocity = currentVelocity;
        }else if (localSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    localSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    localSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                    localSignal.getTranslation(),
                    localSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i], drive);
            module.updateState(dt);
        }

        currentVelocity = chassisVelocity;    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(dt);

        updateModules(time, dt);
    }

    @Override
    public void periodic() {
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());

        for (int i = 0; i < modules.length; i++) {
            moduleAngleEntries[i].setDouble(Math.toDegrees(modules[i].getCurrentAngle()));
            moduleDriveCurrentEntries[i].setDouble(modules[i].drivePercentOutput.get());
            moduleDriveMotorMode[i].setValue(modules[i].getDriveMotor().getControlMode().toString());
        }
    }

    PeriodicIO periodicIO = new PeriodicIO();
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

	public static class PeriodicIO {
		public double robotX = 0;
		public double robotY = 0;
		public double robotHeading = 0;
		public double robotTheta = 0;
		public double robotVelocity = 0;
		public double pathX = 0;
		public double pathY = 0;
		public double pathHeading = 0;
		public double pathTheta = 0;
		public double pathVelocity = 0;
		public double timestamp = 0.0;
	}

	public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SWERVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public void readPeriodicInputs(){
        for (int i = 0; i < modules.length; i++) {
            periodicIO.robotX = pose.getTranslation().x();
            periodicIO.robotY = pose.getTranslation().y();
            periodicIO.robotHeading = pigeon.getAxis(Axis.YAW);
            periodicIO.robotTheta = pose.getRotation().toDegrees();
            // periodicIO.pathX = motionPlanner.setpoint().state().getTranslation().x();
            // periodicIO.pathY = motionPlanner.setpoint().state().getTranslation().y();
            // periodicIO.pathTheta = motionPlanner.setpoint().state().getRotation().getUnboundedDegrees();
            // periodicIO.pathVelocity = motionPlanner.setpoint().velocity() / Constants.kSwerveMaxSpeedInchesPerSecond;
            periodicIO.timestamp = Timer.getFPGATimestamp();
            if (mCSVWriter != null) {
                mCSVWriter.add(periodicIO);
            }
        }
    }

    public void outputTelemetry(){
        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumberArray("Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().toDegrees()});
            //SmartDashboard.putString("Swerve State", currentState.toString());
            if (mCSVWriter != null) {
                mCSVWriter.write();
            }
            if(Constants.kDebuggingOutput){
                SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
                Logger.log("(" + pose.getTranslation().x() + ", " + pose.getTranslation().y() + "), ");
                SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
                SmartDashboard.putNumber("Robot Heading", pose.getRotation().toDegrees());
                // SmartDashboard.putString("Heading Controller", headingController.getState().toString());
                // SmartDashboard.putNumber("Target Heading", headingController.getTargetHeading());
                SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
                SmartDashboard.putString("Robot Velocity", currentVelocity.toString());
                // SmartDashboard.putString("Swerve State", currentState.toString());
                // SmartDashboard.putBoolean("Vision Updates Allowed", visionUpdatesAllowed);
                SmartDashboard.putNumber("Pigeon Yaw", periodicIO.robotTheta);
                // SmartDashboard.putBoolean("Motion Planner DOne", motionPlanner.isDone());
		}
        }
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

}