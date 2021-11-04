package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1678.frc2021.Constants;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Vector2;
import com.team2910.lib.robot.SwerveModule;
import edu.wpi.first.wpilibj.Notifier;

import java.util.Optional;

public class SwerveDriveModule extends SwerveModule {
    private static final double TALONFX_COUNTS_PER_REVOLUTION = 2048;
    private static final double CAN_UPDATE_RATE = 50.0;
    private static final int TALONFX_PID_LOOP_NUMBER = 0;
    private static final double MAX_STEERING_SPEED = 0.5;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;

    private final double absoluteEncoderAngleOffset;

    private TalonFX steeringMotor;
    private CANCoder angleEncoder;
    private TalonFX driveMotor;

    private boolean driveMotorDefaultInvertState;
    private double driveCountsPerInch;
    private double steerCountsPerDegree;

    private final Object canLock = new Object();
    private double driveDistance = 0.0;
    private double driveVelocity = 0.0;
    private double driveCurrent = 0.0;
    private double currentWheelAngle = 0.0;
    private double moduleAngleToAdd = 0.0;
    public Optional<Double> drivePercentOutput = Optional.empty();
    private Optional<Double> angleToResetSteeringTo = Optional.empty();
    private Optional<Double> steeringTargetPositionDegrees = Optional.empty();

    private Vector2 startingPosition;
    private Vector2 position;
    private RigidTransform2 estimatedRobotPose = new RigidTransform2();
    private double previousEncDistance = 0;

    boolean tenVoltRotationMode = false;

    /**
     * All CAN operations are done in a separate thread to reduce latency on the control thread
     */
    private Notifier canUpdateNotifier = new Notifier(() -> {
        double unadjustedDriveCounts = driveMotor.getSelectedSensorPosition(); // TODO Verify
        synchronized (canLock) {
            this.driveDistance = unadjustedDriveCounts * (1.0 / driveCountsPerInch);
        }

        double driveRpm = driveMotor.getSelectedSensorVelocity(); // TODO Verify
        synchronized (canLock) {
            this.driveVelocity = driveRpm * (1.0 / 60.0) * (1.0 / driveCountsPerInch);
        }

        double driveCurrent = driveMotor.getStatorCurrent();
        synchronized (canLock) {
            this.driveCurrent = driveCurrent;
        }

        Optional<Double> drivePercentOutput;
        synchronized (canLock) {
            drivePercentOutput = this.drivePercentOutput;
        }
        if (drivePercentOutput.isPresent()) {
            this.driveMotor.set(TalonFXControlMode.Position.PercentOutput, drivePercentOutput.get());
        }

        StickyFaults faults = new StickyFaults();
        steeringMotor.getStickyFaults(faults);
        if (faults.UnderVoltage || faults.ResetDuringEn) {
            // We clear faults first for two reasons:
            // 1. To reset the angle offset as close as possible to getting the new angle
            // 2. To allow the faults to be set as soon as possible (eg if we're directly in the
            //    middle of a 5ms brownout, we're more likely to trip this again)
            steeringMotor.clearStickyFaults(Constants.kCANTimeoutMs);
            resetAngleOffsetWithAbsoluteEncoder();
        }

        double unadjustedSteeringAngle = this.steeringMotor.getSelectedSensorPosition() / steerCountsPerDegree;

        Optional<Double> angleToResetSteeringTo;
        double moduleAngleToAdd;
        synchronized (canLock) {
            angleToResetSteeringTo = this.angleToResetSteeringTo;
        }
        if (angleToResetSteeringTo.isPresent()) {
            // We're resetting the current angle offset
            double baseAngle = getClampedAngle(unadjustedSteeringAngle);
            moduleAngleToAdd = angleToResetSteeringTo.get() - baseAngle;
            synchronized (canLock) {
                this.moduleAngleToAdd = moduleAngleToAdd;
                this.angleToResetSteeringTo = Optional.empty();
            }
        } else {
            // Just pull the last angle offset, no need to reset every time
            synchronized (canLock) {
                moduleAngleToAdd = this.moduleAngleToAdd;
            }
        }

        double currentWheelAngle = unadjustedSteeringAngle + (moduleAngleToAdd * this.steerCountsPerDegree);
        synchronized (canLock) {
            this.currentWheelAngle = currentWheelAngle;
        }

        Optional<Double> targetPosition;
        synchronized (canLock) {
            targetPosition = this.steeringTargetPositionDegrees;
            this.steeringTargetPositionDegrees = Optional.empty();
        }
        if (targetPosition.isPresent()) {
            // We first offset the target angle by the amount calculated
            // from the absolute encoder so the calculation is relative
            // to the angle the module thinks it's at
            double targetAngle = targetPosition.get() - moduleAngleToAdd;

            // Then calculate the target angle and drive inversion
            SteeringConfiguration steeringConfig = calculateConfiguration(unadjustedSteeringAngle, targetAngle);

            // And set the values
            double targetPosCalculated = steeringConfig.targetAngle * this.steerCountsPerDegree;
            this.steeringMotor.set(TalonFXControlMode.Position, targetPosCalculated);
            this.driveMotor.setInverted(steeringConfig.invertMotor ^ this.driveMotorDefaultInvertState);
        }
    });

    /**
     * @param modulePosition The module's offset from the center of the robot's center of rotation
     * @param angleOffset    An angle in radians that is used to offset the angle encoder
     * @param angleMotor     The motor that controls the module's angle
     * @param driveMotor     The motor that drives the module's wheel
     * @param angleEncoder   The analog input for the angle encoder
     */
    public SwerveDriveModule(Vector2 modulePosition, double angleOffset, double angleGearRatio, double driveGearRatio,
                           TalonFX angleMotor, TalonFX driveMotor, CANCoder angleEncoder) {
        super(modulePosition);
        this.absoluteEncoderAngleOffset = angleOffset;
        this.steeringMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.driveMotor = driveMotor;
        this.driveCountsPerInch = (TALONFX_COUNTS_PER_REVOLUTION * driveGearRatio) / (Math.PI * WHEEL_DIAMETER_INCHES);
        this.steerCountsPerDegree = (TALONFX_COUNTS_PER_REVOLUTION * angleGearRatio) / 360;
        this.driveMotorDefaultInvertState = driveMotor.getInverted();
        this.resetAngleOffsetWithAbsoluteEncoder();

        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = 60;
        config.enable = true;

        driveMotor.configSupplyCurrentLimit(config);

        // Configure PID values
        driveMotor.config_kP(TALONFX_PID_LOOP_NUMBER, Constants.kDriveTranslationKp, Constants.kCANTimeoutMs);
        driveMotor.config_kI(TALONFX_PID_LOOP_NUMBER, Constants.kDriveTranslationKi, Constants.kCANTimeoutMs);
        driveMotor.config_kD(TALONFX_PID_LOOP_NUMBER, Constants.kDriveTranslationKd, Constants.kCANTimeoutMs);

        // Wipe configuration
        steeringMotor.configFactoryDefault();
        steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TALONFX_PID_LOOP_NUMBER, Constants.kCANTimeoutMs);
        // Make the integrated encoder count forever (don't wrap), since it doesn't work properly with continuous mode
        // We account for this manually (unfortunately)
        steeringMotor.configFeedbackNotContinuous(true, Constants.kCANTimeoutMs);
        // Configure PID values
        steeringMotor.config_kP(TALONFX_PID_LOOP_NUMBER, Constants.kDriveSteeringKp, Constants.kCANTimeoutMs);
        steeringMotor.config_kI(TALONFX_PID_LOOP_NUMBER, Constants.kDriveSteeringKi, Constants.kCANTimeoutMs);
        steeringMotor.config_kD(TALONFX_PID_LOOP_NUMBER, Constants.kDriveSteeringKd, Constants.kCANTimeoutMs);
        // Limit steering module speed
        steeringMotor.configPeakOutputForward(MAX_STEERING_SPEED, Constants.kCANTimeoutMs);
        steeringMotor.configPeakOutputReverse(-MAX_STEERING_SPEED, Constants.kCANTimeoutMs);


        canUpdateNotifier.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    public void resetAngleOffsetWithAbsoluteEncoder() {
        // Absolute position is reported in degrees
        // Not running this on CAN thread since we only poll this when necessary, so
        // losing a few ms of main loop to reduce CAN bas usage is worth it
        double offsetInDegrees = angleEncoder.getAbsolutePosition() - absoluteEncoderAngleOffset;
        if (offsetInDegrees < 0) {
            offsetInDegrees += 360;
        }
        synchronized (canLock) {
            this.angleToResetSteeringTo = Optional.of(offsetInDegrees);
        }
    }

    @Override
    protected double readAngle() {
        double angle;
        synchronized (canLock) {
            angle = this.currentWheelAngle;
        }
        angle = Math.toRadians(angle);
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    @Override
    protected double readDistance() {
        synchronized (canLock) {
            return driveDistance;
        }
    }

    @Override
    public double getCurrentVelocity() {
        synchronized (canLock) {
            return driveVelocity;
        }
    }

    @Override
    public double getDriveCurrent() {
        synchronized (canLock) {
            return this.driveCurrent;
        }
    }

    @Override
    protected void setTargetAngle(double angle) {
        angle = Math.toDegrees(angle);
        synchronized (canLock) {
            this.steeringTargetPositionDegrees = Optional.of(angle);
        }
    }

    @Override
    protected void setDriveOutput(double output) {
        synchronized (canLock) {
            this.drivePercentOutput = Optional.of(output);
        }
    }

    public void set10VoltRotationMode(boolean tenVolts){
		if(tenVolts && !tenVoltRotationMode){
			steeringMotor.selectProfileSlot(1, 0);
			steeringMotor.configVoltageCompSaturation(10.0, 10);
			tenVoltRotationMode = true;
		}else if(!tenVolts && tenVoltRotationMode){
			steeringMotor.selectProfileSlot(0, 0);
			steeringMotor.configVoltageCompSaturation(7.0, 10);
			tenVoltRotationMode = false;
		}
	}

    public TalonFX getDriveMotor(){
        return driveMotor;
    }

    public TalonFX getRotationMotor(){
        return steeringMotor;
    }

	public synchronized void zeroSensors() {
		zeroSensors(new RigidTransform2());
		//rotationPID.reset();
    }
    
    public synchronized void resetPose(RigidTransform2 robotPose){
		Vector2 modulePosition = robotPose.transformBy(RigidTransform2.fromTranslation(startingPosition)).getTranslation();
		position = modulePosition;
	}
	
	public synchronized void zeroSensors(RigidTransform2 robotPose) {
		//driveMotor.setSelectedSensorPosition(0, 0, 100); TODO check if this is necessary
		resetPose(robotPose);
		estimatedRobotPose = robotPose;
		// previousEncDistance = getDriveDistanceInches();
	}

    private static final class SteeringConfiguration {
        public boolean invertMotor;
        public double targetAngle;
    }

    private static double getClampedAngle(double angle) {
        angle %= 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    private static SteeringConfiguration calculateConfiguration(double currentAngle, double targetAngle) {
        SteeringConfiguration ret = new SteeringConfiguration();

        // First, get the current angle in [0, 360)
        double currentAngleClamped = getClampedAngle(currentAngle);

        // Limit the target angle to [0, 360)
        targetAngle %= 360;
        if (targetAngle < 0) {
            targetAngle += 360;
        }

        // Get the difference between the two
        double delta = currentAngleClamped - targetAngle;
        // And move the targetAngle to +/- 180 of the
        // current angle
        if (delta > 180) {
            targetAngle += 360;
        } else if (delta < -180) {
            targetAngle -= 360;
        }

        delta = currentAngleClamped - targetAngle;
        // Now, if necessary, we flip the direction of the wheel
        // This makes the wheel travel no more than 90* to get
        // to the target angle
        if (delta > 90 || delta < -90) {
            if (delta > 90)
                targetAngle += 180;
            else if (delta < -90)
                targetAngle -= 180;
            // And invert the drive motor, since the wheel is facing
            // the opposite direction that it "should"
            ret.invertMotor = true;
        } else {
            ret.invertMotor = false;
        }

        targetAngle += currentAngle - currentAngleClamped;
        ret.targetAngle = targetAngle;
        return ret;
    }
}
