package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants
{
    public static class VisionConstants
    {
        public static final String kCameraName = "FrontLeftCamera";
    }

    @SuppressWarnings("removal")
    public static final class ModuleConstants
    {
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final double kDrivingMotorFreeSpeedRps = 94.6;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        static
        {
            double drivingFactor = kWheelCircumferenceMeters / kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / kDriveWheelFreeSpeedRps;

            drivingConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50);
            drivingConfig.encoder.positionConversionFactor(drivingFactor).velocityConversionFactor(drivingFactor / 60.0);
            drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04,0,0).velocityFF(drivingVelocityFeedForward).outputRange(-1,1);

            turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
            turningConfig.absoluteEncoder.inverted(true).positionConversionFactor(turningFactor).velocityConversionFactor(turningFactor / 60.);
            turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1,0,0).outputRange(-1, 1).positionWrappingEnabled(true).positionWrappingInputRange(0, turningFactor);
        }
    }

    public static class LauncherConstants
    {
        public static final int kHopperMotorCanID = 10;
        public static final int kLaunchMotorCanID = 11;

        public static SparkMaxConfig DefaultConfig = new SparkMaxConfig();
        static
        {
            DefaultConfig.smartCurrentLimit(50);
            DefaultConfig.idleMode(IdleMode.kCoast);
            DefaultConfig.openLoopRampRate(1.0);
            DefaultConfig.inverted(true);
            DefaultConfig.voltageCompensation(12);
            DefaultConfig.closedLoop.allowedClosedLoopError(100.0, ClosedLoopSlot.kSlot0);
        }
    }

    public static class IntakeConstants
    {
        public static final int kIntakeMotorCanID = 9;

        public static SparkMaxConfig DefaultConfig = new SparkMaxConfig();
        static
        {
            DefaultConfig.smartCurrentLimit(50);
            DefaultConfig.idleMode(IdleMode.kCoast);
            DefaultConfig.openLoopRampRate(1.0);
            DefaultConfig.inverted(false);
        }
    }

    public static class DriveConstants
    {
        public static final double kSpeedLimit = 0.5;

        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 4;

        public static final int kFrontLeftTurningCanId = 5;
        public static final int kFrontRightTurningCanId = 6;
        public static final int kRearRightTurningCanId = 7;
        public static final int kRearLeftTurningCanId = 8;

        public static final double kWheelBase = Units.inchesToMeters(27.8);
        public static final double kTrackWidth = Units.inchesToMeters(19.25);

        public static final double kMaxSpeedMetersPerSecond = 4.8; // Default is 4.8 meters per second     
        public static final double kMaxAngularSpeed = 2 * Math.PI; // Default is 2 PI radians (one full rotation) per second 

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    }
}