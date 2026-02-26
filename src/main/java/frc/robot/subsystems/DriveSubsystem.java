package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.studica.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
    
    // Declare 4 instances of SwerveModules
    private final SwerveModule frontLeft; 
    private final SwerveModule frontRight; 
    private final SwerveModule rearLeft; 
    private final SwerveModule rearRight; 

    // Declare NavX AHRS Gyroscope
    private final AHRS gyro;
    
    // Speed Limit
    public static double kSpeedLimit = 0.5;

    // SPARK MAX CAN IDs - Driving Motors
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 4;    

    // SPARK MAX CAN IDs - Turning Motors
    public static final int kFrontLeftTurningCanId = 5;    
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 8;  

    // Chassis configuration
    public static final double kWheelBase = Units.inchesToMeters(27.8);
    public static final double kTrackWidth = Units.inchesToMeters(19.25);

    // Driving Parameters 
    public static final double kMaxSpeedMetersPerSecond = 4.8; // Default is 4.8 meters per second     
    public static final double kMaxAngularSpeed = 2 * Math.PI; // Default is 2 PI radians (one full rotation) per second 

    // Swerve Drive Kinematics
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

    // Drive Subsystem Constructor
    public DriveSubsystem() {
        // Initialize 4 instances of SwerveModules
        frontLeft = new SwerveModule(
            kFrontLeftDrivingCanId,
            kFrontLeftTurningCanId,
            kFrontLeftChassisAngularOffset);

        frontRight = new SwerveModule(
            kFrontRightDrivingCanId,
            kFrontRightTurningCanId,
            kFrontRightChassisAngularOffset);

        rearLeft = new SwerveModule(
            kRearLeftDrivingCanId,
            kRearLeftTurningCanId,
            kBackLeftChassisAngularOffset);

        rearRight = new SwerveModule(
            kRearRightDrivingCanId,
            kRearRightTurningCanId,
            kBackRightChassisAngularOffset);

        // Initialize NavX AHRS Gyroscope
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    // Drive Method
    public void drive(double forward, double strafe, double rotation, boolean fieldRelative) {

        // Convert the commanded speeds into the correct units for the drivetrain, and convert controller left and forward into positive numbers as expected for swerve
        forward = -forward * kMaxSpeedMetersPerSecond;
        strafe = -strafe * kMaxSpeedMetersPerSecond;
        rotation = -rotation * kMaxAngularSpeed;

        // Grab the current angle from the Gyroscope and invert it because ChassisSpeeds expects counter clockwise positive. 
        double currentangle = gyro.getYaw() * -1.0;

        // Calculate Swerve Module States
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(currentangle))
                : new ChassisSpeeds(forward, strafe, rotation)
        );

        // Desaturate Swerve Module States 
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeedMetersPerSecond);

        // Set Swerve Module States
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);

    }

    // Drive Command
    public Command driveCommand(CommandXboxController controller, boolean fieldRelative){
        return Commands.run(
            () -> {
                double forward = MathUtil.applyDeadband(controller.getLeftY() * kSpeedLimit, 0.02);
                double strafe = MathUtil.applyDeadband(controller.getLeftX() * kSpeedLimit, 0.02);
                double rotate = MathUtil.applyDeadband(controller.getRightX() * kSpeedLimit, 0.02);
                this.drive(forward, strafe, rotate, fieldRelative);
            }
        , this);
    } 
}
