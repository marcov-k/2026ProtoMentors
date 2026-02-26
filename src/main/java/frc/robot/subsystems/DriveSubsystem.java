package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
    
    // Declare 4 instances of SwerveModules
    private final SwerveModule frontLeft = new SwerveModule(kFrontLeftDrivingCanId, kFrontLeftTurningCanId, kFrontLeftChassisAngularOffset); 
    private final SwerveModule frontRight = new SwerveModule(kFrontRightDrivingCanId, kFrontRightTurningCanId, kFrontRightChassisAngularOffset); 
    private final SwerveModule rearLeft = new SwerveModule(kRearLeftDrivingCanId, kRearLeftTurningCanId, kBackLeftChassisAngularOffset);
    private final SwerveModule rearRight = new SwerveModule(kRearRightDrivingCanId, kRearRightTurningCanId, kBackRightChassisAngularOffset);

    private final SwerveDriveOdometry odometry;

    private final Field2d field = new Field2d();

    // Declare NavX AHRS Gyroscope
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
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
        gyro.zeroYaw();

        // Create an Odometry object
        odometry = new SwerveDriveOdometry(
            kDriveKinematics,
            getHeading(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            }
        );

        SmartDashboard.putData("Field", field);
    }

    // Get current estimated pose from Odometry
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Periodic
    @Override
    public void periodic() {
        odometry.update(
            getHeading(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            }
        );
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Pose X (m)", getPose().getX());
        SmartDashboard.putNumber("Pose Y (m)", getPose().getY());
        SmartDashboard.putNumber("Pose Heading (deg)", getHeading().getDegrees());
        SmartDashboard.putNumber("Raw gyro yaw", gyro.getYaw());        
    }

    // Get Current Angle from Gyroscope (and invert it)
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    // Resets odometry to a specified pose
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getHeading(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            },
            pose);
    }

    // Zero heading
    public void zeroHeading() {
        gyro.zeroYaw();
        resetOdometry(new Pose2d(getPose().getTranslation(),new Rotation2d()));
    }

    // Drive Method
    public void drive(double forward, double strafe, double rotation, boolean fieldRelative) {

        // Convert the commanded speeds into the correct units for the drivetrain, and convert controller left and forward into positive numbers as expected for swerve
        forward = -forward * kMaxSpeedMetersPerSecond;
        strafe = -strafe * kMaxSpeedMetersPerSecond;
        rotation = -rotation * kMaxAngularSpeed;

       
        // Calculate Swerve Module States
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
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
