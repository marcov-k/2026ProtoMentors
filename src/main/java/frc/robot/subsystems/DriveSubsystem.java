package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.field.AllianceUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import com.studica.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
    
    // Declare 4 instances of SwerveModules
    private final SwerveModule frontLeft = new SwerveModule(kFrontLeftDrivingCanId, kFrontLeftTurningCanId, kFrontLeftChassisAngularOffset); 
    private final SwerveModule frontRight = new SwerveModule(kFrontRightDrivingCanId, kFrontRightTurningCanId, kFrontRightChassisAngularOffset); 
    private final SwerveModule rearLeft = new SwerveModule(kRearLeftDrivingCanId, kRearLeftTurningCanId, kBackLeftChassisAngularOffset);
    private final SwerveModule rearRight = new SwerveModule(kRearRightDrivingCanId, kRearRightTurningCanId, kBackRightChassisAngularOffset);

    

    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d gyroFieldOffset = new Rotation2d();

    private final VisionSubsystem vision = new VisionSubsystem();

    private boolean visionLocked;

    private final Field2d field = new Field2d();
    private final Field2d visionfield = new Field2d();
    Pose2d lastVisionPose;


    // Declare NavX AHRS Gyroscope
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
    // Speed Limit
    public static double kSpeedLimit = 0.5;
    public static final double kControllerDeadband = 0.05;

    // SPARK MAX CAN IDs - Driving Motors
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;    

    // SPARK MAX CAN IDs - Turning Motors
    public static final int kFrontLeftTurningCanId = 6;    
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 7;  

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
        

        // Create a Pose Estimator Object
        poseEstimator = new SwerveDrivePoseEstimator(
            kDriveKinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d(),
            // State std devs (how much you trust swerve+gyro)
            VecBuilder.fill(0.1, 0.1, Math.toRadians(5)),
            // Vision std devs (how much you trust vision)
            VecBuilder.fill(2.5, 2.5, Math.toRadians(30.0))
        );

        visionLocked = false;

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("VisionField", visionfield);
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        };
    }

    // Get current estimated pose from Odometry
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    
    public void teleopInit() {
        visionLocked = false;
    }

    // Periodic
    @Override
    public void periodic() {
        // Update Pose Estimate from Swerve Modules
        poseEstimator.update(getHeading(), getModulePositions());

        // Ask vision for a measurement and gate it
        vision.getMeasurement(poseEstimator.getEstimatedPosition()).ifPresent(m -> {
            // Conservative gates
            if (m.tagCount() <= 0) return;

            // Get Pose from Vision and Swerve
            Pose2d current = poseEstimator.getEstimatedPosition();
            Pose2d visionPose = m.pose();
            lastVisionPose = visionPose;
            visionfield.setRobotPose(visionPose);

            // Compare them
            double delta = current.getTranslation().getDistance(visionPose.getTranslation());
            double dtheta = Math.abs(current.getRotation().minus(visionPose.getRotation()).getRadians());

            // Reject crazy jumps once locked in
            if (delta > 1.0 && visionLocked) return;
            else if (delta > 6.0) return;
            if (dtheta > Math.toRadians(60) && visionLocked) return;

            // Allow one crazy jump based on vision when we can see at least 2 tags, then lock it in.
            if (m.tagCount() >= 2 ) visionLocked = true;

            // Integrate vision pose into Swerve Pose
            poseEstimator.addVisionMeasurement(visionPose, m.timestampSeconds());
            
        });

        // Update SmartDashboard with pose
        field.setRobotPose(getPose());        
        SmartDashboard.putNumber("Pose X (m)", getPose().getX());
        SmartDashboard.putNumber("Pose Y (m)", getPose().getY());
        SmartDashboard.putNumber("Pose Heading (deg)", getHeading().getDegrees());
        SmartDashboard.putNumber("Raw gyro yaw", gyro.getYaw());        
    }

    // Get Current Angle from Gyroscope (and invert it)
    public Rotation2d getRawGyroHeading(){
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    // Get raw Gyro heading and add Field Offset
    public Rotation2d getHeading() {
        return getRawGyroHeading().plus(gyroFieldOffset);
    }

    // Set Field Heading
    public void setFieldHeading(Rotation2d desiredFieldHeading) {
        gyroFieldOffset = desiredFieldHeading.minus(getRawGyroHeading());
    }

    // Resets odometry to a specified pose
    public void resetOdometry(Pose2d pose) {
        setFieldHeading(pose.getRotation());
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    // Zero heading
    public void zeroHeading() {        
        visionLocked = false;
        resetOdometry(new Pose2d(getPose().getTranslation(),new Rotation2d()));
    }

    // Drive Method
    public void drive(double forward, double strafe, double rotation, Boolean fieldRelative) {

        // Convert the commanded speeds into the correct units for the drivetrain, and convert controller left and forward into positive numbers as expected for swerve

        if (!AllianceUtil.isRed()) {
            forward = -forward;
            strafe = -strafe;
        }

        forward = forward * kMaxSpeedMetersPerSecond;
        strafe = strafe * kMaxSpeedMetersPerSecond;
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
    public Command driveCommand(CommandXboxController controller, BooleanSupplier fieldRelative){
        return Commands.run(
            () -> {
                double forward = MathUtil.applyDeadband(controller.getLeftY(), kControllerDeadband) * kSpeedLimit;
                double strafe = MathUtil.applyDeadband(controller.getLeftX(), kControllerDeadband) * kSpeedLimit;
                double rotate = MathUtil.applyDeadband(controller.getRightX(), kControllerDeadband) * kSpeedLimit;
                this.drive(forward, strafe, rotate, fieldRelative.getAsBoolean());
            }
        , this);
    } 

    // Reset Odometry to Starting Pose 
    public Command setPoseFromDsCommand() {
        return Commands.runOnce(() -> resetOdometry(AllianceUtil.getStartingPose()),this);
    }

    public void setPoseFromVision() {
        resetOdometry(lastVisionPose);
        return; 
    }

    public Pose2d getShooterPose() {
        // First-pass estimate; measure this properly later.
        Transform2d robotToShooter = new Transform2d(
            new Translation2d(
                Units.inchesToMeters(-8.48),   // forward from robot center
                Units.inchesToMeters(-7.32)    // right of robot center
            ),
            new Rotation2d() // shooter points same direction as robot
        );

        return getPose().transformBy(robotToShooter);
    }

}
