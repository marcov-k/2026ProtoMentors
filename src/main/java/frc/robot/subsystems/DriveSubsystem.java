package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
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

import java.util.Set;
import java.util.function.BooleanSupplier;
import com.studica.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {


    /* CONSTANTS */ 
    /* ~~~~~~~~~ */

    // Controller Speed Limits
    public static double kSpeedLimit = 0.75;
    public static final double kControllerDeadband = 0.03;
    
    // Declare 4 instances of SwerveModules
    private final SwerveModule frontLeft = new SwerveModule(kFrontLeftDrivingCanId, kFrontLeftTurningCanId, kFrontLeftChassisAngularOffset); 
    private final SwerveModule frontRight = new SwerveModule(kFrontRightDrivingCanId, kFrontRightTurningCanId, kFrontRightChassisAngularOffset); 
    private final SwerveModule rearLeft = new SwerveModule(kRearLeftDrivingCanId, kRearLeftTurningCanId, kBackLeftChassisAngularOffset);
    private final SwerveModule rearRight = new SwerveModule(kRearRightDrivingCanId, kRearRightTurningCanId, kBackRightChassisAngularOffset);

    // Pose Estimator
    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d gyroFieldOffset = new Rotation2d();

    // Vision 
    public final VisionSubsystem vision = new VisionSubsystem();
    private boolean visionLocked;
    Pose2d lastVisionPose;
    private int visionLockCycles = 0;

    // Fields for the DashBoard
    private final Field2d field = new Field2d();
    private final Field2d visionfield = new Field2d();

    // NavX AHRS Gyroscope
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
    // SPARK MAX CAN IDs - Driving Motors
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 5;    

    // SPARK MAX CAN IDs - Turning Motors
    public static final int kFrontLeftTurningCanId = 4;    
    public static final int kFrontRightTurningCanId = 3;
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
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Location of Launcher 
    public static final Transform2d robotToLauncher = new Transform2d(
        new Translation2d(
            Units.inchesToMeters(-8.48),   // 8.48 inches back from robot center
            Units.inchesToMeters(-7.32)    // 7.32 inches right of robot center
        ),
        new Rotation2d() // 0 degrees rotation means we launch in the same direction as robot forward
    );



    /* CONSTRUCTOR */ 
    /* ~~~~~~~~~~~ */

    // Drive Subsystem Constructor
    public DriveSubsystem() {
        
        // Pose Estimator
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

        // Vision
        visionLocked = false;

        // Smart Dashboard Fields
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("VisionField", visionfield);

        // PathPlanner
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(.30, 0.0, 0.0), // translation PID
                    new PIDConstants(.30, 0.0, 0.0)  // rotation PID
                ),
                config,
                () -> DriverStation.getAlliance()
                      .map(a -> a == DriverStation.Alliance.Red)
                      .orElse(false),
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner AutoBuilder", e.getStackTrace());
        }

    }

    /* COMMAND SCHEDULER */
    /* ~~~~~~~~~~~~~~~~~ */

    // Periodic - Run by Command Scheduler
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

            // Vision must see more than 2 tags for 10 consecutive cycles before we begin rejecting jumps. 
            if (!visionLocked) {
                if (m.tagCount() >= 2 && visionLockCycles >= 10) 
                    visionLocked = true;
                else if (m.tagCount() >= 2)
                    visionLockCycles++;
                else 
                    visionLockCycles = 0;
            }
            // Reject jumps greater then 1 meter or 60 degrees once vision has locked 
            else if (delta > 1.0 || dtheta > Math.toRadians(80)) return;

            // Integrate vision pose into Swerve Pose
            poseEstimator.addVisionMeasurement(visionPose, m.timestampSeconds());
            
        });

        // Update SmartDashboard with pose
        field.setRobotPose(getPose());               
    }

    /* SWERVE */
    /* ~~~~~~ */

    // Get Swerve Module Positions
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        };
    }

    // Drive Method
    public void drive(double forward, double strafe, double rotation, Boolean fieldRelative) {

        // If we're on the Blue Alliance or running in Robot Relative Mode, convert left and forward controller values to positive numbers.
        if (!AllianceUtil.isRed() || !fieldRelative) {
            forward = -forward;
            strafe = -strafe;
        }
        rotation = -rotation;

        // Convert the commanded speeds to correct units
        forward = forward * kMaxSpeedMetersPerSecond;
        strafe = strafe * kMaxSpeedMetersPerSecond;
        rotation = rotation * kMaxAngularSpeed;

       
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


    /* ODOMETRY */
    /* ~~~~~~~~ */

    // Get Current Estimated Pose 
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // Get Current Angle from Gyroscope (and invert it)
    public Rotation2d getRawGyroHeading(){
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    // Get raw Gyro heading and add a Field Offset
    public Rotation2d getHeading() {
        return getRawGyroHeading().plus(gyroFieldOffset);
    }

    // Set Field Heading - Gyroscope offset
    public void setFieldHeading(Rotation2d desiredFieldHeading) {
        gyroFieldOffset = desiredFieldHeading.minus(getRawGyroHeading());
    }

    // Resets odometry to a specified pose
    public void resetOdometry(Pose2d pose) {
        setFieldHeading(pose.getRotation());
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    // Used by Aim At Target Command
    public Pose2d getLauncherPose() {
        return getPose().transformBy(robotToLauncher);
    }

    // Reset Pose based on latest vision pose
    public void setPoseFromVision() {
        if (lastVisionPose == null) return;
        resetOdometry(lastVisionPose);        
    }



    /* PATHPLANNER */
    /* ~~~~~~~~~~~ */

    // PathPlanner - Get Speeds
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState()
        );
    }

    // PathPlanner's Drive Command
    public void driveRobotRelative(ChassisSpeeds speeds) {
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // PathPlanner - Follow Path Command
    public Command followPathCommand(String pathName) {
        return Commands.defer(() -> {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

                Pose2d startPose = path.getStartingHolonomicPose().orElse(null);
                Pose2d currentPose = getPose();

                if (startPose != null &&
                    currentPose.getTranslation().getDistance(startPose.getTranslation()) > 1.0) {

                    DriverStation.reportWarning(
                        "Too far from path start for " + pathName, false
                    );
                    return Commands.none();
                }

                return AutoBuilder.followPath(path);

            } catch (Exception e) {
                DriverStation.reportError(
                    "Unable to load path: " + pathName, e.getStackTrace()
                );
                return Commands.none();
            }
        }, Set.of(this));
    }
}
