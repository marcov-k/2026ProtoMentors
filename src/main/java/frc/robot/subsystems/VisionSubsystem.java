package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    public record VisionMeasurement(Pose2d pose, double timestampSeconds, int tagCount) {}
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    
    // TODO: Replace with your actual robot-to-camera transform (meters, radians).
    // Coordinate convention: +X forward, +Y left, +Z up (WPILib).
    // Example: new Transform3d(new Translation3d(0.30, 0.00, 0.55), new Rotation3d(0, Units.degreesToRadians(-15), 0))
    private final Transform3d robotToCamera = new Transform3d(new Translation3d(0.30, 0.0, 0.55), new Rotation3d(0, Units.degreesToRadians(0), 0));

    // Optional: keep latest raw vision pose for dashboard
    private Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

    public VisionSubsystem() {
        
        this.camera = new PhotonCamera(VisionConstants.kCameraName);

        AprilTagFieldLayout fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        this.poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                this.robotToCamera
        );

        // Fallback when only one tag is visible
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<VisionMeasurement> getMeasurement(Pose2d referencePose) {
        poseEstimator.setReferencePose(referencePose);
        PhotonPipelineResult result = camera.getLatestResult();
        Optional<EstimatedRobotPose> estimate = poseEstimator.update(result);
        lastEstimatedPose = estimate;

        if (estimate.isEmpty()) {
            SmartDashboard.putBoolean("Vision/HasPose", false);
            return Optional.empty();
        }

        EstimatedRobotPose erp = estimate.get();

        int tagCount = result.hasTargets() ? result.getTargets().size() : 0;
        SmartDashboard.putBoolean("Vision/HasPose", true);
        SmartDashboard.putNumber("Vision/TagCount", tagCount);
        SmartDashboard.putNumber("Vision/Timestamp", erp.timestampSeconds);

        Pose2d pose2d = erp.estimatedPose.toPose2d();
        SmartDashboard.putNumber("Vision/PoseX", pose2d.getX());
        SmartDashboard.putNumber("Vision/PoseY", pose2d.getY());
        SmartDashboard.putNumber("Vision/PoseRotDeg", pose2d.getRotation().getDegrees());

        return Optional.of(new VisionMeasurement(pose2d, erp.timestampSeconds, tagCount));
    }

    public Optional<Pose2d> getLastVisionPose2d() {
        return lastEstimatedPose.map(p -> p.estimatedPose.toPose2d());
    }

    public void init() {
        camera.getLatestResult(); // Warm-up 
    }
}