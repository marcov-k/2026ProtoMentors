package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d( 4.73, 4.25);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.81,4.25);
        
    public static final Rectangle2d BLUE_ALLIANCE_ZONE = new Rectangle2d(new Translation2d(0,0), new Translation2d(5.0, 8.0));
    public static final Rectangle2d NEUTRAL_ZONE = new Rectangle2d(new Translation2d(5.0,0), new Translation2d(11.94, 8.0));
    public static final Rectangle2d RED_ALLIANCE_ZONE = new Rectangle2d(new Translation2d(11.94,0), new Translation2d(16.5, 8.0));

    public final class StartingPositions {
        public static final Pose2d BLUE_STATION_1 = new Pose2d(new Translation2d(4.0,2.3),Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_STATION_2 = new Pose2d(new Translation2d(4.0,4.0),Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_STATION_3 = new Pose2d(new Translation2d(4.0,5.7),Rotation2d.fromDegrees(0));
        public static final Pose2d RED_STATION_1 = new Pose2d(new Translation2d(12.56,2.3),Rotation2d.fromDegrees(180));
        public static final Pose2d RED_STATION_2 = new Pose2d(new Translation2d(12.56,4.0),Rotation2d.fromDegrees(180));
        public static final Pose2d RED_STATION_3 = new Pose2d(new Translation2d(12.56,5.7),Rotation2d.fromDegrees(180));
    }
}