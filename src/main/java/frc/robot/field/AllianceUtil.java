package frc.robot.field;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtil {
    
    public final class FieldConstants {
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d( 4.574159, 4.0213534);
        public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9388636,4.0213534);
            
        public static final Rectangle2d BLUE_ALLIANCE_ZONE = new Rectangle2d(new Translation2d(0,0), new Translation2d(5.0, 8.0));
        public static final Rectangle2d NEUTRAL_ZONE = new Rectangle2d(new Translation2d(5.0,0), new Translation2d(11.94, 8.0));
        public static final Rectangle2d RED_ALLIANCE_ZONE = new Rectangle2d(new Translation2d(11.94,0), new Translation2d(16.5, 8.0));
    }

    public static Translation2d getAllianceHubCenter() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        boolean isRed = alliance.isPresent()
                && alliance.get() == DriverStation.Alliance.Red;

        return isRed
                ? FieldConstants.RED_HUB_CENTER
                : FieldConstants.BLUE_HUB_CENTER;
    }

    public static boolean isRed() {
        return DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false);
    }

}

