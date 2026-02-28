package frc.robot.field;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtil {
    public static Translation2d getAllianceHubCenter() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        boolean isRed = alliance.isPresent()
                && alliance.get() == DriverStation.Alliance.Red;

        return isRed
                ? FieldConstants.RED_HUB_CENTER
                : FieldConstants.BLUE_HUB_CENTER;
    }
}