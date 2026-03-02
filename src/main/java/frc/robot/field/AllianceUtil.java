package frc.robot.field;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtil
{
    public static Translation2d getAllianceHubCenter()
    {
        return isRed() ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;
    }

    public static Translation2d getAllianceLaunchPos()
    {
        return isRed() ? FieldConstants.RED_LAUNCH_POS : FieldConstants.BLUE_LAUNCH_POS;
    }

    public static boolean isRed()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}