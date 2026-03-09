package frc.robot.field;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
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

    // Returns an estimated default Starting Position based on Alliance and Station number
    public static Pose2d getStartingPose() {
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
        int station = DriverStation.getLocation().orElse(2);
        boolean isRed = allianceOpt.isPresent() && allianceOpt.get() == DriverStation.Alliance.Red;
        Pose2d startingPose = new Pose2d();        

        if (!isRed) {
            // BLUE side
            switch (station) {
                case 1: startingPose = FieldConstants.StartingPositions.BLUE_STATION_1; break;
                case 2: startingPose = FieldConstants.StartingPositions.BLUE_STATION_2; break;
                case 3: startingPose = FieldConstants.StartingPositions.BLUE_STATION_3; break;
                default: startingPose = FieldConstants.StartingPositions.BLUE_STATION_1;
            }

        } else {
            // RED side
            switch (station) {
                case 1: startingPose = FieldConstants.StartingPositions.RED_STATION_1; break;
                case 2: startingPose = FieldConstants.StartingPositions.RED_STATION_2; break;
                case 3: startingPose = FieldConstants.StartingPositions.RED_STATION_3; break;
                default: startingPose = FieldConstants.StartingPositions.RED_STATION_1;
            }
        }
        
        return startingPose;
    }


    // Returns an estimated default Starting Position based on Alliance and Station number
    public static Translation2d getAutonomousFiringPosition() {
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
        int station = DriverStation.getLocation().orElse(2);
        boolean isRed = allianceOpt.isPresent() && allianceOpt.get() == DriverStation.Alliance.Red;
        Translation2d firingPose = new Translation2d();        

        if (!isRed) {
            // BLUE side
            switch (station) {
                case 1: firingPose = FieldConstants.AutoFiringPositions.BLUE_STATION_1; break;
                case 2: firingPose = FieldConstants.AutoFiringPositions.BLUE_STATION_2; break;
                case 3: firingPose = FieldConstants.AutoFiringPositions.BLUE_STATION_3; break;
                default: firingPose = FieldConstants.AutoFiringPositions.BLUE_STATION_1;
            }

        } else {
            // RED side
            switch (station) {
                case 1: firingPose = FieldConstants.AutoFiringPositions.RED_STATION_1; break;
                case 2: firingPose = FieldConstants.AutoFiringPositions.RED_STATION_2; break;
                case 3: firingPose = FieldConstants.AutoFiringPositions.RED_STATION_3; break;
                default: firingPose = FieldConstants.AutoFiringPositions.RED_STATION_1;
            }
        }
        
        return firingPose;
    }

    public static boolean isRed() {
        return DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false);
    }

}

