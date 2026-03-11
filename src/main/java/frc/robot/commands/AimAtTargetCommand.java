package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Launcher;

public class AimAtTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final Launcher launcher;
    private final DoubleSupplier fwd;
    private final DoubleSupplier strafe;
    private final BooleanSupplier fieldRelative;
    private final Supplier<Translation2d> targetSupplier;

    private final PIDController thetaPid = new PIDController(4.0, 0.0, 0.2);

    private int stableLoops = 0;

    public AimAtTargetCommand(
            DriveSubsystem drive,
            Launcher launcher,
            DoubleSupplier fwd,
            DoubleSupplier strafe,
            BooleanSupplier fieldRelative,
            Supplier<Translation2d> targetSupplier) {
        this.drive = drive;
        this.launcher = launcher;
        this.fwd = fwd;
        this.strafe = strafe;
        this.fieldRelative = fieldRelative;
        this.targetSupplier = targetSupplier;

        addRequirements(drive);

        thetaPid.enableContinuousInput(-Math.PI, Math.PI);
        thetaPid.setTolerance(Math.toRadians(1.0));
    }

    @Override
    public void initialize() {
        thetaPid.reset();
    }

    @Override
    public void execute() {
        // Current position and orientation of launcher
        Pose2d LauncherPose = drive.getLauncherPose();

        // Target position
        Translation2d target = targetSupplier.get();

        // Vector from launch point to target
        Translation2d toTarget = target.minus(LauncherPose.getTranslation());

        // Calculate distance 
        double distance = toTarget.getNorm();

        // Lookup Voltage
        double voltage = lookupVoltage(distance);

        // Set voltage 
        launcher.setTargetVoltage(voltage);

        // Desired field-facing angle for the Launcher/robot
        Rotation2d desiredFieldHeading = toTarget.getAngle();

        // Current Launcher heading is same as robot heading here
        Rotation2d currentFieldHeading = LauncherPose.getRotation();

        double rotCmdRadPerSec = thetaPid.calculate(
            currentFieldHeading.getRadians(),
            desiredFieldHeading.getRadians()
        );

        drive.drive(
            fwd.getAsDouble(),
            strafe.getAsDouble(),
            -rotCmdRadPerSec / DriveSubsystem.kMaxAngularSpeed,
            fieldRelative.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(fwd.getAsDouble(), strafe.getAsDouble(), 0.0, fieldRelative.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean atGoal() {
      if (thetaPid.atSetpoint()) {
          stableLoops++;
      } else {
          stableLoops = 0;
      }
      return stableLoops > 5; // ~100 ms at 20ms loop
    }

    // The lookup table for voltages
    private static final double[] Distance = { 6, 8, 18 };
    private static final double[] Voltage = { 5.35, 5.4, 7};

    public static double lookupVoltage(double meters) {
        // Convert meters to feet
        double feet = Units.metersToFeet(meters);

        // If less than 6 feet, return 5.35 volts
        if (feet <= Distance[0]) return Voltage[0];

        // Otherwise interpolate from maped values
        for (int i = 0; i < Distance.length - 1; i++) {
        if (feet <= Distance[i + 1]) {
            double t = (feet - Distance[i]) / (Distance[i + 1] - Distance[i]);
            return Voltage[i] + t * (Voltage[i + 1] - Voltage[i]);
        }
        }
        return Voltage[Voltage.length - 1];
    }
}