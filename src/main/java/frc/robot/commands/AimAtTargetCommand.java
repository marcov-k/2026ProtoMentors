package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AimAtTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier fwd;
    private final DoubleSupplier strafe;
    private final BooleanSupplier fieldRelative;
    private final Supplier<Translation2d> targetSupplier;

    private final PIDController thetaPid = new PIDController(4.0, 0.0, 0.2);

    private int stableLoops = 0;

    public AimAtTargetCommand(
            DriveSubsystem drive,
            DoubleSupplier fwd,
            DoubleSupplier strafe,
            BooleanSupplier fieldRelative,
            Supplier<Translation2d> targetSupplier) {
        this.drive = drive;
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
        Pose2d shooterPose = drive.getShooterPose();
        Translation2d target = targetSupplier.get();

        // Vector from actual launch point to target
        Translation2d toTarget = target.minus(shooterPose.getTranslation());

        // Desired field-facing angle for the shooter/robot
        Rotation2d desiredFieldHeading = toTarget.getAngle();

        // Current shooter heading is same as robot heading here
        Rotation2d currentFieldHeading = shooterPose.getRotation();

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
}