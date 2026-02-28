package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Constants.*;

public class AimAtTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier fwd;
    private final DoubleSupplier strafe;
    private final BooleanSupplier fieldRelative;
    private final Supplier<Translation2d> targetSupplier;
    private final PIDController thetaPid = new PIDController(4.0, 0.0, 0.2); // tune

    public AimAtTargetCommand(DriveSubsystem drive,
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
        thetaPid.setTolerance(Math.toRadians(2.0)); // “good enough” aim
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        Translation2d target = targetSupplier.get();
        Translation2d delta = target.minus(pose.getTranslation());
        // double distance = delta.getNorm();
        Rotation2d bearingField = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
        Rotation2d robotToTarget = bearingField.minus(pose.getRotation());
        double rotCmd = thetaPid.calculate(robotToTarget.getRadians(), 0.0);

        drive.drive(
        fwd.getAsDouble(),
        strafe.getAsDouble(),
        rotCmd / DriveConstants.kMaxAngularSpeed, // because your drive() scales rotation by kMaxAngularSpeed
        fieldRelative.getAsBoolean()
        );
    }
}