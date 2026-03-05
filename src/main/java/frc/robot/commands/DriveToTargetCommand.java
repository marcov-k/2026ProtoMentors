package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import frc.robot.field.AllianceUtil;

public class DriveToTargetCommand extends Command
{
    static final double kMinDist = 0.1;
    final DriveSubsystem drive;
    final Supplier<Translation2d> targetSupplier;
    final PIDController posPID = new PIDController(4.0, 0.0, 0.2); // tune

    public DriveToTargetCommand(DriveSubsystem drive, Supplier<Translation2d> targetSupplier)
    {
        this.drive = drive;
        this.targetSupplier = targetSupplier;

        addRequirements(drive);

        posPID.setTolerance(0.1); // "close enough" position
    }

    @Override
    public void execute()
    {
        // TODO: add support for driving to target when not in field relative

        Pose2d pose = drive.getPose();
        Translation2d target = targetSupplier.get();
        Translation2d delta = target.minus(pose.getTranslation());

        double dist = delta.getNorm();
        if (dist > kMinDist)
        {
            Translation2d input = delta.div(dist); // turn delta into a unit vector

            double fwd = posPID.calculate(input.getX());
            fwd = (AllianceUtil.isRed()) ? -fwd : fwd;

            double strafe = posPID.calculate(input.getY());
            strafe = (AllianceUtil.isRed()) ? strafe : -strafe;

            drive.drive(
                fwd,
                strafe,
                0,
                true
            );
        }
    }
}