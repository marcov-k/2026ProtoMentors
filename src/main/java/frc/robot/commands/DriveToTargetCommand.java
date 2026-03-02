package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import frc.robot.field.AllianceUtil;

public class DriveToTargetCommand extends Command
{
    static final double kMinDist = 0.1;
    final DriveSubsystem drive;
    final BooleanSupplier fieldRelative;
    final Supplier<Translation2d> targetSupplier;

    public DriveToTargetCommand(DriveSubsystem drive, BooleanSupplier fieldRelative, Supplier<Translation2d> targetSupplier)
    {
        this.drive = drive;
        this.fieldRelative = fieldRelative;
        this.targetSupplier = targetSupplier;

        addRequirements(drive);
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
            double fwd = (AllianceUtil.isRed()) ? -input.getX() : input.getX();
            double strafe = (AllianceUtil.isRed()) ? input.getY() : -input.getY();

            drive.drive(
                fwd,
                strafe,
                0,
                fieldRelative.getAsBoolean()
            );
        }
    }
}
