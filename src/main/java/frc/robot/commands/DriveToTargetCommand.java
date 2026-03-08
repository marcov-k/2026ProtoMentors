package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

public class DriveToTargetCommand extends Command
{
    static final double kMaxSpeed = 0.1;
    static final double kMinDist = 0.1;
    final DriveSubsystem drive;
    final Supplier<Translation2d> targetSupplier;
    final PIDController fwdPID = new PIDController(0.45, 0.0, 0.02); // tune
    final PIDController strafePID = new PIDController(0.45, 0.0, 0.02); // tune

    public DriveToTargetCommand(DriveSubsystem drive, Supplier<Translation2d> targetSupplier)
    {
        this.drive = drive;
        this.targetSupplier = targetSupplier;

        addRequirements(drive);

        fwdPID.setTolerance(0.1);
        strafePID.setTolerance(0.1);
    }

    @Override
    public void execute()
    {
        // TODO: add support for driving to target when not in field relative?

        Pose2d pose = drive.getPose();
        Translation2d target = targetSupplier.get();
        Translation2d delta = target.minus(pose.getTranslation());
        double fwd = -MathUtil.clamp(fwdPID.calculate(delta.getX()), -kMaxSpeed, kMaxSpeed);            
        double strafe = -MathUtil.clamp(strafePID.calculate(delta.getY()), -kMaxSpeed, kMaxSpeed);

        drive.drive(fwd, strafe, 0, true);

    }


    @Override
    public void end(boolean interrupted)
    {
        drive.drive(0,0,0,true);
    }

    @Override
    public boolean isFinished()
    {
        Pose2d pose = drive.getPose();
        Translation2d target = targetSupplier.get();
        return target.minus(pose.getTranslation()).getNorm() <= kMinDist;
    }
}