package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Launcher;
import frc.robot.utilities.ShooterMap;

public class AutoRPMFromDistanceCommand extends Command {
  private final DriveSubsystem drive;
  private final Launcher launcher;
  private final Supplier<Translation2d> targetSupplier;

  public AutoRPMFromDistanceCommand(DriveSubsystem drive, Launcher launcher, Supplier<Translation2d> targetSupplier) {
    this.drive = drive;
    this.launcher = launcher;
    this.targetSupplier = targetSupplier;

    addRequirements(launcher);
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    Translation2d target = targetSupplier.get();
    double distance = target.minus(pose.getTranslation()).getNorm();

    double rpm = ShooterMap.rpmForDistance(distance);
    launcher.setTargetRpm(rpm);

    SmartDashboard.putNumber("Aim/DistanceMeters", distance);
    SmartDashboard.putNumber("Shooter/DistanceRPM", rpm);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Choose behavior:
    launcher.stopAll();  
    launcher.setHopper(0); // safer default: at least stop feeding
  }
}
  