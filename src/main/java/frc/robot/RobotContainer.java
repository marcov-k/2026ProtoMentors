// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.field.AllianceUtil;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class RobotContainer {

  public final DriveSubsystem drive = new DriveSubsystem();
  public final Intake intake = new Intake();
  public final Launcher launcher = new Launcher();
  private final CommandXboxController controller = new CommandXboxController(0);
  private Boolean fieldRelative = true;

  public RobotContainer() {
    configureBindings();
    drive.setDefaultCommand(drive.driveCommand(controller, () -> fieldRelative));    
    launcher.setDefaultCommand(Commands.run(launcher::stopAll, launcher));
  }

  private void configureBindings() {
    DoubleSupplier fwd = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftY() * DriveSubsystem.kSpeedLimit, 0.02);
    DoubleSupplier str = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftX() * DriveSubsystem.kSpeedLimit, 0.02);

    
    controller.leftTrigger().onTrue(intake.run()).onFalse(intake.stop());
    controller.leftBumper().onTrue(intake.dump()).onFalse(intake.stop());
    controller.rightTrigger().onTrue(launcher.run()).onFalse(launcher.stop());
    controller.rightBumper().whileTrue(
      Commands.parallel(
        new AimAtTargetCommand(drive, fwd, str, () -> fieldRelative, AllianceUtil::getAllianceHubCenter),
        new AutoRPMFromDistanceCommand(drive, launcher, AllianceUtil::getAllianceHubCenter)
      )
    );    
    controller.a().onTrue(Commands.runOnce(drive::zeroHeading, drive));    
    controller.start().onTrue(toggleFieldRelative());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command toggleFieldRelative() {
    return Commands.runOnce(() -> {this.fieldRelative = !this.fieldRelative;});
  }
}
