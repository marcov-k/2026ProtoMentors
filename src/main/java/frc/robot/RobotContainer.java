// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final DriveSubsystem drive = new DriveSubsystem();
  public final Intake intake = new Intake();
  public final Launcher launcher = new Launcher();
  private final CommandXboxController controller = new CommandXboxController(0);
  private Boolean fieldRelative = true;

  public RobotContainer() {
    configureBindings();
    CommandScheduler.getInstance().setDefaultCommand(drive, drive.driveCommand(controller, fieldRelative));
  }

  private void configureBindings() {
    controller.leftTrigger().onTrue(intake.run()).onFalse(intake.stop());
    controller.leftBumper().onTrue(intake.dump()).onFalse(intake.stop());
    controller.rightTrigger().onTrue(launcher.run()).onFalse(launcher.stop());
    controller.rightBumper().onTrue(launcher.runAtSpeed(3800)).onFalse(launcher.stop());
    controller.a().onTrue(Commands.runOnce(drive::zeroHeading, drive));
    controller.start().onTrue(toggleFieldRelative());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command toggleFieldRelative() {
    return Commands.run(() -> {this.fieldRelative = !this.fieldRelative;});
  }
}
