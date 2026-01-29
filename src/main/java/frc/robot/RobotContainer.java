// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  public final DriveSubsystem drive = new DriveSubsystem();
  private final XboxController controller = new XboxController(0);

  public RobotContainer() {
    configureBindings();
    CommandScheduler.getInstance().setDefaultCommand(drive, drive.driveCommand(controller, false));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
