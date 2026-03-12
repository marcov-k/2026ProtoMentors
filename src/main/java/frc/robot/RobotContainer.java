// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class RobotContainer {

  public final DriveSubsystem drive = new DriveSubsystem();
  public final Intake intake = new Intake();
  public final Launcher launcher = new Launcher();
  public final Climber climber = new Climber();
  public final LEDSubsystem led = new LEDSubsystem();
  private final CommandXboxController controller = new CommandXboxController(0);
  private Boolean fieldRelative = true;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Auto Chooser
    autoChooser.setDefaultOption("Do Nothing", null);
    autoChooser.addOption("Path Planner Test", new PathPlannerAuto("Auto1stTest"));
    autoChooser.addOption("Left Side Trench", new PathPlannerAuto("AutoLeftSideTrench"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    drive.setDefaultCommand(drive.driveCommand(controller, () -> fieldRelative));        
  }

  private void configureBindings() {
    DoubleSupplier fwd = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftY() * DriveSubsystem.kSpeedLimit, 0.01);
    DoubleSupplier str = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftX() * DriveSubsystem.kSpeedLimit, 0.01);

    
    controller.leftTrigger().onTrue(intake.run()).onFalse(intake.stop());
    controller.leftBumper().onTrue( Commands.parallel(intake.dump(),launcher.dump())).onFalse( Commands.parallel(intake.stop(),launcher.stop()));
    controller.povUp().onTrue(climber.raise()).onFalse(climber.stop());
    controller.povDown().onTrue(climber.lower()).onFalse(climber.stop());
    controller.povRight().whileTrue(launcher.increaseLaunchVoltage());
    controller.povLeft().whileTrue(launcher.decreaseLaunchVoltage());
    controller.rightTrigger().onTrue( Commands.parallel(intake.run(),launcher.run())).onFalse( Commands.parallel(intake.stop(),launcher.stop()));
    controller.rightBumper().whileTrue(new AimAtTargetCommand(drive, launcher, fwd, str, ()-> fieldRelative));
    controller.y().onTrue(Commands.runOnce(drive::setPoseFromVision));       
    controller.start().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));
  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }


}
