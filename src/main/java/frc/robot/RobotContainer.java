// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.field.AllianceUtil;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class RobotContainer {

  public final DriveSubsystem drive = new DriveSubsystem();
  public final Intake intake = new Intake();
  public final Launcher launcher = new Launcher();
  public final Climber climber = new Climber();
  private final CommandXboxController controller = new CommandXboxController(0);
  private Boolean fieldRelative = true;

  public RobotContainer() {
    configureBindings();
    drive.setDefaultCommand(drive.driveCommand(controller, () -> fieldRelative));    
    // launcher.setDefaultCommand(Commands.run(launcher::stopAll, launcher));
  }

  private void configureBindings() {
    DoubleSupplier fwd = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftY() * DriveSubsystem.kSpeedLimit, 0.01);
    DoubleSupplier str = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftX() * DriveSubsystem.kSpeedLimit, 0.01);

    
    controller.leftTrigger().onTrue(intake.run()).onFalse(intake.stop());
    controller.leftBumper().onTrue(intake.dump()).onFalse(intake.stop());
    controller.povUp().onTrue(climber.raise()).onFalse(climber.stop());
    controller.povDown().onTrue(climber.lower()).onFalse(climber.stop());
    controller.povRight().whileTrue(launcher.increaseLaunchVoltage());
    controller.povLeft().whileTrue(launcher.decreaseLaunchVoltage());
    controller.rightTrigger().onTrue(launcher.run()).onFalse(launcher.stop());
    controller.rightBumper().whileTrue(new AimAtTargetCommand(drive, fwd, str, ()-> fieldRelative, AllianceUtil::getAllianceHubCenter));
    /*   
    controller.rightBumper().whileTrue(
      Commands.parallel(
        new AimAtTargetCommand(drive, fwd, str, () -> fieldRelative, AllianceUtil::getAllianceHubCenter),
        new AutoRPMFromDistanceCommand(drive, launcher, AllianceUtil::getAllianceHubCenter)
      )
    );  
    */
    
    controller.y().onTrue(Commands.runOnce(drive::setPoseFromVision)); 
    controller.back().onTrue(Commands.runOnce(drive::zeroHeading, drive));   
    controller.start().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      Commands.print("Driving to Firing Pose"),
      new DriveToTargetCommand(drive, AllianceUtil::getAutonomousFiringPosition),
      Commands.print("Reset pose from vision"),
      Commands.runOnce(() -> drive.setPoseFromVision()),
      Commands.print("Aim at Target"),
      new AimAtTargetCommand(drive, () -> 0, () -> 0, () -> true, AllianceUtil::getAllianceHubCenter),
      Commands.print("Fire"),
      launcher.run(),
      Commands.waitSeconds(5),
      Commands.print("Cease Fire"),
      launcher.stop()
    );
  }


}
