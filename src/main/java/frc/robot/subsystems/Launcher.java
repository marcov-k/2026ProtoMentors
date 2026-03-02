package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.Constants.LauncherConstants;

@SuppressWarnings("removal")
public class Launcher extends SubsystemBase
{
    private SparkMax LaunchMotor;     
    private SparkMax HopperMotor;
    private SparkClosedLoopController LaunchController;
    private RelativeEncoder launchEncoder;
    private double targetRpm = 0.0;

    public Launcher()
    {
        LaunchMotor = new SparkMax(LauncherConstants.kLaunchMotorCanID, MotorType.kBrushless);
        LaunchMotor.configure(LauncherConstants.DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        HopperMotor = new SparkMax(LauncherConstants.kHopperMotorCanID, MotorType.kBrushless);
        HopperMotor.configure(LauncherConstants.DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        LaunchController = LaunchMotor.getClosedLoopController();
        launchEncoder = LaunchMotor.getEncoder();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
        SmartDashboard.putNumber("Shooter/ActualRPM", getActualRpm());
        SmartDashboard.putBoolean("Shooter/AtSetpoint", atSetpoint());
    }

    public void setTargetRpm(double rpm)
    {
        targetRpm = rpm;
        LaunchController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public double getTargetRpm()
    {
        return targetRpm;
    }

    public double getActualRpm()
    {
        return launchEncoder.getVelocity(); // RPM
    }

    public boolean atSetpoint()
    {
        return LaunchController.isAtSetpoint(); // +/- 100 RPM error 
    }

    public void setHopper(double power)
    {
        HopperMotor.set(power);
    }

    public void stopAll()
    {
        HopperMotor.stopMotor();
        LaunchMotor.stopMotor();
    }

    public Command runAtSpeed(double velocity)
    {      
        return Commands.sequence(
            Commands.runOnce(() -> LaunchController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0)),
            Commands.waitUntil(() -> LaunchController.isAtSetpoint()),
            Commands.runOnce(() -> HopperMotor.set(.5))
        );   
    }

    public Command run()
    {
        return Commands.sequence(
            Commands.runOnce(() -> LaunchMotor.set(.75)),
            Commands.runOnce(() -> LaunchMotor.setVoltage(10)),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> HopperMotor.set(.3))
        );
    }

    public Command stop()
    {
        return Commands.runOnce(() -> {
            this.stopAll();
        });
    }
}