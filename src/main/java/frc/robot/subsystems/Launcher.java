package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@SuppressWarnings("removal")
public class Launcher extends SubsystemBase{

    private static SparkMaxConfig DefaultConfig = new SparkMaxConfig();    
    private SparkMax LaunchMotor; 
    private SparkMax PreLaunchMotor;    
    private SparkMax HopperMotor;
    public static final int kHopperMotorCanID = 9;
    public static final int kPreLaunchMotorCanID = 12;
    public static final int kLaunchMotorCanID = 10;
    private SparkClosedLoopController LaunchController;
    private RelativeEncoder launchEncoder;
    private double targetRpm = 0.0;

    static {
        DefaultConfig.smartCurrentLimit(50);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(1.0);
        DefaultConfig.inverted(true);
        DefaultConfig.voltageCompensation(12);
        DefaultConfig.closedLoop.allowedClosedLoopError(100.0, ClosedLoopSlot.kSlot0);
    }

    public Launcher() {
        LaunchMotor = new SparkMax(kLaunchMotorCanID, MotorType.kBrushless);
        LaunchMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        PreLaunchMotor = new SparkMax(kPreLaunchMotorCanID, MotorType.kBrushless);
        PreLaunchMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        HopperMotor = new SparkMax(kHopperMotorCanID, MotorType.kBrushless);
        HopperMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // LaunchController = LaunchMotor.getClosedLoopController();
        // launchEncoder = LaunchMotor.getEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
        // SmartDashboard.putNumber("Shooter/ActualRPM", getActualRpm());
        // SmartDashboard.putBoolean("Shooter/AtSetpoint", atSetpoint());
    }

/*     public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        LaunchController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    } */

    public double getTargetRpm() {
        return targetRpm;
    }

/*     public double getActualRpm() {
        return launchEncoder.getVelocity(); // RPM
    } */

/*     public boolean atSetpoint() {
        return LaunchController.isAtSetpoint(); // +/- 100 RPM error 
    } */

    public void setHopper(double power) {
        HopperMotor.set(power);
    }

    public void stopAll() {
        HopperMotor.stopMotor();
        PreLaunchMotor.stopMotor();
        LaunchMotor.stopMotor();
    }

/*     public Command runAtSpeed(double velocity) {      
        return Commands.sequence(
            Commands.runOnce(() -> LaunchController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0)),
            Commands.waitUntil(() -> LaunchController.isAtSetpoint()),
            Commands.runOnce(() -> PreLaunchMotor.set(0.65)),
            Commands.runOnce(() -> HopperMotor.set(.5))
        );   
    } */

    public Command run() {
        return Commands.sequence(
            Commands.runOnce(() -> LaunchMotor.setVoltage(6.5)),
            Commands.waitSeconds(1.0), 
            Commands.runOnce(() -> PreLaunchMotor.set(0.6)),
            Commands.runOnce(() -> HopperMotor.set(.75))
        );
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            HopperMotor.stopMotor(); 
            PreLaunchMotor.stopMotor();
            LaunchMotor.stopMotor();
        });
    }
}