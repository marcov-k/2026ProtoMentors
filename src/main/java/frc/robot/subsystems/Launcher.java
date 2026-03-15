package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@SuppressWarnings("removal")
public class Launcher extends SubsystemBase{

    private static SparkMaxConfig DefaultConfig = new SparkMaxConfig();    
    private SparkMax LaunchMotor; 
    private SparkMax PreLaunchMotor;    
    private SparkMax HopperMotor;
    public static final int kHopperMotorCanID = 12;
    public static final int kPreLaunchMotorCanID = 10;
    public static final int kLaunchMotorCanID = 9;

    private double targetRpm = 0.0;
    private double targetVoltage = 5.0;
    private double hopperVoltage = 3.5;
    private double prelaunchVoltage = 5.5;
    private DoubleSupplier targetVoltageSupplier;
    

    static {
        DefaultConfig.smartCurrentLimit(50);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(1.5);
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
        targetVoltageSupplier = ()-> targetVoltage;
    }

    @Override
    public void periodic() {        
        SmartDashboard.putNumber("Shooter/Voltage", targetVoltage);        
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetVoltage(double v) {
        targetVoltage = MathUtil.clamp(v, 4.0, 8.0);
    }

    public Command increaseLaunchVoltage() {
        return Commands.runOnce(() -> targetVoltage = Math.min(targetVoltage + .1, 8.0));
    }

    public Command decreaseLaunchVoltage() {
        return Commands.runOnce(() -> targetVoltage = Math.max(targetVoltage - .1, 4.0));
    }

    public void stopAll() {
        HopperMotor.stopMotor();
        PreLaunchMotor.stopMotor();
        LaunchMotor.stopMotor();
    }

    public Command run() {
        return Commands.sequence(
            Commands.runOnce(() -> LaunchMotor.setVoltage(targetVoltageSupplier.getAsDouble())),
            Commands.waitSeconds(1.3), 
            Commands.runOnce(() -> PreLaunchMotor.setVoltage(prelaunchVoltage)),
            Commands.runOnce(() -> HopperMotor.setVoltage(hopperVoltage))
        );
    }

    public Command dump() {
        return Commands.sequence(
            Commands.runOnce(() -> LaunchMotor.setVoltage(-4)),            
            Commands.runOnce(() -> PreLaunchMotor.setVoltage(-4)),
            Commands.runOnce(() -> HopperMotor.setVoltage(-4))
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