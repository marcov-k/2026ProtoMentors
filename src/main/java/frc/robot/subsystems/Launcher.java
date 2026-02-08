package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    private SparkMax HopperMotor;
    public static final int kHopperMotorCanID = 10;
    public static final int kLaunchMotorCanID = 11;
    private SparkClosedLoopController LaunchController;

    static {
        DefaultConfig.smartCurrentLimit(50);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(1.0);
        DefaultConfig.inverted(true);
        DefaultConfig.closedLoop.allowedClosedLoopError(100.0, ClosedLoopSlot.kSlot0);
    }

    public Launcher() {
        LaunchMotor = new SparkMax(kLaunchMotorCanID, MotorType.kBrushless);
        LaunchMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        HopperMotor = new SparkMax(kHopperMotorCanID, MotorType.kBrushless);
        HopperMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        LaunchController = LaunchMotor.getClosedLoopController();
        

    }

    public Command runAtSpeed(double velocity) {      
        return Commands.sequence(
            Commands.runOnce(() -> LaunchController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0)),
            Commands.waitUntil(() -> LaunchController.isAtSetpoint()),
            Commands.runOnce(() -> HopperMotor.set(.5))
        );   
    }

    public Command run() {
        return Commands.sequence(
            Commands.runOnce(() ->  LaunchMotor.set(.75)),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> HopperMotor.set(.5))
        );
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            HopperMotor.stopMotor(); 
            LaunchMotor.stopMotor();
        });
    }
}