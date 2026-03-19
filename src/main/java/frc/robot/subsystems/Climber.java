package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@SuppressWarnings("removal")
public class Climber extends SubsystemBase{

    private static SparkMaxConfig DefaultConfig = new SparkMaxConfig();    
    private SparkMax ClimberMotor;     
    public static final int kClimberMotorCanID = 11;

    static {
        DefaultConfig.smartCurrentLimit(20);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(1.0);
        DefaultConfig.inverted(false);
    }

    public Climber() {
        ClimberMotor = new SparkMax(kClimberMotorCanID, MotorType.kBrushed);
        ClimberMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command raise() {
        return Commands.runOnce(() -> ClimberMotor.set(1.0));
    }


    public Command lower() {
        return Commands.runOnce(() -> ClimberMotor.set(-1.0));
    }
    public Command stop() {
        return Commands.runOnce(() -> ClimberMotor.stopMotor());
    }
}
