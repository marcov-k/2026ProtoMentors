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
public class Intake extends SubsystemBase{

    private static SparkMaxConfig DefaultConfig = new SparkMaxConfig();    
    private SparkMax IntakeMotor;     
    public static final int kIntakeMotorCanID = 13;

    static {
        DefaultConfig.smartCurrentLimit(50);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(1.0);
        DefaultConfig.inverted(true);
    }

    public Intake() {
        IntakeMotor = new SparkMax(kIntakeMotorCanID, MotorType.kBrushless);
        IntakeMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command run() {
        return Commands.runOnce(() -> IntakeMotor.set(.50));
    }


    public Command dump() {
        return Commands.runOnce(() -> IntakeMotor.set(-.50));
    }
    public Command stop() {
        return Commands.runOnce(() -> IntakeMotor.stopMotor());
    }
}
