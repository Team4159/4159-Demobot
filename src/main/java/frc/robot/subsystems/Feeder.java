package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;

public class Feeder extends SubsystemBase {
    private final SparkMax leftMotor = new SparkMax(FeederConstants.kLeftMotorId, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(FeederConstants.kRightMotorId, MotorType.kBrushless);
    {
        var leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(false);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        var rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(true);
        rightMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Feeder() {
        //leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void setState(FeederState state) {
        leftMotor.set(state.speed);
        //rightMotor.set(state.speed);
    }

    public class ChangeState extends Command {
        FeederState state;

        public ChangeState(FeederState state) {
            this.state = state;
            addRequirements(Feeder.this);
        }

        @Override
        public void initialize() {
            Feeder.this.setState(state);
        }

        @Override
        public void end(boolean interrupted) {
            Feeder.this.setState(FeederState.IDLE);
        }
    }
}
