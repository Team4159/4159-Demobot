package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;

public class Feeder extends SubsystemBase {

    private final SparkMax leftMotor = new SparkMax(FeederConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(FeederConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    {
        leftMotor.configure(
            FeederConstants.LEFT_FEEDER_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        rightMotor.configure(
            FeederConstants.RIGHT_FEEDER_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public Feeder() {}

    public void setState(FeederState state) {
        leftMotor.set(state.dutyCycle);
        rightMotor.set(state.dutyCycle);
    }

    public class ChangeState extends Command {

        private final FeederState state;

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
