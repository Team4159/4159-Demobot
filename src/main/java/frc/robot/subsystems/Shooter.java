package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.HoodState;
import frc.robot.Constants.ShooterConstants.ShooterState;

public class Shooter extends SubsystemBase {

    /*
     * Write subsystem code for the shooter of Demobot, it uses two
     * Neos as the shooter motors as well as a Neo 550 to control
     * the adjustable hood. Neo motors use the revlib library.
     */
    // two Neos
    private final SparkMax leftShooterMotor = new SparkMax(
        ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
        MotorType.kBrushless
    );
    private final SparkMax rightShooterMotor = new SparkMax(
        ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
        MotorType.kBrushless
    );

    {
        leftShooterMotor.configure(
            ShooterConstants.LEFT_SHOOTER_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        rightShooterMotor.configure(
            ShooterConstants.RIGHT_SHOOTER_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    // one Neo 550
    private final SparkMax hoodMotor = new SparkMax(ShooterConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
    private double lastHoodAngle = hoodMotor.getEncoder().getPosition();

    {
        hoodMotor.configure(
            ShooterConstants.HOOD_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public Shooter() {
        adjustHood(HoodState.IDLE.dutyCycle);
    }

    public void setDutyCycle(double dutyCycle) {
        leftShooterMotor.set(dutyCycle);
        rightShooterMotor.set(dutyCycle);
    }

    public void stop() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }

    public void adjustHood(double dutyCycle) {
        hoodMotor.set(dutyCycle);
    }

    public class AdjustHood extends Command {

        private final HoodState state;

        public AdjustHood(HoodState state) {
            this.state = state;
            addRequirements(Shooter.this);
        }

        @Override
        public void execute() {
            lastHoodAngle = hoodMotor.getEncoder().getPosition();
            adjustHood(state.dutyCycle);
        }

        @Override
        public void end(boolean interrupted) {
            adjustHood(HoodState.IDLE.dutyCycle);
        }
    } // end change hood command

    // controls the spin of the shooter
    public class ControlSpin extends Command {

        private final ShooterState shooterState;

        public ControlSpin(ShooterState shooterState) {
            this.shooterState = shooterState;
        }

        @Override
        public void initialize() {
            Shooter.this.setDutyCycle(shooterState.dutyCycle);
        }

        @Override
        public void end(boolean interupted) {
            Shooter.this.setDutyCycle(ShooterState.IDLE.dutyCycle);
        }
    }
}
