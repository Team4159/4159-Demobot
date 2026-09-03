package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
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
            new SparkMaxConfig().inverted(false),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        rightShooterMotor.configure(
            new SparkMaxConfig().inverted(true),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    // one Neo 550
    private final SparkMax hoodMotor = new SparkMax(ShooterConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
    private double lastHoodAngle = hoodMotor.getEncoder().getPosition();

    {
        hoodMotor.configure(
            ShooterConstants.HOOD_MOTOR_CONFIG,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public Shooter() {
        adjustHood(HoodState.IDLE.speed);
    }

    @Override
    public void periodic() {
        if (ShooterConstants.SHOOTER_PROFILED_PID_CONTROLLER.getGoal().position != 0.0) {
            double motorVoltage = ShooterConstants.SHOOTER_PROFILED_PID_CONTROLLER.calculate(
                getAxleVelocity().in(RotationsPerSecond)
            );
            leftShooterMotor.setVoltage(motorVoltage);
            rightShooterMotor.setVoltage(motorVoltage);
        } else {
            leftShooterMotor.stopMotor();
            rightShooterMotor.stopMotor();
        }
        // TODO: add code to preserve last hood angle with pid
    }

    public void setSpeed(double speed) {
        ShooterConstants.SHOOTER_PROFILED_PID_CONTROLLER.reset(getAxleVelocity().in(RotationsPerSecond));
        ShooterConstants.SHOOTER_PROFILED_PID_CONTROLLER.setGoal(speed);
    }

    public void adjustHood(double speed) {
        hoodMotor.set(speed);
    }

    public boolean isShooterReady() {
        if (ShooterConstants.SHOOTER_PROFILED_PID_CONTROLLER.getGoal().velocity <= 0) {
            // must be spinning in the positive direction to be shooting
            return false;
        }
        return ShooterConstants.SHOOTER_PROFILED_PID_CONTROLLER.atGoal();
    }

    private AngularVelocity getAxleVelocity() {
        return getLeftVelocity().plus(getRightVelocity()).div(2);
    }

    private AngularVelocity getLeftVelocity() {
        return RPM.of(leftShooterMotor.getEncoder().getVelocity());
    }

    private AngularVelocity getRightVelocity() {
        return RPM.of(rightShooterMotor.getEncoder().getVelocity());
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
            adjustHood(state.speed);
        }

        @Override
        public void end(boolean interrupted) {
            adjustHood(HoodState.IDLE.speed);
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
            Shooter.this.setSpeed(shooterState.speed);
        }

        @Override
        public void end(boolean interupted) {
            Shooter.this.setSpeed(ShooterState.IDLE.speed);
        }
    }
}
