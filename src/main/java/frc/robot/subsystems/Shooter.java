package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.HoodState;
import frc.robot.Constants.ShooterConstants.ShooterState;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter extends SubsystemBase {
    /*
     * Write subsystem code for the shooter of Demobot, it uses two
     * Neos as the shooter motors as well as a Neo 550 to control
     * the adjustable hood. Neo motors use the revlib library.
     */
    // two Neos
    private final SparkMax leftShooterMotor = new SparkMax(ShooterConstants.kLeftShooterMotorId, MotorType.kBrushless);
    private final SparkMax rightShooterMotor = new SparkMax(ShooterConstants.kRightShooterMotorId,
            MotorType.kBrushless);
    {
        leftShooterMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        rightShooterMotor.configure(new SparkMaxConfig().inverted(true), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }
    // one Neo 550
    private final SparkMax hoodMotor = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);
    {
        hoodMotor.configure(ShooterConstants.kHoodMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public Shooter() {
        adjustHood(HoodState.IDLE.speed);
    }

    @Override
    public void periodic() {
        // motors should be at the same velocity because they are connected to the same
        // axle
        double axleVelocity = getAxleVelocityInRotationsPerSecond();
        double motorVoltage = ShooterConstants.kShooterProfiledPIDController.calculate(axleVelocity);
        // System.out.println("axlevelocity: " + axleVelocity + " motorVoltage: " +
        // motorVoltage + " speed: " +
        // ShooterConstants.kShooterPIDController.getGoal().position);
        leftShooterMotor.setVoltage(motorVoltage);
        rightShooterMotor.setVoltage(motorVoltage);
    }

    public void setSpeed(double speed) {
        ShooterConstants.kShooterProfiledPIDController.reset(getAxleVelocityInRotationsPerSecond());
        // System.out.println(speed);
        ShooterConstants.kShooterProfiledPIDController.setGoal(speed);
    }

    public void adjustHood(double speed) {
        hoodMotor.set(speed);
    }

    public void zeroHood() {
        hoodMotor.getEncoder().setPosition(0.0);
    }

    public void enableHoodReverseSoftLimit(boolean enabled) {
        var reverseConfig = new SparkMaxConfig();
        reverseConfig.softLimit.reverseSoftLimitEnabled(enabled);
        hoodMotor.configure(reverseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public boolean isShooterReady() {
        if (ShooterConstants.kShooterProfiledPIDController.getGoal().velocity <= 0) {
            // must be spinning in the positive direction to be shooting
            return false;
        }
        return ShooterConstants.kShooterProfiledPIDController.atGoal();
    }

    private double getAxleVelocityInRotationsPerSecond() {
        return RotationsPerSecond.convertFrom(leftShooterMotor.getEncoder().getVelocity(), RPM);
    }

    public class AdjustHood extends Command {
        private final HoodState state;

        public AdjustHood(HoodState state) {
            this.state = state;
            addRequirements(Shooter.this);
        }

        @Override
        public void execute() {
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
