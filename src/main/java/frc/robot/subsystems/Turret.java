
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants.TurretState;

public class Turret extends SubsystemBase {
    final private SparkMax turretMotor = new SparkMax(Constants.TurretConstants.kTurretMotorId, MotorType.kBrushless);
    final private SparkMaxConfig turretConfig = new SparkMaxConfig();
    final private Constraints turretPidConstraints = new Constraints(10, 10);
    final private ProfiledPIDController turretPid = new ProfiledPIDController(3, 0.1, 0.1, turretPidConstraints);
    {
        turretConfig.softLimit
            .forwardSoftLimit(0.3).forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0.3).reverseSoftLimitEnabled(true);
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Turret() {

    }

    public void turn(double speed) {
        turretMotor.set(speed);
    }


    public class TurnTurret extends Command {
        private final TurretState turretState;

        public TurnTurret(TurretState turretState) {
            this.turretState = turretState;
            addRequirements(Turret.this);
        }

        @Override
        public void execute() {
            turn(turretState.motorSpeed);
        }
    }

    public class AutoAlign extends Command {
        double error;
        double initializeTime;

        public AutoAlign() {
            addRequirements(Turret.this);
        }
        
        @Override
        public void initialize() {
            initializeTime = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {
            error = Units.degreesToRotations(LimelightHelpers.getTX("limelight"));
            double position = turretMotor.getEncoder().getPosition();
            turretMotor.set(turretPid.calculate(position, position + error));
        }

        @Override
        public boolean isFinished() {
            return error < 0.01 || Timer.getFPGATimestamp() - initializeTime > 2;
        }
    }
}