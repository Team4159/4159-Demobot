
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants.TurretState;

public class Turret extends SubsystemBase {
    final private SparkMax turretMotor = new SparkMax(Constants.TurretConstants.kTurretMotorId, MotorType.kBrushless);
    final private Constraints turretPidConstraints = new Constraints(10, 10);
    final private ProfiledPIDController turretPid = new ProfiledPIDController(3, 0.1, 0.1, turretPidConstraints);

    public Turret() {

    }

    public class TurnTurret extends Command {
        public TurretState turretState;

        public TurnTurret(TurretState turretState) {
            this.turretState = turretState;
        }

        @Override
        public void execute() {
            // do nothing
        }
    }

    public class AutoAlign extends Command {

        public AutoAlign() {
        }
        
        @Override
        public void initialize() {
        }

        @Override
        public void execute() {
            double error = Units.degreesToRadians(LimelightHelpers.getTX("limelight"));
            double position = turretMotor.getEncoder().getPosition();
            turretMotor.set(turretPid.calculate(position, position + error));
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("help");
        }
    }
}