
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
        private double goalX;

        public AutoAlign() {
        }
        
        @Override
        public void initialize() {
            goalX = turretMotor.getEncoder().getPosition() - LimelightHelpers.getTX("limelight");
        }

        @Override
        public void execute() {
            turretMotor.set(turretPid.calculate(turretMotor.getEncoder().getPosition(), goalX));
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            
        }
    }
}