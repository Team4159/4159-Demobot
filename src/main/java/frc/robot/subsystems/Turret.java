
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
<<<<<<< HEAD
import edu.wpi.first.math.util.Units;
=======
>>>>>>> 2782f6be9ec5e8755262fdd838b19ffac37e3b93
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
<<<<<<< HEAD
=======
        private double goalX;
>>>>>>> 2782f6be9ec5e8755262fdd838b19ffac37e3b93

        public AutoAlign() {
        }
        
        @Override
        public void initialize() {
<<<<<<< HEAD
=======
            goalX = turretMotor.getEncoder().getPosition() - LimelightHelpers.getTX("limelight");
>>>>>>> 2782f6be9ec5e8755262fdd838b19ffac37e3b93
        }

        @Override
        public void execute() {
<<<<<<< HEAD
            double error = Units.degreesToRadians(LimelightHelpers.getTX("limelight"));
            double position = turretMotor.getEncoder().getPosition();
            turretMotor.set(turretPid.calculate(position, position + error));
=======
            turretMotor.set(turretPid.calculate(turretMotor.getEncoder().getPosition(), goalX));
>>>>>>> 2782f6be9ec5e8755262fdd838b19ffac37e3b93
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