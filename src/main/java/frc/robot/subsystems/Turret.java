
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants.TurretState;

public class Turret extends SubsystemBase {
    private SparkMax turretMotor;

    public Turret() {
        turretMotor = new SparkMax(Constants.TurretConstants.kTurretMotorId, MotorType.kBrushless);
    }

    public class TurnTurret extends Command {
        public TurretState turretState;

        public TurnTurret(TurretState turretState) {
            this.turretState = turretState;
        }

        @Override
        public void initialize() {
            turretMotor.set(turretState.motorSpeed);
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