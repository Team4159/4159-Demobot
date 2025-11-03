
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants.TurretState;

public class Turret extends SubsystemBase {
    private SparkMax turretSpark;

    public Turret() {
        turretSpark = new SparkMax(Constants.TurretConstants.kTurretSparkId, MotorType.kBrushless);
    }

    private void setState(TurretState turretState) {
        turretSpark.set(turretState.motorSpeed);
    }

    public class TurnTurret extends Command {
        public TurretState turretState;

        public TurnTurret(TurretState turretState) {
            this.turretState = turretState;
            addRequirements(Turret.this);
        }

        @Override
        public void initialize() {
            Turret.this.setState(turretState);
        }

        @Override
        public void end(boolean i) {
            Turret.this.setState(TurretState.IDLE);        
        }
    }
}