
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.ArrayDeque;
import java.util.Deque;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants.TurretState;

public class Turret extends SubsystemBase {
    private SparkMax turretSpark;
    private Deque<TurretState> stateQueue;

    public Turret() {
        turretSpark = new SparkMax(Constants.TurretConstants.kTurretSparkId, MotorType.kBrushless);
        stateQueue = new ArrayDeque<TurretState>();
    }

    private void setSpark() {
        TurretState currentTurretState = stateQueue.size() > 0 ? stateQueue.peek() : TurretState.OFF;
        turretSpark.set(currentTurretState.motorSpeed);
    }

    public void enableState(TurretState turretState) {
        stateQueue.remove(turretState);
        stateQueue.addFirst(turretState);
        setSpark();
    }

    public void disableState(TurretState turretState) {
        stateQueue.remove(turretState);
        setSpark();
    }

    public class TurnTurret extends Command {
        public TurretState turretState;

        public TurnTurret(TurretState turretState) {
            this.turretState = turretState;
        }

        @Override
        public void initialize() {
            Turret.this.enableState(turretState);
        }

        @Override
        public void end(boolean i) {
            Turret.this.disableState(turretState);
        }
    }
}