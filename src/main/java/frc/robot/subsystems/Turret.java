
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.RumbleConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretState;
import frc.robot.lib.HIDRumble;
import frc.robot.lib.HIDRumble.RumbleRequest;

public class Turret extends SubsystemBase {
    private final SparkMax turretMotor = new SparkMax(Constants.TurretConstants.kTurretMotorId, MotorType.kBrushless);
    {
        turretMotor.configure(TurretConstants.kTurretMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public Turret() {

    }

    private void setState(TurretState turretState) {
        turretMotor.set(turretState.motorSpeed);
    }

    public class TurnTurret extends Command {
        private TurretState turretState;

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

    public class TurretPositionControl extends Command {
        private final CommandXboxController controller;
        private double turretSetpoint;

        public TurretPositionControl(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Turret.this);
        }

        @Override
        public void initialize() {
            turretSetpoint = 0;
            TurretConstants.kTurretProfiledPIDController.reset(turretMotor.getEncoder().getPosition());
        }

        @Override
        public void execute() {
            double inputX = controller.getRightX();
            double inputY = controller.getRightY();
            double magnitude = Math.hypot(inputX, inputY);

            if (magnitude > TurretConstants.kInputDeadzone) {
                double angle = Math.atan2(inputY, inputX);
                // normalizes angle while scaling
                // note: negative 90 degrees is up
                double angleFromVertical = Units.radiansToRotations(TurretConstants.kAngleScalar
                        * (((angle + Units.degreesToRadians(90) + Units.degreesToRadians(180))
                                % Units.degreesToRadians(360))
                                - Units.degreesToRadians(180)));
                boolean angleWithinRange = angleFromVertical >= TurretConstants.kTurretAngleMinimum
                        && angleFromVertical <= TurretConstants.kTurretAngleMaximum;

                // convert turret position to rotations
                if (angleWithinRange) {
                    turretSetpoint = angleFromVertical;
                    HIDRumble.rumble(controller,
                            new RumbleRequest(RumbleType.kLeftRumble, RumbleConstants.kTurretTurnFeedbackValue, 0));
                }
            }

            double motorSetpoint = turretSetpoint * TurretConstants.kTurretMotorGearRatio;
            double pidVoltage = TurretConstants.kTurretProfiledPIDController
                    .calculate(turretMotor.getEncoder().getPosition(), motorSetpoint);
            double feedforwardVoltage = TurretConstants.kTurretFeedforward
                    .calculate(TurretConstants.kTurretProfiledPIDController.getSetpoint().velocity);
            turretMotor.setVoltage(pidVoltage + feedforwardVoltage);
        }
    }
}