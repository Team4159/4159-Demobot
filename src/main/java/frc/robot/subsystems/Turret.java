
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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
    private final SparkMax turretMotor = new SparkMax(Constants.TurretConstants.kTurretSparkId, MotorType.kBrushless);
    {
        turretMotor.configure(TurretConstants.kTurretMotorConfig, ResetMode.kResetSafeParameters,
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
        private double setpoint = 0;
        private double previousSetpoint = setpoint;

        public TurretPositionControl(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Turret.this);
        }

        @Override
        public void initialize() {
            setpoint = 0;
            previousSetpoint = setpoint;
            TurretConstants.kTurretProfiledPIDController.reset(turretMotor.getEncoder().getPosition());
        }

        @Override
        public void execute() {
            double inputX = controller.getRightX();
            double inputY = controller.getRightY();
            double magnitude = Math.hypot(inputX, inputY);

            if (magnitude > TurretConstants.kInputDeadzone) {
                double angle = Math.atan2(inputY, inputX);
                // also normalizes angle while converting
                // note:negative 90 degrees is up
                double angleFromVertical = TurretConstants.kInputScalar
                        * (((angle + Units.degreesToRadians(90 + 180)) % Units.degreesToRadians(360))
                                - Units.degreesToRadians(180));

                // get turret position in radians
                double positionRadians = MathUtil.clamp(angleFromVertical,
                        Units.rotationsToRadians(-TurretConstants.kTurretReverseLimit),
                        Units.rotationsToRadians(TurretConstants.kTurretForwardLimit));

                // convert turret position to rotations
                previousSetpoint = setpoint;
                setpoint = Units.radiansToRotations(positionRadians);

                // rumble if input is within range to let driver know the turret turning to a
                // new setpoint
                boolean setpointWithinRange = previousSetpoint > -TurretConstants.kTurretReverseLimit
                        && previousSetpoint < TurretConstants.kTurretForwardLimit;
                boolean setpointChanged = (setpoint != previousSetpoint);
                if (setpointWithinRange) {
                    HIDRumble.rumble(controller,
                            new RumbleRequest(RumbleType.kLeftRumble, RumbleConstants.kTurretTurnFeedbackValue, 0));
                } else if (setpointChanged) {
                    HIDRumble.rumble(controller, new RumbleRequest(RumbleType.kLeftRumble,
                            RumbleConstants.kTurretTurnFeedbackValue, 0, 0.2));
                }
            }

            double correctedSetpoint = setpoint * TurretConstants.kTurretMotorGearRatio;
            double pidVoltage = TurretConstants.kTurretProfiledPIDController
                    .calculate(turretMotor.getEncoder().getPosition(), correctedSetpoint);
            double feedforwardVoltage = TurretConstants.kTurretFeedForward.calculateWithVelocities(
                    turretMotor.getEncoder().getVelocity(),
                    TurretConstants.kTurretFeedForwardSpeed * Math.signum(setpoint - previousSetpoint));
            turretMotor.setVoltage(pidVoltage + feedforwardVoltage);
        }
    }
}