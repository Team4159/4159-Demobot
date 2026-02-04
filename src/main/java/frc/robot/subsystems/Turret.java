
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
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants;
import frc.robot.Constants.RumbleConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final SparkMax turretMotor = new SparkMax(Constants.TurretConstants.kTurretMotorId, MotorType.kBrushless);
    {
        turretMotor.configure(TurretConstants.kTurretMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public Turret() {
    }

    public class TurretPositionControl extends Command {
        private final CommandXboxController controller;
        private double turretSetpoint;

        private boolean previousTurretSetpointWithinRange;
        private double previousWantedTurretSetpoint;

        public TurretPositionControl(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Turret.this);
        }

        @Override
        public void initialize() {
            turretSetpoint = 0;
            previousTurretSetpointWithinRange = false;
            previousWantedTurretSetpoint = turretSetpoint;
            TurretConstants.kTurretProfiledPIDController.reset(turretMotor.getEncoder().getPosition());
        }

        @Override
        public void execute() {
            double inputX = controller.getRightX();
            double inputY = controller.getRightY();
            double magnitude = Math.hypot(inputX, inputY);

            if (magnitude >= TurretConstants.kInputDeadzone) {
                double angle = Math.atan2(inputY, inputX);
                // normalizes angle while scaling
                // note: negative 90 degrees is up
                double wantedTurretSetpoint = Units.radiansToRotations(TurretConstants.kAngleScalar
                        * (((angle + Units.degreesToRadians(90) + Units.degreesToRadians(180))
                                % Units.degreesToRadians(360))
                                - Units.degreesToRadians(180)));
                boolean turretSetpointWithinRange = wantedTurretSetpoint >= TurretConstants.kTurretAngleMinimum
                        && wantedTurretSetpoint <= TurretConstants.kTurretAngleMaximum;

                // convert turret position to rotations
                if (turretSetpointWithinRange) {
                    turretSetpoint = wantedTurretSetpoint;
                    HIDRumble.rumble(controller,
                            new RumbleRequest(RumbleType.kLeftRumble, RumbleConstants.kTurretTurnStrength, 0));
                } else if (previousTurretSetpointWithinRange) {
                    if (previousWantedTurretSetpoint > 0) {
                        turretSetpoint = TurretConstants.kTurretAngleMaximum;
                    } else {
                        turretSetpoint = TurretConstants.kTurretAngleMinimum;
                    }
                    HIDRumble.rumble(controller,
                            new RumbleRequest(RumbleType.kRightRumble, RumbleConstants.kTurretTripStrength, 0, 0.3));
                }
                previousTurretSetpointWithinRange = turretSetpointWithinRange;
                previousWantedTurretSetpoint = wantedTurretSetpoint;
            }

            double motorSetpoint = turretSetpoint * TurretConstants.kTurretMotorGearRatio;
            double pidVoltage = TurretConstants.kTurretProfiledPIDController
                    .calculate(turretMotor.getEncoder().getPosition(), motorSetpoint);
            double feedforwardVoltage = TurretConstants.kTurretFeedforward
                    .calculate(TurretConstants.kTurretProfiledPIDController.getSetpoint().velocity);
            turretMotor.set((pidVoltage + feedforwardVoltage) / 12.0);
        }

        public void zeroTurretMotor() {
            turretSetpoint = 0;
            TurretConstants.kTurretProfiledPIDController.reset(0);
            turretMotor.getEncoder().setPosition(0);
            turretMotor.set(0);
        }
    }
}