package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants.RumbleConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final SparkMax turretMotor = new SparkMax(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

    {
        turretMotor.configure(
            TurretConstants.TURRET_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public Turret() {}

    public class TurretPositionControl extends Command {

        private final CommandXboxController controller;
        private Angle turretSetpoint;

        private boolean previousTurretSetpointWithinRange;
        private Angle previousWantedTurretSetpoint;

        public TurretPositionControl(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Turret.this);
        }

        @Override
        public void initialize() {
            turretSetpoint = Rotations.of(0.0);
            previousTurretSetpointWithinRange = false;
            previousWantedTurretSetpoint = turretSetpoint;
            TurretConstants.TURRET_PROFILED_PID_CONTROLLER.reset(turretMotor.getEncoder().getPosition());
        }

        @Override
        public void execute() {
            double inputX = controller.getRightX();
            double inputY = controller.getRightY();
            double magnitude = Math.hypot(inputX, inputY);

            if (magnitude >= TurretConstants.INPUT_DEADZONE) {
                double inputAngle = Math.atan2(inputY, inputX);
                double lastInputAngle = turretSetpoint.in(Rotations) / TurretConstants.INPUT_ANGLE_SCALAR;
                // normalizes angle while scaling
                // note: negative 90 degrees is up
                double desiredAngle =
                    Math.abs(inputAngle - lastInputAngle) >= TurretConstants.INPUT_ANGLE_JITTER_BUFFER
                        ? inputAngle
                        : lastInputAngle;
                Angle wantedTurretSetpoint = Radians.of(
                    TurretConstants.INPUT_ANGLE_SCALAR *
                        (((desiredAngle + Units.degreesToRadians(90) + Units.degreesToRadians(180)) %
                            Units.degreesToRadians(360)) -
                            Units.degreesToRadians(180))
                );
                boolean turretSetpointWithinRange =
                    wantedTurretSetpoint.compareTo(TurretConstants.TURRET_ANGLE_MINIMUM) >= 0 &&
                    wantedTurretSetpoint.compareTo(TurretConstants.TURRET_ANGLE_MAXIMUM) <= 0;

                // convert turret position to rotations
                if (turretSetpointWithinRange) {
                    turretSetpoint = wantedTurretSetpoint;
                    HIDRumble.rumble(
                        controller,
                        new RumbleRequest(RumbleType.kLeftRumble, RumbleConstants.TURRET_TURN_STRENGTH, 0)
                    );
                } else if (previousTurretSetpointWithinRange) {
                    if (previousWantedTurretSetpoint.magnitude() > 0) {
                        turretSetpoint = TurretConstants.TURRET_ANGLE_MAXIMUM;
                    } else {
                        turretSetpoint = TurretConstants.TURRET_ANGLE_MINIMUM;
                    }
                    HIDRumble.rumble(
                        controller,
                        new RumbleRequest(RumbleType.kRightRumble, RumbleConstants.TURRET_TRIP_STRENGTH, 0.3, 0)
                    );
                }
                previousTurretSetpointWithinRange = turretSetpointWithinRange;
                previousWantedTurretSetpoint = wantedTurretSetpoint;
            }

            double motorSetpoint = turretSetpoint.in(Rotations) * TurretConstants.TURRET_MOTOR_GEAR_RATIO;
            double pidVoltage = TurretConstants.TURRET_PROFILED_PID_CONTROLLER.calculate(
                turretMotor.getEncoder().getPosition(),
                motorSetpoint
            );
            double feedforwardVoltage = TurretConstants.TURRET_FEED_FORWARD.calculate(
                TurretConstants.TURRET_PROFILED_PID_CONTROLLER.getSetpoint().velocity
            );
            turretMotor.set((pidVoltage + feedforwardVoltage) / 12.0);
        }

        public void zeroTurret() {
            turretMotor.set(0.0);
            turretSetpoint = Rotations.of(0.0);
            TurretConstants.TURRET_PROFILED_PID_CONTROLLER.reset(0.0);
            turretMotor.getEncoder().setPosition(0.0);
        }
    }
}
