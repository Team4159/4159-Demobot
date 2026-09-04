package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArcadeDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {

    private final TalonFX leftMotor1 = new TalonFX(DrivetrainConstants.LEFT_MOTOR_1_ID);
    private final TalonFX leftMotor2 = new TalonFX(DrivetrainConstants.LEFT_MOTOR_2_ID);
    private final TalonFX rightMotor1 = new TalonFX(DrivetrainConstants.RIGHT_MOTOR_1_ID);
    private final TalonFX rightMotor2 = new TalonFX(DrivetrainConstants.RIGHT_MOTOR_2_ID);

    {
        leftMotor1.getConfigurator().apply(DrivetrainConstants.DRIVETRAIN_LEFT_MOTOR_CONFIG);
        leftMotor2.getConfigurator().apply(DrivetrainConstants.DRIVETRAIN_LEFT_MOTOR_CONFIG);
        rightMotor1.getConfigurator().apply(DrivetrainConstants.DRIVETRAIN_RIGHT_MOTOR_CONFIG);
        rightMotor2.getConfigurator().apply(DrivetrainConstants.DRIVETRAIN_RIGHT_MOTOR_CONFIG);
    }

    public Drivetrain() {}

    public TalonFX[] getMotors() {
        return new TalonFX[] { leftMotor1, leftMotor2, rightMotor1, rightMotor2 };
    }

    public void drive(double leftVelocity, double rightVelocity) {
        leftMotor1.set(leftVelocity);
        leftMotor2.set(leftVelocity);
        rightMotor1.set(rightVelocity);
        rightMotor2.set(rightVelocity);
    }

    public void stop() {
        leftMotor1.stopMotor();
        leftMotor2.stopMotor();
        rightMotor1.stopMotor();
        rightMotor2.stopMotor();
    }

    public class TankDrive extends Command {

        private final CommandXboxController controller;

        public TankDrive(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Drivetrain.this);
        }

        @Override
        public void execute() {
            Drivetrain.this.drive(
                controller.getLeftY() * OperatorConstants.DRIVETRAIN_VELOCITY_FACTOR,
                controller.getRightY() * OperatorConstants.DRIVETRAIN_VELOCITY_FACTOR
            );
        }

        @Override
        public void end(boolean interrupted) {
            Drivetrain.this.stop();
        }
    }

    public class ArcadeDrive extends Command {

        private final CommandXboxController controller;

        public ArcadeDrive(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Drivetrain.this);
        }

        @Override
        public void execute() {
            Pair<Double, Double> velocities = getVelocities(controller.getLeftX(), controller.getLeftY());
            Drivetrain.this.drive(
                velocities.getFirst() * OperatorConstants.DRIVETRAIN_VELOCITY_FACTOR,
                velocities.getSecond() * OperatorConstants.DRIVETRAIN_VELOCITY_FACTOR
            );
        }

        @Override
        public void end(boolean interrupted) {
            Drivetrain.this.stop();
        }

        private Pair<Double, Double> getVelocities(double inputX, double inputY) {
            double forwardDirection = Math.signum(inputY);
            double rawMagnitude = Math.min(1, Math.hypot(inputX, inputY));
            double correctedMagnitude = MathUtil.applyDeadband(rawMagnitude, ArcadeDriveConstants.INPUT_DEADZONE, 1);

            // absolute angles
            double absoluteAngleFromHorizontal = Math.atan2(Math.abs(inputY), Math.abs(inputX));
            double absoluteAngleFromVertical = Math.abs(
                Units.degreesToRadians(90) - Math.abs(absoluteAngleFromHorizontal)
            );

            double leftDirection = 0,
                rightDirection = 0;
            if (rawMagnitude >= ArcadeDriveConstants.INPUT_DEADZONE) {
                double rotationAlpha;
                if (absoluteAngleFromVertical <= ArcadeDriveConstants.TRANSLATION_BUFFER_ANGLE) {
                    rotationAlpha = forwardDirection;
                } else if (absoluteAngleFromHorizontal <= ArcadeDriveConstants.ROTATION_BUFFER_ANGLE) {
                    rotationAlpha = 0;
                } else {
                    // range of analog motion that is outside of the buffer zones
                    double analogRange =
                        Units.degreesToRadians(90) -
                        (ArcadeDriveConstants.ROTATION_BUFFER_ANGLE + ArcadeDriveConstants.TRANSLATION_BUFFER_ANGLE);
                    double relativeAngle = absoluteAngleFromHorizontal - ArcadeDriveConstants.ROTATION_BUFFER_ANGLE;
                    rotationAlpha = MathUtil.clamp(forwardDirection * (relativeAngle / analogRange), -1, 1);
                }

                // get directions
                // 90 to 0 degrees: lerp from (-1, -1) to (-1, 1)
                // 0 to -90 degrees: lerp from (-1, 1) to (1, 1)
                leftDirection = -1 + Math.max(0, rotationAlpha * 2);
                leftDirection = MathUtil.clamp(leftDirection, -1, 1);
                rightDirection = -1 + (rotationAlpha + 1) * 2;
                rightDirection = MathUtil.clamp(rightDirection, -1, 1);

                // reverse inputs if the input is leftward
                if (inputX < 0) {
                    double temp = leftDirection;
                    leftDirection = rightDirection;
                    rightDirection = temp;
                }
            }

            // get velocities based off direction and input magnitude
            double leftVelocity = leftDirection * correctedMagnitude;
            leftVelocity = MathUtil.clamp(leftVelocity, -1, 1);
            double rightVelocity = rightDirection * correctedMagnitude;
            rightVelocity = MathUtil.clamp(rightVelocity, -1, 1);

            return Pair.of(leftVelocity, rightVelocity);
        }
    }
}
