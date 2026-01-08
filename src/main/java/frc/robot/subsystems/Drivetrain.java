package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.ArcadeDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RumbleConstants;
import frc.robot.lib.HIDRumble.RumbleRequest;
import frc.robot.lib.HIDRumble;

public class Drivetrain extends SubsystemBase {
    private final TalonFX leftMotor1 = new TalonFX(Constants.DrivetrainConstants.kLeftMotor1Id);
    private final TalonFX leftMotor2 = new TalonFX(Constants.DrivetrainConstants.kLeftMotor2Id);
    private final TalonFX rightMotor1 = new TalonFX(Constants.DrivetrainConstants.kRightMotor1Id);
    private final TalonFX rightMotor2 = new TalonFX(Constants.DrivetrainConstants.kRightMotor2Id);
    {
        var leftMotorConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        leftMotor1.getConfigurator().apply(leftMotorConfig);
        leftMotor2.getConfigurator().apply(leftMotorConfig);

        var rightMotorConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        rightMotor1.getConfigurator().apply(rightMotorConfig);
        rightMotor2.getConfigurator().apply(rightMotorConfig);
    }

    public Drivetrain() {

    }

    public void drive(double leftSpeed, double rightSpeed) {
        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
        leftSpeed *= DrivetrainConstants.kSpeedScalar;
        rightSpeed *= DrivetrainConstants.kSpeedScalar;
        leftMotor1.set(leftSpeed);
        leftMotor2.set(leftSpeed);
        rightMotor1.set(rightSpeed);
        rightMotor2.set(rightSpeed);
    }

    public void stop() {
        leftMotor1.stopMotor();
        leftMotor2.stopMotor();
        rightMotor1.stopMotor();
        rightMotor2.stopMotor();
    }

    public class TankDrive extends Command {
        private CommandXboxController controller;

        public TankDrive(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Drivetrain.this);
        }

        @Override
        public void execute() {
            Drivetrain.this.drive(controller.getLeftY(), controller.getRightY());
        }

        @Override
        public void end(boolean interrupted) {
            Drivetrain.this.stop();
        }
    }

    public class ArcadeDrive extends Command {
        private CommandXboxController controller;

        public ArcadeDrive(CommandXboxController controller) {
            this.controller = controller;
            addRequirements(Drivetrain.this);
        }

        @Override
        public void execute() {
            double inputX = controller.getLeftX();
            double inputY = controller.getLeftY();
            double forwardDirection = Math.signum(inputY);
            double rawMagnitude = Math.min(1, Math.hypot(inputX, inputY));
            double correctedMagnitude = MathUtil.applyDeadband(rawMagnitude, ArcadeDriveConstants.kInputDeadzone, 1);

            // absolute angles
            double absoluteAngleFromHorizontal = Math.atan2(Math.abs(inputY), Math.abs(inputX));
            double absoluteAngleFromVertical = Math
                    .abs(Units.degreesToRadians(90) - Math.abs(absoluteAngleFromHorizontal));

            double leftDirection = 0, rightDirection = 0;
            if (rawMagnitude < ArcadeDriveConstants.kInputDeadzone) {
                // do nothing
            } else {
                double rotationAlpha;
                if (absoluteAngleFromVertical <= ArcadeDriveConstants.kTranslationBufferAngle) {
                    rotationAlpha = forwardDirection;
                } else if (absoluteAngleFromHorizontal <= ArcadeDriveConstants.kRotationBufferAngle) {
                    rotationAlpha = 0;
                } else {
                    // range of analog motion that is outside of the buffer zones
                    double analogRange = Units.degreesToRadians(90) - (ArcadeDriveConstants.kRotationBufferAngle
                            + ArcadeDriveConstants.kTranslationBufferAngle);
                    double relativeAngle = absoluteAngleFromHorizontal - ArcadeDriveConstants.kRotationBufferAngle;
                    rotationAlpha = MathUtil.clamp(forwardDirection * (relativeAngle / analogRange), -1, 1);
                }

                // rumble for rotation
                double rumbleAlpha = 1 - Math.abs(rotationAlpha);
                double rumbleStrength = (rumbleAlpha == 1) ? RumbleConstants.kArcadeDriveRotateValue
                        : rumbleAlpha * RumbleConstants.kArcadeDriveIntermediateRotateValue;
                HIDRumble.rumble(controller, new RumbleRequest(RumbleType.kRightRumble, rumbleStrength, 1));

                // get directions
                // 90 to 0 degrees: lerp from (-1, -1) to (-1, 1)
                // 0 to -90 degrees: lerp from (-1, 1) to (1, 1)
                leftDirection = -1 + Math.max(0, rotationAlpha * 2);
                leftDirection = MathUtil.clamp(leftDirection, -1, 1);
                rightDirection = -1 + ((rotationAlpha + 1) * 2);
                rightDirection = MathUtil.clamp(rightDirection, -1, 1);

                // reverse inputs if the input is leftward
                if (inputX < 0) {
                    double temp = leftDirection;
                    leftDirection = rightDirection;
                    rightDirection = temp;
                }
            }

            // get speeds based off direction and input magnitude
            double leftSpeed = leftDirection * correctedMagnitude;
            leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
            double rightSpeed = rightDirection * correctedMagnitude;
            rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

            Drivetrain.this.drive(leftSpeed, rightSpeed);
        }

        @Override
        public void end(boolean interrupted) {
            Drivetrain.this.stop();
        }
    }
}
