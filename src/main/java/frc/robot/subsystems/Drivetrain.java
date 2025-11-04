package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.Constants.ArcadeDriveConstants;

public class Drivetrain extends SubsystemBase {
    private TalonFX leftMotor1, leftMotor2;
    private TalonFX rightMotor1, rightMotor2;
    private MotorOutputConfigs motorConfig;

    public Drivetrain() {
        leftMotor1 = new TalonFX(Constants.DrivetrainConstants.leftMotor1ID);
        leftMotor2 = new TalonFX(Constants.DrivetrainConstants.leftMotor2ID);
        rightMotor1 = new TalonFX(Constants.DrivetrainConstants.rightMotor1ID);
        rightMotor2 = new TalonFX(Constants.DrivetrainConstants.rightMotor2ID);

        motorConfig = new MotorOutputConfigs();
        motorConfig.Inverted = InvertedValue.Clockwise_Positive;

        leftMotor1.getConfigurator().apply(motorConfig);
        leftMotor2.getConfigurator().apply(motorConfig);
    }

    public void drive(double leftSpeed, double rightSpeed) {
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

    public class Drive extends Command {
        private CommandXboxController controller;

        public Drive(CommandXboxController controller) {
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
            double direction = Math.signum(inputY);
            double rawMagnitude = Math.hypot(inputX, inputY);
            // adjusts range to start at 0 instead of starting at the deadzone
            double correctedMagnitude = Math.max(0,
                    (rawMagnitude - ArcadeDriveConstants.kInputDeadzone) / (1 - ArcadeDriveConstants.kInputDeadzone));

            // absolute angles
            double angleFromHorizontal = Math.atan2(Math.abs(inputY), Math.abs(inputX));
            double angleFromVertical = Math.abs(Units.degreesToRadians(90) - Math.abs(angleFromHorizontal));

            double leftDirection = 0, rightDirection = 0;
            if (rawMagnitude < ArcadeDriveConstants.kInputDeadzone) {
                // do nothing
            } else if (angleFromVertical <= ArcadeDriveConstants.kTranslationBufferAngle) {
                leftDirection = direction;
                rightDirection = direction;
            } else {
                double rotationAlpha;
                if (angleFromHorizontal <= ArcadeDriveConstants.kRotationBufferAngle) {
                    rotationAlpha = 0;
                } else {
                    // range of analog motion that is outside of the buffer zones
                    double angleRange = Units.degreesToRadians(90) - (ArcadeDriveConstants.kRotationBufferAngle
                            + ArcadeDriveConstants.kTranslationBufferAngle);
                    double relativeAngle = angleFromHorizontal - ArcadeDriveConstants.kRotationBufferAngle;
                    rotationAlpha = direction * (relativeAngle / angleRange); // range: [-1, 1]
                }

                // get directions
                // 90 to 0 degrees: lerp from (-1, -1) to (-1, 1)
                // 0 to -90 degrees: lerp from (-1, 1) to (1, 1)
                leftDirection = -1 + Math.max(0, rotationAlpha * 2);
                leftDirection = Math.min(1, Math.max(-1, leftDirection));
                rightDirection = -1 + ((rotationAlpha + 1) * 2);
                rightDirection = Math.min(1, Math.max(-1, rightDirection));

                // reverse inputs if the input is leftward
                if (inputX < 0) {
                    double temp = leftDirection;
                    leftDirection = rightDirection;
                    rightDirection = temp;
                }
            }

            // get speeds based off direction and input magnitude
            double leftSpeed = leftDirection * correctedMagnitude;
            leftSpeed = Math.min(1, Math.max(-1, leftSpeed));
            double rightSpeed = rightDirection * correctedMagnitude;
            rightSpeed = Math.min(1, Math.max(-1, rightSpeed));

            Drivetrain.this.drive(leftSpeed, rightSpeed);
        }
    }
}
