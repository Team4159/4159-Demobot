package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;

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
        private Drivetrain drivetrain;
        private CommandXboxController controller;

        public Drive(Drivetrain drivetrain, CommandXboxController controller) {
            this.drivetrain = drivetrain;
            this.controller = controller;

            addRequirements(drivetrain);
        }

        @Override
        public void execute() {
            drivetrain.drive(controller.getLeftY(), controller.getRightY());
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.stop();
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
            double x = controller.getLeftX();
            double y = controller.getLeftY();
            double direction = Math.signum(y);
            double magnitude = Math.hypot(x, y);
            double angle = Math.atan2(Math.abs(y), Math.abs(x));
            double angleFromVertical = Math.abs(Units.degreesToRadians(90) - Math.abs(angle));
            double angleFromHorizontal = Math.abs(angle);
            double translationThresholdAngle = Units.degreesToRadians(15);
            double rotationThresholdAngle = Units.degreesToRadians(15);

            double left = 0;
            double right = 0;
            if (angleFromVertical <= translationThresholdAngle) {
                left = magnitude * direction;
                right = left;
            } else {
                double rotationAlpha;
                if (angleFromHorizontal <= rotationThresholdAngle) {
                    rotationAlpha = 0;
                } else {
                    double range = Units.degreesToRadians(90) - rotationThresholdAngle - translationThresholdAngle;
                    rotationAlpha = direction * ((angle - rotationThresholdAngle) / range);
                }
                left = Math.min(1, -1 + Math.max(0, rotationAlpha * 2));
                right = Math.max(-1, -1 - Math.min(0, rotationAlpha * 2));
                if (x < 0) {
                    double temp = left;
                    left = right;
                    right = temp;
                }
            }
            Drivetrain.this.drive(left, right);
            System.out.println(left + " " + right);
        }
    }
}
