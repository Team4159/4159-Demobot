package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.ctre.phoenix6.hardware.TalonFX;

public class Drivetrain extends SubsystemBase {
    private TalonFX leftMotor1, leftMotor2;
    private TalonFX rightMotor1, rightMotor2;

    public Drivetrain() {
        leftMotor1 = new TalonFX(1);
        leftMotor2 = new TalonFX(2);
        rightMotor1 = new TalonFX(3);
        rightMotor2 = new TalonFX(4);

        //change to configs soon
        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);
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
        private CommandJoystick leftStick, rightStick;

        public Drive(Drivetrain drivetrain, CommandJoystick leftStick, CommandJoystick rightStick) {
            this.drivetrain = drivetrain;
            this.leftStick = leftStick;
            this.rightStick = rightStick;

            addRequirements(drivetrain);
        } 

        @Override
        public void execute() {
            drivetrain.drive(leftStick.getY(), rightStick.getY());
        }
        
        @Override
        public void end(boolean interrupted) {
            drivetrain.stop();
        }
    }
}
