package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter {
    /*
     * Write subsystem code for the shooter of Demobot, it uses two
     *  Neos as the shooter motors as well as a Neo 550 to control 
     * the adjustable hood. Neo motors use the revlib library.
     */
    // two Neos
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    // one Neo 550
    private SparkMax hoodAdjuster;
    
    public Shooter() {
        leftMotor = new SparkMax(Constants.ShooterConstants.leftShooterControllerPort, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ShooterConstants.leftShooterControllerPort, MotorType.kBrushless);
        hoodAdjuster = new SparkMax(Constants.ShooterConstants.hoodAdjusterControllerPort, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void adjustHood(double angle) {
        hoodAdjuster.getClosedLoopController().setReference(angle, ControlType.kPosition);
    }

    public void stopSpeed() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    // To Do:
    // Commands
    // encapsulate
    // put constants in constants.java
    // replace myShooter --> Shooter.this

    public class ChangeHood extends Command {

        private CommandJoystick myJoystick;
        // put joystick as parameter into command
        // run conversion that converts -1 to 1 range into range for angles
        public ChangeHood(CommandJoystick joystick) {
            myJoystick = joystick;
        }

        public double throttleToAngle() {
            // convert throttle to angle

            double new_value = ( (myJoystick.getThrottle() - Constants.ShooterConstants.throttleMin) / (Constants.ShooterConstants.throttleMax - Constants.ShooterConstants.throttleMin) ) * 
            (Constants.ShooterConstants.hoodAngleMax - Constants.ShooterConstants.hoodAngleMin) + Constants.ShooterConstants.hoodAngleMin;
            return new_value;
        }

        @Override
        public void initialize() {
            Shooter.this.adjustHood(throttleToAngle());
        }

        // execute command (over and over running) call set function to correspond
        @Override
        public void execute() {
            Shooter.this.adjustHood(throttleToAngle());
        }

        @Override
        public void end(boolean i) {
            Shooter.this.stopSpeed();
        }
    }

    
}
