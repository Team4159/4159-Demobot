package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.SpinState;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Shooter extends SubsystemBase{
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
        rightMotor = new SparkMax(Constants.ShooterConstants.rightShooterControllerPort, MotorType.kBrushless);
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

    public class ChangeHood extends Command {

        private CommandJoystick myJoystick;
        // put joystick as parameter into command
        // run conversion that converts -1 to 1 range into range for angles
        public ChangeHood(CommandJoystick joystick, Shooter shooter) {
            myJoystick = joystick;
            addRequirements(Shooter.this);
        }

        public double throttleToAngle() {
            // convert throttle to angle
            double new_value = ( (myJoystick.getThrottle() - Constants.ShooterConstants.throttleMin) / (Constants.ShooterConstants.throttleMax - Constants.ShooterConstants.throttleMin) ) * 
            (Constants.ShooterConstants.hoodAngleMax - Constants.ShooterConstants.hoodAngleMin) + Constants.ShooterConstants.hoodAngleMin;
            return new_value;
        }


        public void setShooterState(double state) {
            leftMotor.set(state);
            rightMotor.set(state);
        }

        // execute command (over and over running) call set function to correspond
        @Override
        public void execute() {
            Shooter.this.adjustHood(throttleToAngle());
        }
    } // end change hood command

    // controls the spin of the shooter
    public class ControlSpin extends Command {
        SpinState mySpinState;
        public ControlSpin(SpinState spinState) {
            mySpinState = spinState;
        }
        

        @Override
        public void execute() {
            Shooter.this.setSpeed(mySpinState.getSpeed());
        }
    }

    
}
