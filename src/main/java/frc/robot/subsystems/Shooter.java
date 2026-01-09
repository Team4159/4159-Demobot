package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    /*
     * Write subsystem code for the shooter of Demobot, it uses two
     * Neos as the shooter motors as well as a Neo 550 to control
     * the adjustable hood. Neo motors use the revlib library.
     */
    // two Neos
    private final SparkMax leftMotor = new SparkMax(ShooterConstants.kLeftShooterMotorId, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ShooterConstants.kRightShooterMotorId, MotorType.kBrushless);
    // one Neo 550
    private final SparkMax hoodAdjuster = new SparkMax(ShooterConstants.kHoodAdjusterMotorId, MotorType.kBrushless);
    private double hoodAngle = 0;

    private double shootSpeed = 0;

    public Shooter() {
        adjustHood(hoodAngle);
    }

    public void setSpeed(double speed) {
        shootSpeed = speed;
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void adjustHood(double angle) {
        hoodAngle = ShooterConstants.hoodGearRatio * angle + ShooterConstants.hoodAngleOffset;
        hoodAdjuster.getClosedLoopController().setReference(hoodAngle, ControlType.kPosition);
    }

    public void stop() {
        shootSpeed = 0;
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean isShooterReady() {
        if (shootSpeed == 0) {
            return false;
        }
        boolean leftSteady = Math.abs(Math.abs(leftMotor.getAlternateEncoder().getVelocity()) - shootSpeed) <= ShooterConstants.spinTolerance;
        boolean rightSteady = Math.abs(Math.abs(rightMotor.getAlternateEncoder().getVelocity()) - shootSpeed) <= ShooterConstants.spinTolerance;
        return true;
    }

    public class ChangeHood extends Command {
        private double angleDifferentialSpeed;
        private double previousTime;

        // put joystick as parameter into command
        // run conversion that converts -1 to 1 range into range for angles
        public ChangeHood(double angleDifferentialSpeed) {
            this.angleDifferentialSpeed = angleDifferentialSpeed;
            addRequirements(Shooter.this);
        }

        public double throttleToAngle(double deltaTime) {
            // convert throttle to angle
            return MathUtil.clamp(hoodAngle + angleDifferentialSpeed * deltaTime, ShooterConstants.hoodAngleMin, ShooterConstants.hoodAngleMax);
        }

        @Override
        public void initialize() {
            previousTime = MathSharedStore.getTimestamp();
        }

        // execute command (over and over running) call set function to correspond
        @Override
        public void execute() {
            Shooter.this.adjustHood(throttleToAngle(MathSharedStore.getTimestamp() - previousTime));
            previousTime = MathSharedStore.getTimestamp();
        }
    } // end change hood command

    // controls the spin of the shooter
    public class ControlSpin extends Command {
        ShooterState shooterState;

        public ControlSpin(ShooterState shooterState) {
            this.shooterState = shooterState;
        }

        @Override
        public void initialize() {
            Shooter.this.setSpeed(shooterState.speed);
        }

        @Override
        public void end(boolean interupted) {
            Shooter.this.stop();
        }
    }
}
