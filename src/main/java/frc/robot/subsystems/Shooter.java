package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter extends SubsystemBase {
    /*
     * Write subsystem code for the shooter of Demobot, it uses two
     * Neos as the shooter motors as well as a Neo 550 to control
     * the adjustable hood. Neo motors use the revlib library.
     */
    // two Neos
    private final SparkMax leftShooterMotor = new SparkMax(ShooterConstants.kLeftShooterMotorId, MotorType.kBrushless);
    private final SparkMax rightShooterMotor = new SparkMax(ShooterConstants.kRightShooterMotorId, MotorType.kBrushless);
    {
        leftShooterMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightShooterMotor.configure(new SparkMaxConfig().inverted(true), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    // one Neo 550
    private final SparkMax hoodAdjuster = new SparkMax(ShooterConstants.kHoodAdjusterMotorId, MotorType.kBrushless);
    {
        hoodAdjuster.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private double hoodAngle = 0;

    public Shooter() {
        adjustHood(hoodAngle);
    }

    @Override
    public void periodic() {
        // motors should be at the same velocity because they are connected to the same axle
        // convert rpm to rps
        double axleVelocity = getAxleVelocity();
        double motorVoltage = ShooterConstants.kShooterProfiledPIDController.calculate(axleVelocity);
        //System.out.println("axlevelocity: " +  axleVelocity + " motorVoltage: " + motorVoltage + " speed: " + ShooterConstants.kShooterPIDController.getGoal().position);
        leftShooterMotor.setVoltage(motorVoltage);
        rightShooterMotor.setVoltage(motorVoltage);
    }

    public void setSpeed(double speed) {
        ShooterConstants.kShooterProfiledPIDController.reset(getAxleVelocity());
        //System.out.println(speed);
        ShooterConstants.kShooterProfiledPIDController.setGoal(speed);
    }

    public void adjustHood(double angle) {
        hoodAngle = ShooterConstants.kHoodGearRatio * angle + ShooterConstants.kHoodAngleOffset;
        //System.out.println(hoodAngle);
        hoodAdjuster.getClosedLoopController().setReference(hoodAngle, ControlType.kPosition);
    }

    public boolean isShooterReady() {
        if (ShooterConstants.kShooterProfiledPIDController.getGoal().velocity <= 0) {
            return false;
        }
        return ShooterConstants.kShooterProfiledPIDController.atGoal();
    }

    private double getAxleVelocity() {
        return leftShooterMotor.getEncoder().getVelocity() / 60.0;
    }

    public class AdjustHood extends Command {
        private double angleDifferentialSpeed;
        private double previousTime;

        // put joystick as parameter into command
        // run conversion that converts -1 to 1 range into range for angles
        public AdjustHood(double angleDifferentialSpeed) {
            this.angleDifferentialSpeed = angleDifferentialSpeed;
            addRequirements(Shooter.this);
        }

        public double throttleToAngle(double deltaTime) {
            // convert throttle to angle
            return MathUtil.clamp(hoodAngle + angleDifferentialSpeed * deltaTime, ShooterConstants.kHoodAngleMinimum, ShooterConstants.kHoodAngleMaximum);
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
        private final ShooterState shooterState;

        public ControlSpin(ShooterState shooterState) {
            this.shooterState = shooterState;
        }

        @Override
        public void initialize() {
            Shooter.this.setSpeed(shooterState.speed);
        }

        @Override
        public void end(boolean interupted) {
            Shooter.this.setSpeed(ShooterState.IDLE.speed);
        }
    }
}
