// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVETRAIN_VELOCITY_FACTOR = 0.2;
    }

    public static class ArcadeDriveConstants {

        public static final double TRANSLATION_BUFFER_ANGLE = Units.degreesToRadians(15);
        public static final double ROTATION_BUFFER_ANGLE = Units.degreesToRadians(15);
        public static final double INPUT_DEADZONE = 0.3;
    }

    public static class DrivetrainConstants {

        public static final int LEFT_MOTOR_1_ID = 1;
        public static final int LEFT_MOTOR_2_ID = 4;
        public static final int RIGHT_MOTOR_1_ID = 3;
        public static final int RIGHT_MOTOR_2_ID = 2;

        public static final TalonFXConfiguration DRIVETRAIN_MOTOR_CONFIG = new TalonFXConfiguration();
        public static final TalonFXConfiguration DRIVETRAIN_LEFT_MOTOR_CONFIG, DRIVETRAIN_RIGHT_MOTOR_CONFIG;

        static {
            DRIVETRAIN_MOTOR_CONFIG.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
            DRIVETRAIN_MOTOR_CONFIG.CurrentLimits.withStatorCurrentLimit(80).withSupplyCurrentLimit(40);

            DRIVETRAIN_LEFT_MOTOR_CONFIG = DRIVETRAIN_MOTOR_CONFIG.clone();
            DRIVETRAIN_LEFT_MOTOR_CONFIG.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
            DRIVETRAIN_RIGHT_MOTOR_CONFIG = DRIVETRAIN_MOTOR_CONFIG.clone();
            DRIVETRAIN_RIGHT_MOTOR_CONFIG.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        }
    }

    public static class FeederConstants {

        public static final int LEFT_MOTOR_ID = 1;
        public static final int RIGHT_MOTOR_ID = 2;

        public static enum FeederState {
            IDLE(0),
            INTAKE(0.5),
            OUTTAKE(-0.15);

            public final double dutyCycle;

            private FeederState(double dutyCycle) {
                this.dutyCycle = dutyCycle;
            }
        }
    }

    public static class TurretConstants {

        public static final int TURRET_MOTOR_ID = 9;

        public static final double INPUT_DEADZONE = 0.75;
        public static final double INPUT_ANGLE_SCALAR = 0.5;
        public static final double INPUT_ANGLE_JITTER_BUFFER = Units.degreesToRadians(1);

        public static final double TURRET_MOTOR_GEAR_RATIO = 54.0;

        public static final Angle TURRET_ANGLE_MINIMUM = Degrees.of(-45);
        public static final Angle TURRET_ANGLE_MAXIMUM = Degrees.of(45);

        public static final SparkMaxConfig TURRET_MOTOR_CONFIG = new SparkMaxConfig();

        static {
            TURRET_MOTOR_CONFIG.smartCurrentLimit(40);
            TURRET_MOTOR_CONFIG.secondaryCurrentLimit(50);
            TURRET_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
            TURRET_MOTOR_CONFIG.inverted(true);
        }

        public static final ProfiledPIDController TURRET_PROFILED_PID_CONTROLLER = new ProfiledPIDController(
            0.4,
            0.0,
            0.02,
            new TrapezoidProfile.Constraints(30, 150)
        );
        public static final SimpleMotorFeedforward TURRET_FEED_FORWARD = new SimpleMotorFeedforward(0.05, 0.1, 0);
    }

    public static class ShooterConstants {

        // IDs for all motors
        public static final int LEFT_SHOOTER_MOTOR_ID = 6;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 5;
        public static final int HOOD_MOTOR_ID = 7;

        // pid
        public static final SparkMaxConfig SHOOTER_MOTOR_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig LEFT_SHOOTER_MOTOR_CONFIG, RIGHT_SHOOTER_MOTOR_CONFIG;

        static {
            SHOOTER_MOTOR_CONFIG.smartCurrentLimit(40);
            SHOOTER_MOTOR_CONFIG.secondaryCurrentLimit(80);
            SHOOTER_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
            LEFT_SHOOTER_MOTOR_CONFIG = (SparkMaxConfig) SHOOTER_MOTOR_CONFIG.apply(new SparkMaxConfig()).inverted(
                false
            );
            RIGHT_SHOOTER_MOTOR_CONFIG = (SparkMaxConfig) SHOOTER_MOTOR_CONFIG.apply(new SparkMaxConfig()).inverted(
                true
            );
        }

        public static final ProfiledPIDController SHOOTER_PROFILED_PID_CONTROLLER = new ProfiledPIDController(
            2.5,
            0,
            0,
            new TrapezoidProfile.Constraints(30, 60)
        );

        // used the value for spinTolerance from FRC-2024
        public static final double SPIN_GOAL_TOLERANCE = 5;

        static {
            SHOOTER_PROFILED_PID_CONTROLLER.setTolerance(ShooterConstants.SPIN_GOAL_TOLERANCE);
        }

        // hood angle ranges
        public static final double HOOD_GEAR_RATIO = 25.0;

        public static final SparkMaxConfig HOOD_MOTOR_CONFIG = new SparkMaxConfig();

        static {
            HOOD_MOTOR_CONFIG.smartCurrentLimit(15);
            HOOD_MOTOR_CONFIG.secondaryCurrentLimit(40);
            HOOD_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
            HOOD_MOTOR_CONFIG.inverted(false);
        }

        public static enum HoodState {
            IDLE(0.0),
            UP(0.5),
            DOWN(-0.5),
            DOWN_SLOW(-0.25);

            public final double dutyCycle;

            private HoodState(double dutyCycle) {
                this.dutyCycle = dutyCycle;
            }
        }

        // ----- tolerances: so if the motors/whatnot are a bit off, it'll still
        // work------
        // pitch refers to the angle
        public static final double HOOD_PITCH_TOLERANCE = Units.degreesToRotations(5);

        // ENUMS
        public static enum ShooterState {
            IDLE(RPM.of(0.0)),
            SHOOT(RPM.of(5000.0)),
            REVERSE(RPM.of(-225.0));

            public final AngularVelocity velocity;

            private ShooterState(AngularVelocity velocity) {
                this.velocity = velocity;
            }
        }
    }

    public static class RumbleConstants {

        public static final double TURRET_TURN_STRENGTH = 0.2;
        public static final double TURRET_TRIP_STRENGTH = 0.2;
        public static final double TURRET_ZERO_STRENGTH = 0.5;
        public static final double HOOD_ZERO_STRENGTH = 0.3;
    }
}
