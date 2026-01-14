// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    public static final int kDriverControllerPort = 0;
  }

  public static class ArcadeDriveConstants {
    public static final double kTranslationBufferAngle = Units.degreesToRadians(15);
    public static final double kRotationBufferAngle = Units.degreesToRadians(15);
    public static final double kInputDeadzone = 0.3;
  }

  public static class DrivetrainConstants {
    public static final int kLeftMotor1Id = 1;
    public static final int kLeftMotor2Id = 4;
    public static final int kRightMotor1Id = 3;
    public static final int kRightMotor2Id = 2;

    public static final double kSpeedScalar = 0.2;
  }

  public static class FeederConstants {
    public static final int kLeftMotorId = 1;
    public static final int kRightMotorId = 2;

    public static enum FeederState {
      IDLE(0), INTAKE(0.15), OUTTAKE(-0.15);

      public double speed;
      
      private FeederState(double speed) {
        this.speed = speed;
      }
    }
  }

  public static class TurretConstants {
    public static final int kTurretMotorId = 9;

    public static final double kInputDeadzone = 0.5;
    public static final double kAngleScalar = 1;

    public static final double kTurretMotorGearRatio = 54; // approximated because we don't want to count teeth

    public static final double kTurretAngleMinimum = Units.degreesToRotations(45);
    public static final double kTurretAngleMaximum = Units.degreesToRotations(45);
    public static final SparkMaxConfig kTurretMotorConfig = new SparkMaxConfig();
    {
      kTurretMotorConfig.idleMode(IdleMode.kBrake);
      kTurretMotorConfig.softLimit
          .forwardSoftLimitEnabled(true).forwardSoftLimit(kTurretAngleMaximum)
          .reverseSoftLimitEnabled(true).reverseSoftLimit(kTurretAngleMaximum);
    }

    public static final ProfiledPIDController kTurretProfiledPIDController = new ProfiledPIDController(
        0.4, 0.05, 0.0,
        new TrapezoidProfile.Constraints(50, 80));
    public static final SimpleMotorFeedforward kTurretFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    public static enum TurretState {
      CLOCKWISE(0.1), COUNTERCLOCKWISE(-0.1), IDLE(0);

      public double motorSpeed;

      private TurretState(double speed) {
        motorSpeed = speed;
      }
    }
  }

  public static class ShooterConstants {
    //IDs for all motors
    public static final int kLeftShooterMotorId = 6;
    public static final int kRightShooterMotorId = 5;
    public static final int kHoodAdjusterMotorId = 7;

    // pid
    public static final ProfiledPIDController kShooterProfiledPIDController = new ProfiledPIDController(
        0.01, 0, 0,
        new TrapezoidProfile.Constraints(50, 50)
    );
    {
      kShooterProfiledPIDController.setTolerance(kSpinTolerance);
    }
    
    // hood angle ranges
    public static final double kHoodAngleMinimum = Units.degreesToRotations(0);
    public static final double kHoodAngleMaximum = Units.degreesToRotations(60);
    public static final double kHoodAngleOffset = Units.degreesToRotations(0);
    public static final double kHoodGearRatio = 25.0; // does not affect offset
    public static final double kHoodAdjustSpeed = 0.03;

    //----- tolerances: so if the motors/whatnot are a bit off, it'll still work------
    // pitch refers to the angle
    public static final double kPitchTolerance = Units.degreesToRotations(5);
    // used the value for spinTolerance from FRC-2024 
    public static final double kSpinTolerance = 5;

    // minimum and maximums for each.. because there are so many diff positions the shooter could be in. More
    // efficient to have a range instead of set states
    // REPLACE the values below with actual mins and maxes later (after testing).
    public static final double kMinPitch = Units.degreesToRadians(14);
    public static final double kMaxPitch = Units.rotationsToRadians(0.2);

    // ENUMS 
    public static enum ShooterState {
      OFF(0), SHOOT(0.67), REVERSE(-0.67);

      public double speed;

      private ShooterState(double speed) {
        this.speed = speed;
      }
    }
  }

  public static class RumbleConstants {
    public static final double kArcadeDriveIntermediateRotateValue = 0.1;
    public static final double kArcadeDriveRotateValue = 0.2;
    public static final double kTurretTurnFeedbackValue = 0.2;
  }
}