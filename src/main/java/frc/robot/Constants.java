// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final int leftMotor1ID = 2;
    public static final int leftMotor2ID = 3;
    public static final int rightMotor1ID = 4;
    public static final int rightMotor2ID = 1;

    public static final double kInputScalar = 0.2;
  }

  public static class TurretConstants {
    public static final int kTurretSparkId = 9;

    public static final double kInputDeadzone = 0.5;
    public static final double kInputScalar = 0.5;

    public static final double kTurretMotorGearRatio = 49; // approximated because we don't want to count teeth

    public static final double kTurretForwardLimit = Units.degreesToRotations(30) * kTurretMotorGearRatio;
    public static final double kTurretReverseLimit = Units.degreesToRotations(30) * kTurretMotorGearRatio;
    public static final SparkMaxConfig kTurretMotorConfig = new SparkMaxConfig();
    {
      kTurretMotorConfig.idleMode(IdleMode.kBrake);
      kTurretMotorConfig.softLimit
          .forwardSoftLimitEnabled(true).forwardSoftLimit(kTurretForwardLimit)
          .reverseSoftLimitEnabled(true).reverseSoftLimit(kTurretReverseLimit);
    }

    public static final ProfiledPIDController kTurretProfiledPIDController = new ProfiledPIDController(
        10, 5, 0,
        new TrapezoidProfile.Constraints(80, 40));

    public static enum TurretState {
      CLOCKWISE(0.1), COUNTERCLOCKWISE(-0.1), IDLE(0);

      public double motorSpeed;

      private TurretState(double speed) {
        motorSpeed = speed;
      }
    }
  }

  public static class RumbleConstants {
    public static final double kArcadeDriveIntermediateRotateValue = 0.1;
    public static final double kArcadeDriveRotateValue = 0.2;
    public static final double kTurretTurnFeedbackValue = 0.2;
  }
}