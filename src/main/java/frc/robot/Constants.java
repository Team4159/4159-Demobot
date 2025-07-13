// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // enum for shooter
  public static class ShooterConstants {
    //IDs for all motors
    public static final int leftShooterControllerPort = 5;
    public static final int rightShooterControllerPort = 6;
    public static final int hoodAdjusterControllerPort = 7;

    // hood angle ranges
    public static final double throttleMin = -1;
    public static final double throttleMax = 1;
    public static final double hoodAngleMin = 0;
    public static final double hoodAngleMax = 70;

    //----- tolerances: so if the motors/whatnot are a bit off, it'll still work------
    // pitch refers to the angle
    public static final double pitchTolerance = Units.degreesToRadians(5);
    // used the value for spinTolerance from FRC-2024 
    public static final double spinTolerance = Math.PI/8;

    // minimum and maximums for each.. because there are so many diff positions the shooter could be in. More
    // efficient to have a range instead of set states
    // REPLACE the values below with actual mins and maxes later (after testing).
    public static final double minPitch = Units.degreesToRadians(14);
    public static final double maxPitch = Units.rotationsToRadians(0.2);

  }
}
