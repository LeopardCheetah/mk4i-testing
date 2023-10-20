// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static class DriveConstants {
    public static final double kDriveEncoderPositionToMeters = (1.0 / 6.75) * Units.inchesToMeters(4.0 * Math.PI);
    public static final double kDriveEncoderVelocityToMetersPerSec = kDriveEncoderPositionToMeters / 60.0;
    public static final double kTurnEncoderPositionToRadians = 2.0 * Math.PI;
    public static final double kTranslationalDeadbandMetersPerSecond = 0.001;
    public static final double kMaxTranslationalMetersPerSecond = 14.5;
    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;
    public static final double kWheelBase = Units.inchesToMeters(18.75);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kWheelBase / 2.0),
      new Translation2d(kWheelBase / 2.0, -kWheelBase / 2.0),
      new Translation2d(-kWheelBase / 2.0, kWheelBase / 2.0),
      new Translation2d(-kWheelBase / 2.0, -kWheelBase / 2.0)
    );
    // TODO: gather all of these constants.
    public static int kFrontLeftDriveId = 7;
    public static int kFrontLeftTurnId = 2;
    public static int kFrontLeftAbsoluteEncoderPort = 7;
    public static double kFrontLeftAbsoluteEncoderOffset;
    public static boolean kFrontLeftDriveReversed;

    public static int kFrontRightDriveId = 8;
    public static int kFrontRightTurnId = 1;
    public static int kFrontRightAbsoluteEncoderPort = 6;
    public static double kFrontRightAbsoluteEncoderOffset;
    public static boolean kFrontRightDriveReversed;

    public static int kBackLeftDriveId = 10;
    public static int kBackLeftTurnId = 3;
    public static int kBackLeftAbsoluteEncoderPort = 9;
    public static double kBackLeftAbsoluteEncoderOffset;
    public static boolean kBackLeftDriveReversed;

    public static int kBackRightDriveId = 9;
    public static int kBackRightTurnId = 4;
    public static int kBackRightAbsoluteEncoderPort = 8;
    public static double kBackRightAbsoluteEncoderOffset;
    public static boolean kBackRightDriveReversed;

    public static double kDeadband;
    public static double kTeleopMaxAccelMetersPerSecondSquared;
    public static double kTeleopMaxAngularAccelRadiansPerSecondSquared;
    public static double kTeleopMaxSpeedMetersPerSecond;
    public static int kTeleopMaxTurningRadiansPerSecond;
    public static int kDriveJoystickId;
    public static int kJoystickXAxis;
    public static int kJoystickYxis;
    public static int kJoystickRotAxis;
  }
}
