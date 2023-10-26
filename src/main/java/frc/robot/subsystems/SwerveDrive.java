// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveId, 
    DriveConstants.kFrontLeftTurnId,
    DriveConstants.kFrontLeftAbsoluteEncoderPort, 
    DriveConstants.kFrontLeftAbsoluteEncoderOffset, 
    DriveConstants.kFrontLeftDriveReversed,
    DriveConstants.kFrontLeftTurningReversed, 
    0
  );

  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveId, 
    DriveConstants.kFrontRightTurnId,
    DriveConstants.kFrontRightAbsoluteEncoderPort, 
    DriveConstants.kFrontRightAbsoluteEncoderOffset, 
    DriveConstants.kFrontRightDriveReversed, 
    DriveConstants.kFrontRightTurningReversed, 
    1
  );

  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveId, 
    DriveConstants.kBackLeftTurnId,
    DriveConstants.kBackLeftAbsoluteEncoderPort, 
    DriveConstants.kBackLeftAbsoluteEncoderOffset, 
    DriveConstants.kBackLeftDriveReversed, 
    DriveConstants.kBackLeftTurningReversed, 
    2
  );

  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kBackRightDriveId, 
    DriveConstants.kBackRightTurnId,
    DriveConstants.kBackRightAbsoluteEncoderPort, 
    DriveConstants.kBackRightAbsoluteEncoderOffset, 
    DriveConstants.kBackRightDriveReversed, 
    DriveConstants.kBackRightTurningReversed, 
    3
  );

  private final AHRS m_imu = new AHRS();

  public void resetHeading() {
    m_imu.reset();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxTranslationalMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(m_imu.getAngle(), 360));
  }
  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }
}
