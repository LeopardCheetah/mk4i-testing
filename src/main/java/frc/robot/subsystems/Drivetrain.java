// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public class Config {
    public static final double kMaxSpeed = 1.0; 
    public static final double kMaxAngularSpeed = Math.PI / 2.0;
  }

  private final Translation2d m_frontLeftLocation = new Translation2d(0.0, 0.0);
  private final Translation2d m_frontRightLocation = new Translation2d(0.0, 0.0);
  private final Translation2d m_backLeftLocation = new Translation2d(0.0, 0.0);
  private final Translation2d m_backRightLocation = new Translation2d(0.0, 0.0);

  private final SwerveModule m_frontLeft = new SwerveModule(3, 10, 9);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  public void drive(double xSpeed, double ySpeed, double rot) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Config.kMaxSpeed);

    m_frontLeft.setDesiredState(states[0]);
  }

}
