// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;

public class LogSwerveState extends CommandBase {
  private String m_name;
  private SwerveModule m_module;
  /** Creates a new LogSwerveState. */
  public LogSwerveState(String name, SwerveModule module) {
    m_name = name;
    m_module = module;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState state = m_module.getState();
    SwerveModulePosition pos = m_module.getPosition();

    SmartDashboard.putNumber(m_name + "_Angle_deg", state.angle.getDegrees());
    SmartDashboard.putNumber(m_name + "_Velocity_m/s", state.speedMetersPerSecond);
    SmartDashboard.putNumber(m_name + "_Distance_m", pos.distanceMeters);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
