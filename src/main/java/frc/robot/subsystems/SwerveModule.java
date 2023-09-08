// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private class Config {
    public static final double rotationsToMeters = 0.0254 * ((4 * Math.PI) / (6.75)); 
    public static final double RPMToMetersPerSecond = rotationsToMeters / 60.0; 
    public static final double radiansPerRotation = Math.PI * 2; 
  }

  private TalonFX m_turningMotor;
  private CANSparkMax m_drivingMotor;

  private DutyCycleEncoder m_absoluteEncoder;
  private RelativeEncoder m_drivingEncoder;


  private final PIDController m_drivePIDController = new PIDController(0.5, 0, 0);
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.5 / (Math.PI * 2.0),
          0,
          0,
          new TrapezoidProfile.Constraints(
              Drivetrain.Config.kMaxAngularSpeed, 0.5 * Math.PI));


  /** Creates a new SwerveModule. */
  public SwerveModule(int turningID, int drivingID, int encoderID) {
    m_turningMotor = new TalonFX(turningID);
    m_drivingMotor = new CANSparkMax(drivingID, MotorType.kBrushless);

    m_absoluteEncoder = new DutyCycleEncoder(new DigitalInput(encoderID));
    m_drivingEncoder = m_drivingMotor.getEncoder();

    m_drivingEncoder.setVelocityConversionFactor(Config.RPMToMetersPerSecond);
    m_drivingEncoder.setPositionConversionFactor(Config.rotationsToMeters);
    m_absoluteEncoder.setDistancePerRotation(Config.radiansPerRotation);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_drivingEncoder.getVelocity(), new Rotation2d(m_absoluteEncoder.getDistance()) 
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_drivingEncoder.getPosition(), new Rotation2d(m_absoluteEncoder.getDistance())
    );
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = getState();

    desiredState = SwerveModuleState.optimize(desiredState, state.angle);

    double driveOutput = m_drivePIDController.calculate(state.speedMetersPerSecond, desiredState.speedMetersPerSecond);

    double turnOutput = m_turningPIDController.calculate(state.angle.getRadians(), desiredState.angle.getRadians());

    m_drivingMotor.set(driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
  }
}
