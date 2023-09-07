// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private class Config {
    public static final double RPMToMetersPerSecond = 1.0; // TODO: calculate for driving motor
    public static final double distancePerRotation = 1.0; // TODO: calculate distance per rotation
    public static final double rotationsToMeters = 1.0; // TODO: calculate rotations to meters
  }

  private TalonFX m_turningMotor;
  private CANSparkMax m_drivingMotor;

  private DutyCycleEncoder m_absoluteEncoder;
  private RelativeEncoder m_drivingEncoder;


  /** Creates a new SwerveModule. */
  public SwerveModule(int turningID, int drivingID, int encoderID) {
    m_turningMotor = new TalonFX(turningID);
    m_drivingMotor = new CANSparkMax(drivingID, MotorType.kBrushless);

    m_absoluteEncoder = new DutyCycleEncoder(new DigitalInput(encoderID));
    m_drivingEncoder = m_drivingMotor.getEncoder();

    m_drivingEncoder.setVelocityConversionFactor(Config.RPMToMetersPerSecond);
    m_absoluteEncoder.setDistancePerRotation(Config.distancePerRotation);
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

  public void setDesiredState(SwerveModuleState state) {
    // TODO: this entire method
    return;
  }
}
