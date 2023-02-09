// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Motor;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretIOTalonSRX implements TurretIO {
    private static final double GEAR_RATIO = 1.5;
  private static final double TICKS_PER_REV = 2048;

  private final TalonSRX m_turret;



 
  public TurretIOTalonSRX() {
    m_turret = new WPI_TalonSRX(1);

    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_turret.configAllSettings(config);

  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(
        m_turret.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        m_turret.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO);
    inputs.appliedVolts = m_turret.getMotorOutputVoltage();
    inputs.currentAmps = new double[] { m_turret.getSupplyCurrent() };
  }
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    double velocityFalconUnits = Units.radiansToRotations(velocityRadPerSec)
        * GEAR_RATIO * TICKS_PER_REV / 10.0;
    m_turret.set(ControlMode.Velocity, velocityFalconUnits,
        DemandType.ArbitraryFeedForward, ffVolts / 12.0);
  }

  @Override
  public void stop() {
    m_turret.set(ControlMode.PercentOutput, 0.0);
  }

 
}
