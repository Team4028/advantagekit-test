// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.util.Units;

public class TurretIOTalonSRX implements TurretIO {
    // Local Constants
    private static final double GEAR_RATIO = 1.5;
    private static final double TICKS_PER_REV = 2048;
    private static final int CAN_ID = 1;

    // Member Variables
    private TalonSRX m_motor;

    public TurretIOTalonSRX() {
        m_motor = new WPI_TalonSRX(CAN_ID);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        m_motor.configAllSettings(config);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(m_motor.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                m_motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO);
        inputs.appliedVolts = m_motor.getMotorOutputVoltage();
        inputs.currentAmps = m_motor.getSupplyCurrent();
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        double velocityFalconUnits = Units.radiansToRotations(velocityRadPerSec) * GEAR_RATIO * TICKS_PER_REV / 10.0;
        m_motor.set(
                ControlMode.Velocity, velocityFalconUnits, DemandType.ArbitraryFeedForward, ffVolts / 12.0);
    }

    /**
     * Sets the motor to a specified percent Vbus
     *
     * @param vbus a decimal from -1.0 to 1.0
     */
    @Override
    public void setVoltage(double vbus) {
        m_motor.set(ControlMode.PercentOutput, vbus);
    }

    /** Stops the motor by setting it to 0% Vbus */
    @Override
    public void stop() {
        m_motor.set(ControlMode.PercentOutput, 0.0);
    }
}
