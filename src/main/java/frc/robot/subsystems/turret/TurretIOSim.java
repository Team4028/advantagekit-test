// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class TurretIOSim implements TurretIO {
  private DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(1), 1.5, 0.004);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (closedLoop) {
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }
}
