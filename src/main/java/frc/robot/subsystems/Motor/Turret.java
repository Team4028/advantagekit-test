// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Motor;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SimpleMotorFeedforward ffModel;
  /** Creates a new Turret. */
  
 public Turret(TurretIO io) {
        this.io = io;
        
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
              ffModel = new SimpleMotorFeedforward(0.1, 0.05);
             // io.configurePID(1.0, 0.0, 0.0);
              break;
            case SIM:
              ffModel = new SimpleMotorFeedforward(0.1, 0.05);
              //io.configurePID(0.5, 0.0, 0.0);
              break;
            default:
              ffModel = new SimpleMotorFeedforward(0.0, 0.0);
              break;
          }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Turret", inputs);

    // Log flywheel speed in RPM
    Logger.getInstance().recordOutput("TurretSpeedRPM", getVelocityRPM());
  }

  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.getInstance().recordOutput("TurretSetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}

    


