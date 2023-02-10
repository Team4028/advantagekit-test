package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlywheelIOInputsAutoLogged extends FlywheelIO.FlywheelIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionRad", positionRad);
    table.put("VelocityRadPerSec", velocityRadPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    positionRad = table.getDouble("PositionRad", positionRad);
    velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
    appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
    currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
  }

  public FlywheelIOInputsAutoLogged clone() {
    FlywheelIOInputsAutoLogged copy = new FlywheelIOInputsAutoLogged();
    copy.positionRad = this.positionRad;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps.clone();
    return copy;
  }
}
