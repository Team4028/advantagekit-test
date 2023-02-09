package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DriveIOInputsAutoLogged extends DriveIO.DriveIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeftPositionRad", leftPositionRad);
    table.put("LeftVelocityRadPerSec", leftVelocityRadPerSec);
    table.put("RightPositionRad", rightPositionRad);
    table.put("RightVelocityRadPerSec", rightVelocityRadPerSec);
    table.put("GyroYawRad", gyroYawRad);
  }

  @Override
  public void fromLog(LogTable table) {
    leftPositionRad = table.getDouble("LeftPositionRad", leftPositionRad);
    leftVelocityRadPerSec = table.getDouble("LeftVelocityRadPerSec", leftVelocityRadPerSec);
    rightPositionRad = table.getDouble("RightPositionRad", rightPositionRad);
    rightVelocityRadPerSec = table.getDouble("RightVelocityRadPerSec", rightVelocityRadPerSec);
    gyroYawRad = table.getDouble("GyroYawRad", gyroYawRad);
  }

  public DriveIOInputsAutoLogged clone() {
    DriveIOInputsAutoLogged copy = new DriveIOInputsAutoLogged();
    copy.leftPositionRad = this.leftPositionRad;
    copy.leftVelocityRadPerSec = this.leftVelocityRadPerSec;
    copy.rightPositionRad = this.rightPositionRad;
    copy.rightVelocityRadPerSec = this.rightVelocityRadPerSec;
    copy.gyroYawRad = this.gyroYawRad;
    return copy;
  }
}
