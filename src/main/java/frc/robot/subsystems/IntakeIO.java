package frc.robot.subsystems;

import frc.robot.subsystems.IntakeSubsystem.Modes;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs implements LoggableInputs {
    public String currentIntakeMode = Modes.OFF.toString();
    public double filterOutput = 0.0;
    public boolean isCone = false;
    public double motorOutput = 0.0;

    public void toLog(LogTable table) {
      table.put("IntakeMode", currentIntakeMode);
      table.put("FilterOutput", filterOutput);
      table.put("IsCone", isCone);
      table.put("MotorOutput", motorOutput);
    }

    public void fromLog(LogTable table) {
      currentIntakeMode = table.getString("IntakeMode", currentIntakeMode);
      filterOutput = table.getDouble("FilterOutput", filterOutput);
      motorOutput = table.getDouble("MotorOutput", motorOutput);
      isCone = table.getBoolean("IsCone", isCone);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setMotorPower(double power) {}

  public default void setIsCone(boolean isCone) {}

  /** Stop in open loop. */
  public default void stop() {}
}
