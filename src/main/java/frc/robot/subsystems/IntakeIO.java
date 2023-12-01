package frc.robot.subsystems;

import frc.robot.subsystems.IntakeSubsystem.Modes;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Modes currentIntakeMode = Modes.OFF;
    public double filterOutput = 0.0;
    public boolean isCone = false;
    public double motorOutput = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setMotorPower(double power) {}

  /** Stop in open loop. */
  public default void stop() {}
}
