package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {

    public double currentExtension = 0.0;
    public double targetExtension = 0.0;
    public double currentWristAngle = 0.0;
    public double targetAngle = 0.0;
    public double elevatorPercentControl = 0.0;
    public double wristPercentControl = 0.0;
    private double elevatorFilterOutput = 0.0;
    private double wristFilterOutput = 0.0;

    public boolean elevatorZeroed = false;
    public boolean wristZeroed = false;

    public enum Modes {
      PERCENT_CONTROL,
      POSITION_CONTROL,
      ZERO
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
