// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface DrivebaseSubsystemIO {
  @AutoLog
  public static class DrivebaseSubsystemIOInputs {
    public double frontLeftPositionRad = 0.0;
    public double frontLeftVelocityRadPerSec = 0.0;
    public double frontLeftAppliedVolts = 0.0;
    public double[] frontLeftCurrentAmps = new double[] {};

    public double backLeftPositionRad = 0.0;
    public double backLeftVelocityRadPerSec = 0.0;
    public double backLeftAppliedVolts = 0.0;
    public double[] backLeftCurrentAmps = new double[] {};

    public double frontRightPositionRad = 0.0;
    public double frontRightVelocityRadPerSec = 0.0;
    public double frontRightAppliedVolts = 0.0;
    public double[] frontRightCurrentAmps = new double[] {};

    public double backRightPositionRad = 0.0;
    public double backRightVelocityRadPerSec = 0.0;
    public double backRightAppliedVolts = 0.0;
    public double[] backRightCurrentAmps = new double[] {};

    public Rotation2d gyroYaw = new Rotation2d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DrivebaseSubsystemIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(
      double frontLeftVolts, double backLeftVolts, double frontRightVolts, double backRightVolts) {}
}
