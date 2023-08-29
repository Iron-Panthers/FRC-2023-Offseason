// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {

  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  private double filterOutput;

  // FIXME: Is that what you need/want??
  public static record ArmState(double angle, double extension) {}

  // stator limits
  private LinearFilter filter;

  public ArmSubsystem() {

    filter = LinearFilter.movingAverage(35);

    // Add debug table
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {}
  }

  @Override
  public void periodic() {}
}
