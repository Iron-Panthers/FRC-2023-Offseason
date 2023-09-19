// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class WristManualCommand extends CommandBase {
  private double targetAngle;
  private ElevatorSubsystem subsystem;

  public WristManualCommand(ElevatorSubsystem subsystem, double targetAngle) {
    this.targetAngle = targetAngle;
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  public void settargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setTargetAngle(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setTargetAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
