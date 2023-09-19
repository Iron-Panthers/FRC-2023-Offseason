// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends CommandBase {
  private final ElevatorSubsystem ElevatorSubsystem;
  private final double targetHeight;
  private final double desiredAngle; // FIXME rename to "targetAngle"

  /** Creates a new ArmPositionCommand. */
  public ElevatorPositionCommand(
      ElevatorSubsystem elevatorSubsystem, double targetHeight, double desiredAngle) {
    // FIXME make a second constructor that requires an ElevatorState instead of two doubles for
    // height and angle
    this.ElevatorSubsystem = elevatorSubsystem;
    this.targetHeight = targetHeight;
    this.desiredAngle = desiredAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSubsystem.setTargetHeight(targetHeight);
    ElevatorSubsystem.setDesiredAngle(desiredAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
