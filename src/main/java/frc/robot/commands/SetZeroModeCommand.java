// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.Optional;

public class SetZeroModeCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private boolean includeWrist;
  private boolean includeElevator;

  public SetZeroModeCommand(
      ElevatorSubsystem elevatorSubsystem,
      Optional<Boolean> includeWrist,
      Optional<Boolean> includeElevator) {
    this.elevatorSubsystem = elevatorSubsystem;
    if (includeWrist.isPresent()) this.includeWrist = includeWrist.get();
    if (includeElevator.isPresent()) this.includeElevator = includeElevator.get();

    addRequirements(elevatorSubsystem);
  }

  public SetZeroModeCommand(ElevatorSubsystem elevatorSubsystem) {
    this(elevatorSubsystem, Optional.of(true), Optional.of(true));
  }

  public SetZeroModeCommand(ElevatorSubsystem elevatorSubsystem, boolean includeWrist) {
    this(elevatorSubsystem, Optional.of(includeWrist), Optional.of(true));
  }

  // FIXME make it zero elevator + wrist
  @Override
  public void initialize() {}

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
