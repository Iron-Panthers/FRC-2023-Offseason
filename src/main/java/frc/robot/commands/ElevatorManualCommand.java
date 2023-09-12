// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorManualCommand extends CommandBase {
  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier rate;

  /** Creates a new AngleArmCommand. */
  public ElevatorManualCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier rate) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.rate = rate;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setTargetHeight(elevatorSubsystem.getTargetHeight() + rate.getAsDouble());
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
