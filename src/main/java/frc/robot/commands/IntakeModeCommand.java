// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class IntakeModeCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private Modes mode;
  private Optional<BooleanSupplier> isCone;

  /** Creates a new IntakeCommand. */
  public IntakeModeCommand(
      IntakeSubsystem intakeSubsystem, Modes mode, Optional<BooleanSupplier> isCone) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.mode = mode;
    this.isCone = isCone;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    intakeSubsystem.setMode(mode);
    if (isCone.isPresent()) {
      intakeSubsystem.setIsCube(isCone.get().getAsBoolean());
    }
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
    if (intakeSubsystem.getMode() == Modes.HOLD || intakeSubsystem.getMode() == Modes.OFF) {
      return true;
    }
    return false;
  }
}
