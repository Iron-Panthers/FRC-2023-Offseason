// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickupCommand extends SequentialCommandGroup {
  /** Creates a new GroundPickupCommand. */
  private BooleanSupplier isCube;

  private ElevatorSubsystem elevatorSubsystem;

  public GroundPickupCommand(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BooleanSupplier isCube) {
    // Add your commands in the addCommands() call, e.g.
    this.isCube = isCube;
    this.elevatorSubsystem = elevatorSubsystem;

    addCommands(
        determineIntakeType()
            .alongWith(new IntakeModeCommand(intakeSubsystem, Modes.INTAKE, isCube))
            .andThen(new ElevatorPositionCommand(elevatorSubsystem, Elevator.Setpoints.STOWED)));
  }

  private Command determineIntakeType() {
    if (isCube.getAsBoolean()) {
      return new ElevatorPositionCommand(elevatorSubsystem, Elevator.Setpoints.GROUND_INTAKE_CUBE);
    } else {
      return new ElevatorPositionCommand(elevatorSubsystem, Elevator.Setpoints.GROUND_INTAKE_CONE);
    }
  }
}
