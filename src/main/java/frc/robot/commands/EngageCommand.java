// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive.AutoBalance;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.Optional;

public class EngageCommand extends CommandBase {
  enum Mode {
    DOCKING,
    ENGAGING,
    DEFENSE
  }

  private DrivebaseSubsystem drivebaseSubsystem;
  private Optional<IntakeSubsystem> intakeSubsystem;
  private EngageDirection engageDirection;
  private boolean exceededDockingThreshold = false;
  private Mode currentMode = Mode.DOCKING;
  private LinearFilter filter = LinearFilter.movingAverage(20);

  // static List<SmartBoard> boards;
  static boolean inited = false;

  public enum EngageDirection {
    GO_FORWARD(1),
    GO_BACKWARD(-1);

    private int driveSign;

    EngageDirection(int driveSign) {
      this.driveSign = driveSign;
    }
  }
  /** Creates a new EngageCommand. */
  public EngageCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      Optional<IntakeSubsystem> intakeSubsystem,
      EngageDirection engageDirection) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.engageDirection = engageDirection;
    addRequirements(drivebaseSubsystem);
    intakeSubsystem.ifPresent(this::addRequirements);
  }

  public EngageCommand(DrivebaseSubsystem drivebaseSubsystem, EngageDirection engageDirection) {
    this(drivebaseSubsystem, Optional.empty(), engageDirection);
  }

  public EngageCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem,
      EngageDirection engageDirection) {
    this(drivebaseSubsystem, Optional.of(intakeSubsystem), engageDirection);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    exceededDockingThreshold = false;
    currentMode = Mode.DOCKING;
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA && !inited) {
      inited = true;
      ShuffleboardTab tab = Shuffleboard.getTab("EngageCommand");
      tab.addString("Mode", () -> currentMode.toString());
      tab.addDouble("Angle", () -> this.drivebaseSubsystem.getRollPitch().roll());
      // boards =
      //     List.of(
      //         new SmartBoard(
      //             tab,
      //             "dock speed ms",
      //             () -> AutoBalance.DOCK_SPEED_METERS_PER_SECOND,
      //             m -> AutoBalance.DOCK_SPEED_METERS_PER_SECOND = m,
      //             new Range(0, 1.5)),
      //         new SmartBoard(
      //             tab,
      //             "dock min angle",
      //             () -> AutoBalance.DOCK_MIN_ANGLE_DEGREES,
      //             d -> AutoBalance.DOCK_MIN_ANGLE_DEGREES = d,
      //             new Range(0, 20)),
      //         new SmartBoard(
      //             tab,
      //             "dock horizon angle",
      //             () -> AutoBalance.DOCK_HORIZON_ANGLE_DEGREES,
      //             d -> AutoBalance.DOCK_HORIZON_ANGLE_DEGREES = d,
      //             new Range(0, 20)),
      //         new SmartBoard(
      //             tab,
      //             "engage speed ms",
      //             () -> AutoBalance.ENGAGE_SPEED_METERS_PER_SECOND,
      //             m -> AutoBalance.ENGAGE_SPEED_METERS_PER_SECOND = m,
      //             new Range(0, .7)),
      //         new SmartBoard(
      //             tab,
      //             "engage min angle",
      //             () -> AutoBalance.ENGAGE_MIN_ANGLE_DEGREES,
      //             d -> AutoBalance.ENGAGE_MIN_ANGLE_DEGREES = d,
      //             new Range(0, 15)));
    }
  }

  private Mode advanceState() {
    var rollPitch = drivebaseSubsystem.getRollPitch();
    var filterAbsRoll = filter.calculate(rollPitch.absRoll());

    switch (currentMode) {
      case DOCKING -> {
        if (filterAbsRoll > AutoBalance.DOCK_HORIZON_ANGLE_DEGREES) {
          exceededDockingThreshold = true;
        }
        intakeSubsystem.ifPresent(
            intake ->
                intake.setMode(
                    engageDirection == EngageDirection.GO_BACKWARD
                        ? IntakeSubsystem.Modes.CLIMB
                        : IntakeSubsystem.Modes.STOWED));
        drivebaseSubsystem.drive(
            new ChassisSpeeds(
                AutoBalance.DOCK_SPEED_METERS_PER_SECOND * engageDirection.driveSign, 0, 0));
        return (exceededDockingThreshold && filterAbsRoll < AutoBalance.DOCK_MIN_ANGLE_DEGREES)
            ? Mode.ENGAGING
            : Mode.DOCKING;
      }
      case ENGAGING -> {
        intakeSubsystem.ifPresent(intake -> intake.setMode(IntakeSubsystem.Modes.STOWED));
        drivebaseSubsystem.drive(
            new ChassisSpeeds(
                Math.copySign(AutoBalance.ENGAGE_SPEED_METERS_PER_SECOND, rollPitch.roll()), 0, 0));
        return (rollPitch.absRoll() < AutoBalance.ENGAGE_MIN_ANGLE_DEGREES)
            ? Mode.DEFENSE
            : Mode.ENGAGING;
      }

      case DEFENSE -> {
        intakeSubsystem.ifPresent(intake -> intake.setMode(IntakeSubsystem.Modes.STOWED));
        drivebaseSubsystem.setDefenseMode();
        return rollPitch.absRoll() > AutoBalance.ENGAGE_MIN_ANGLE_DEGREES
            ? Mode.ENGAGING
            : Mode.DEFENSE;
      }
      default -> {
        System.err.println("Engage command unknown mode: " + currentMode);
        throw new IllegalStateException();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentMode = advanceState();
    // if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
    //   boards.forEach(SmartBoard::poll);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.setDefenseMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
