// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private Modes mode;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Intake Subsystem");

  // private final TimeOfFlight coneToF, cubeToF;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;

    shuffleboard.addString("Current mode", () -> mode.toString());
    shuffleboard.addDouble("filter output", () -> inputs.filterOutput);
    shuffleboard.addDouble("motor output", () -> inputs.motorOutput);
    shuffleboard.addBoolean("is cube intake", () -> inputs.isCone);
  }

  // These modes should probably be in the IntakeIOTalonFX class
  public enum Modes {
    INTAKE,
    OUTTAKE,
    HOLD,
    OFF;
  }

  public Modes getMode() {
    return mode;
  }

  public void setMode(Modes mode) {
    this.mode = mode;
  }

  public void setIsCone(boolean isCone) {
    inputs.isCone = isCone;
  }

  public double getFilterOutput() {
    return inputs.filterOutput;
  }

  public void intakePeriodic(Modes modes) {

    switch (modes) {
      case INTAKE:
        if (inputs.isCone) {
          io.setMotorPower(Intake.INTAKE_CONE_PERCENT);
        } else {
          io.setMotorPower(Intake.INTAKE_CUBE_PERCENT);
        }
        break;
      case OUTTAKE:
        if (inputs.isCone) {
          io.setMotorPower(Intake.OUTTAKE_CONE_PERCENT);
        } else {
          io.setMotorPower(Intake.OUTTAKE_CUBE_PERCENT);
        }
        break;
      case HOLD:
        if (inputs.isCone) {
          io.setMotorPower(Intake.HOLD_CONE_PERCENT);
        } else {
          io.setMotorPower(Intake.HOLD_CUBE_PERCENT);
        }
        break;
      case OFF:
      default:
        io.setMotorPower(0);
        break;
    }
  }

  @Override
  public void periodic() {
    if (inputs.isCone && inputs.filterOutput >= Intake.CONE_STATOR_LIMIT) {
      mode = Modes.HOLD;
    } else if (inputs.filterOutput >= Intake.CUBE_STATOR_LIMIT) {
      mode = Modes.HOLD;
    }

    intakePeriodic(mode);

    io.updateInputs(inputs);

    Logger.getInstance().processInputs("Intake", inputs);
  }
}
