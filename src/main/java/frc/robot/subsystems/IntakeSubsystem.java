// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private final IntakeIOTalonFX motorIO = new IntakeIOTalonFX();
  private Modes mode;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Intake Subsystem");

  // private final TimeOfFlight coneToF, cubeToF;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;

    shuffleboard.addString("Current mode", () -> mode.toString());
    shuffleboard.addDouble("filter output", () -> motorIO.getFilterOutput());
    shuffleboard.addDouble("motor output", () -> motorIO.getMotorPower());
    shuffleboard.addBoolean("is cube intake", () -> motorIO.getIsCone());
  }

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
    motorIO.setIsCone(isCone);
  }

  public double getFilterOutput() {
    return motorIO.getFilterOutput();
  }

  public void intakePeriodic(Modes modes) {

    switch (modes) {
      case INTAKE:
        if (motorIO.getIsCone()) {
          io.setMotorPower(Intake.INTAKE_CONE_PERCENT);
        } else {
          io.setMotorPower(Intake.INTAKE_CUBE_PERCENT);
        }
        break;
      case OUTTAKE:
        if (motorIO.getIsCone()) {
          io.setMotorPower(Intake.OUTTAKE_CONE_PERCENT);
        } else {
          io.setMotorPower(Intake.OUTTAKE_CUBE_PERCENT);
        }
        break;
      case HOLD:
        if (motorIO.getIsCone()) {
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
    double filterOutput = motorIO.getFilterOutput();
    if (motorIO.getIsCone() && filterOutput >= Intake.CONE_STATOR_LIMIT) {
      mode = Modes.HOLD;
    } else if (filterOutput >= Intake.CUBE_STATOR_LIMIT) {
      mode = Modes.HOLD;
    }

    intakePeriodic(mode);
  }
}
