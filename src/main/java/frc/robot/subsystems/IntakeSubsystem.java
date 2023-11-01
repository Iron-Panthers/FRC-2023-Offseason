// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Intake Subsystem");
  private Modes currentIntakeMode;
  private LinearFilter filter;
  private double statorCurrentLimit;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.Intake.Ports.INTAKE_MOTOR_PORT);

    currentIntakeMode = Modes.OFF;

    intakeMotor.setNeutralMode(NeutralMode.Brake);

    filter = LinearFilter.movingAverage(30);

    shuffleboard.addDouble(
        "Intake motor sensor position", () -> intakeMotor.getSelectedSensorPosition());
    shuffleboard.addString("Current mode", () -> currentIntakeMode.toString());
  }

  public enum Modes {
    INTAKE,
    OUTTAKE,
    HOLD,
    OFF;
  }

  public void intakePeriodic(Modes mode) {

    switch (mode) {
      case INTAKE:
        intakeMotor.set(TalonFXControlMode.PercentOutput, Intake.INTAKE_PERCENT);
      case OUTTAKE:
        intakeMotor.set(TalonFXControlMode.PercentOutput, Intake.OUTTAKE_PERCENT);
      case HOLD:
        intakeMotor.set(TalonFXControlMode.PercentOutput, Intake.HOLD_PERCENT);
      case OFF:
      default:
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public Modes getMode() {
    return currentIntakeMode;
  }

  public void setMode(Modes mode) {
    currentIntakeMode = mode;
  }

  @Override
  public void periodic() {
    if (filter.calculate(intakeMotor.getStatorCurrent()) >= statorCurrentLimit) {
      currentIntakeMode = Modes.HOLD;
    }

    intakePeriodic(currentIntakeMode);
  }
}
