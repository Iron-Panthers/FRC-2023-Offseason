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
    intakeMotor.setInverted(true);

    filter = LinearFilter.movingAverage(30);

    shuffleboard.addDouble("Intake Motor", () -> intakeMotor.getSelectedSensorPosition());

    shuffleboard.addString("Current Mode", () -> currentIntakeMode.toString());

    shuffleboard.addDouble("stator current", intakeMotor::getStatorCurrent);

    shuffleboard.addDouble("motor power", intakeMotor::getMotorOutputPercent);

    shuffleboard.addDouble("filter output", this::getFilterCalculatedValue);

    // FIXME wrong current limit
    statorCurrentLimit = 85;
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
        break;
      case OUTTAKE:
        intakeMotor.set(TalonFXControlMode.PercentOutput, Intake.OUTTAKE_PERCENT);
        break;
      case HOLD:
        intakeMotor.set(TalonFXControlMode.PercentOutput, Intake.HOLD_PERCENT);
        break;
      case OFF:
      default:
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
        break;
    }
  }

  public Modes getMode() {
    return currentIntakeMode;
  }

  public void setMode(Modes mode) {
    currentIntakeMode = mode;
  }

  private double getFilterCalculatedValue() {
    return filter.calculate(intakeMotor.getStatorCurrent());
  }

  @Override
  public void periodic() {
    if (getFilterCalculatedValue() >= statorCurrentLimit) {
      currentIntakeMode = Modes.HOLD;
    }

    intakePeriodic(currentIntakeMode);
  }
}
