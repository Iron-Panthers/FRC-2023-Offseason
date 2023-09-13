// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Intake Subsystem");
  private IntakeMode currentIntakeMode;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(0);
    shuffleboard.addDouble("Intake Motor", () -> intakeMotor.getSelectedSensorPosition());
    currentIntakeMode = IntakeMode.OFF;
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public static record intakeState(IntakeMode mode) {}

  public enum IntakeMode {
    INTAKE,
    OUTTAKE,
    HOLD,
    OFF;
  }

  public void intakePeriodic(IntakeMode mode) {

    switch (mode) {
      case INTAKE:
        intakeMotor.set(TalonFXControlMode.PercentOutput, 1);
      case OUTTAKE:
        intakeMotor.set(TalonFXControlMode.PercentOutput, -1);
      case HOLD:
      case OFF:
      default:
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public IntakeMode getMode() {
    return currentIntakeMode;
  }

  public void setMode(IntakeMode mode) {
    currentIntakeMode = mode;
  }

  @Override
  public void periodic() {

    intakePeriodic(currentIntakeMode);
  }
}
