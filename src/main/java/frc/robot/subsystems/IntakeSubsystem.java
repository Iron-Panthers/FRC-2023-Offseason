// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor;
  private TalonFX wristMotor;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Intake Subsystem");
  private PIDController pidController;
  private double desiredAngle;
  private CANCoder canCoder;
  private IntakeMode currentIntakeMode;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(0);
    wristMotor = new TalonFX(1);
    shuffleboard.addDouble("Intake Motor", () -> intakeMotor.getSelectedSensorPosition());
    shuffleboard.addDouble("Wrist Motor Angle", () -> canCoder.getAbsolutePosition());
    pidController = new PIDController(0, 0, 0);
    canCoder = new CANCoder(0);
    currentIntakeMode = IntakeMode.Off;
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public static record intakeState(IntakeMode mode, double angle) {}

  public enum IntakeMode {
    Intake,
    IntakeOut,
    Hold,
    Off;
  }

  public void intakePeriodic(IntakeMode mode) {

    switch (mode) {
      case Intake:
        intakeMotor.set(TalonFXControlMode.PercentOutput, 1);
      case IntakeOut:
        intakeMotor.set(TalonFXControlMode.PercentOutput, -1);
      case Hold:
      case Off:
      default:
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void setAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public double getWristAngle() {
    return canCoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {

    intakePeriodic(currentIntakeMode);

    wristMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(
            pidController.calculate(canCoder.getAbsolutePosition(), desiredAngle), -0.25, 0.25));
  }
}
