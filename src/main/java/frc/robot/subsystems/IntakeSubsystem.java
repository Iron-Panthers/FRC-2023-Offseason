// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX wristMotor;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Intake Subsystem");

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(0);
    wristMotor = new TalonFX(1);
  }

  @Override
  public void periodic() {}
}
