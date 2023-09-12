// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class WristManualCommand extends CommandBase {
  private double desiredAngle;
  private double kp;
  private double ki;
  private double kd;
  private PIDController pidController;
  private TalonFX wristMotor;
  private CANCoder canCoder;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Wrist Manual Command");

  public WristManualCommand(ElevatorSubsystem subsystem, double desiredAngle) {
    this.desiredAngle = desiredAngle;
    pidController = new PIDController(0, 0, 0);
    canCoder = new CANCoder(0);
    shuffleboard.addDouble("PID P value", () -> kp);
    shuffleboard.addDouble("PID I value", () -> ki);
    shuffleboard.addDouble("PID D value", () -> kd);
    shuffleboard.addDouble("Current Angle", () -> canCoder.getAbsolutePosition());
    shuffleboard.addDouble("Desired Angle", () -> desiredAngle);
    addRequirements(subsystem);
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(
            pidController.calculate(canCoder.getAbsolutePosition(), desiredAngle), -0.25, 0.25));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
