// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

  public WristManualCommand(ElevatorSubsystem subsystem, double desiredAngle) {
    // FIXME woahh there we cant be putting pid controllers in commands now
    // all this logic belongs in the subsystem!!
    this.desiredAngle = desiredAngle;
    pidController = new PIDController(0, 0, 0);
    canCoder = new CANCoder(0);
    addRequirements(subsystem);
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // FIXME you should set the desired angle in here
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // FIXME this logic is already in the subsystem you don't need it again here!
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
