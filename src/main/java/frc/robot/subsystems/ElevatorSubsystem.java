// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// need: have a way for elevator to go up, take in double and goes up
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private TalonFX wristMotor;

  private double currentHeight;
  private double targetHeight;
  private double currentWristAngle;
  private double targetAngle;
  private double statorCurrentLimit;

  private PIDController heightController;
  private PIDController wristController;
  private CANCoder canCoder;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

  private double filterOutput;

  private DigitalInput proxySensor;

  // add soft limits - check 2022 frc code

  // stator limits
  private LinearFilter filter;

  public static record ElevatorState(double height, double angle) {}

  public ElevatorSubsystem() {
    leftMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_LEFT_MOTOR_PORT);
    rightMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_RIGHT_MOTOR_PORT);
    wristMotor = new TalonFX(Constants.Elevator.Ports.WRIST_MOTOR_PORT);

    leftMotor.follow(rightMotor);

    heightController = new PIDController(0.0001, 0, 0);
    wristController = new PIDController(0, 0, 0);
    canCoder = new CANCoder(0);
    proxySensor = new DigitalInput(0);

    currentHeight = 0.0;
    targetHeight = 0.0;
    currentWristAngle = 0.0;
    targetAngle = 0.0;
    statorCurrentLimit = 50.0;

    rightMotor.configFactoryDefault();
    leftMotor.configFactoryDefault();

    rightMotor.clearStickyFaults();
    leftMotor.clearStickyFaults();

    rightMotor.configOpenloopRamp(.5);

    rightMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setNeutralMode(NeutralMode.Brake);

    rightMotor.configForwardSoftLimitThreshold(heightToTicks(Constants.Elevator.MAX_HEIGHT), 20);
    rightMotor.configReverseSoftLimitThreshold(heightToTicks(Elevator.MIN_HEIGHT), 20);

    rightMotor.configForwardSoftLimitEnable(true, 20);
    rightMotor.configReverseSoftLimitEnable(true, 20);

    filter = LinearFilter.movingAverage(30);

    tab.addDouble("Wrist Motor Position", () -> canCoder.getAbsolutePosition());
    tab.addDouble("Wrist Target Angle", () -> targetAngle);
    tab.addDouble("Wrist Current Angle", () -> currentWristAngle);
    tab.addDouble("Elevator Target Height", () -> targetHeight);
    tab.addDouble("Elevator Current Height", () -> currentHeight);
  }

  // FIXME: all the numbers wrong in constants
  public static double heightToTicks(double height) {
    return height
        * ((Elevator.ELEVATOR_GEAR_RATIO * Elevator.ELEVATOR_TICKS)
            / (Elevator.ELEVATOR_GEAR_CIRCUMFERENCE));
  }

  public static double ticksToHeight(double ticks) {
    return (ticks * Elevator.ELEVATOR_GEAR_CIRCUMFERENCE)
        / (Elevator.ELEVATOR_TICKS * Elevator.ELEVATOR_GEAR_RATIO);
  }

  private double getCurrentTicks() {
    return rightMotor.getSelectedSensorPosition();
  }

  public void setTargetHeight(double targetHeight) {
    this.targetHeight = targetHeight;
  }

  public void setTargetState(ElevatorState targetState) {
    targetHeight = targetState.height();
    targetAngle = targetState.angle();
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getHeight() {
    return ticksToHeight(getCurrentTicks());
  }

  public double getCurrentAngleDegrees() {
    return canCoder.getAbsolutePosition();
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  @Override
  public void periodic() {
    currentHeight = getHeight();
    currentWristAngle = getCurrentAngleDegrees();

    if (filter.calculate(rightMotor.getStatorCurrent()) < statorCurrentLimit
        || proxySensor.get() == false) {
      double motorPower = heightController.calculate(currentHeight, targetHeight);
      rightMotor.set(TalonFXControlMode.PercentOutput, motorPower);
    } else {
      rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    wristMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(wristController.calculate(currentWristAngle, targetAngle), -0.25, 0.25));
  }
}
