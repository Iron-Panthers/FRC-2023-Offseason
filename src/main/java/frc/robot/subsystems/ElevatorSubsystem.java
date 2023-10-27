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
// import edu.wpi.first.wpilibj.DigitalInput;
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

  private double currentExtension;
  private double targetExtension;
  private double currentWristAngle;
  private double targetAngle;
  private double statorCurrentLimit;

  private PIDController extensionController;
  private PIDController wristController;
  private CANCoder canCoder;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

  private double filterOutput;

  // private DigitalInput proxySensor;

  // add soft limits - check 2022 frc code

  // stator limits
  private LinearFilter filter;

  public static record ElevatorState(double extension, double angle) {}

  public ElevatorSubsystem() {
    leftMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_LEFT_MOTOR_PORT);
    rightMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_RIGHT_MOTOR_PORT);
    wristMotor = new TalonFX(Constants.Elevator.Ports.WRIST_MOTOR_PORT);

    leftMotor.follow(rightMotor);

    extensionController = new PIDController(0.0001, 0, 0);
    wristController = new PIDController(0, 0, 0);
    canCoder = new CANCoder(0);
    // proxySensor = new DigitalInput(0);

    currentExtension = 0.0;
    targetExtension = 0.0;
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

    rightMotor.configForwardSoftLimitThreshold(
        extensionInchesToTicks(Constants.Elevator.MAX_EXTENSION_INCHES), 20);
    rightMotor.configReverseSoftLimitThreshold(
        extensionInchesToTicks(Elevator.MIN_EXTENSION_INCHES), 20);
    wristMotor.configForwardSoftLimitThreshold(angleToTicks(0));

    rightMotor.configForwardSoftLimitEnable(true, 20);
    rightMotor.configReverseSoftLimitEnable(true, 20);
    wristMotor.configForwardSoftLimitThreshold(angleToTicks(0));

    canCoder.configMagnetOffset(Elevator.ANGULAR_OFFSET);

    canCoder.configSensorDirection(true);

    canCoder.setPositionToAbsolute(10); // ms

    filter = LinearFilter.movingAverage(30);

    tab.addDouble("Wrist Motor Position", () -> canCoder.getAbsolutePosition());
    tab.addDouble("Wrist Target Angle", () -> targetAngle);
    tab.addDouble("Wrist Current Angle", () -> currentWristAngle);
    tab.addDouble("Elevator Target Extension", () -> targetExtension);
    tab.addDouble("Elevator Current Extension", () -> currentExtension);
  }

  public static double extensionInchesToTicks(double inches) {
    return (Elevator.FALCON_CPR * inches)
        / ((Elevator.ELEVATOR_SPROCKET_DIAMETER_INCHES * Math.PI) * Elevator.ELEVATOR_GEAR_RATIO);
  }

  public double ticksToExtensionInches(double ticks) {
    return (Elevator.ELEVATOR_SPROCKET_DIAMETER_INCHES * Math.PI)
        * ((ticks / Elevator.FALCON_CPR) * Elevator.ELEVATOR_GEAR_RATIO);
  }

  private double getCurrentTicks() {
    return rightMotor.getSelectedSensorPosition();
  }

  public void setTargetExtensionInches(double targetExtension) {
    this.targetExtension = targetExtension;
  }

  public void setTargetState(ElevatorState targetState) {
    targetExtension = targetState.extension();
    targetAngle = targetState.angle();
  }

  public double getTargetExtension() {
    return targetExtension;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getExtensionInches() {
    return ticksToExtensionInches(getCurrentTicks());
  }

  public double getCurrentAngleDegrees() {
    return canCoder.getAbsolutePosition();
  }

  private double angleToTicks(double angle) {
    return angle / Elevator.WRIST_TICKS;
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  @Override
  public void periodic() {
    currentExtension = getExtensionInches();
    currentWristAngle = getCurrentAngleDegrees();

    if (filter.calculate(rightMotor.getStatorCurrent()) < statorCurrentLimit) {
      // || proxySensor.get() == false) {
      double motorPower = extensionController.calculate(currentExtension, targetExtension);
      rightMotor.set(TalonFXControlMode.PercentOutput, motorPower);
    } else {
      rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    wristMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(wristController.calculate(currentWristAngle, targetAngle), -0.25, 0.25));
  }
}
