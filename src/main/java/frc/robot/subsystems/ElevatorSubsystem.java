// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// need: have a way for elevator to go up, take in double and goes up
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase {

  public enum Modes {
    PERCENT_CONTROL,
    POSITION_CONTROL,
    ZERO
  }

  private Modes currentMode;

  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private TalonFX wristMotor;

  private double currentExtension;
  private double targetExtension;
  private double currentWristAngle;
  private double targetAngle;
  private double percentControl;

  private ProfiledPIDController extensionController;
  private PIDController wristController;
  private CANCoder canCoder;

  private boolean isInSlowZone;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

  private double filterOutput;
  private double gravityOffset;

  // private DigitalInput proxySensor;

  // add soft limits - check 2022 frc code

  // stator limits
  private LinearFilter filter;

  public static record ElevatorState(double extension, double angle) {}

  public ElevatorSubsystem() {

    currentMode = Modes.ZERO;

    leftMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_LEFT_MOTOR_PORT);
    rightMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_RIGHT_MOTOR_PORT);
    wristMotor = new TalonFX(Constants.Elevator.Ports.WRIST_MOTOR_PORT);

    leftMotor.follow(rightMotor);

    // extensionController = new PIDController(0.1, 0.03, 0.035);
    wristController = new PIDController(0, 0, 0);
    canCoder = new CANCoder(Elevator.Ports.CANCODER);
    // proxySensor = new DigitalInput(0);

    extensionController.setTolerance(0.25, 0.05);

    rightMotor.setSelectedSensorPosition(0);
    leftMotor.setSelectedSensorPosition(0);

    currentExtension = 0.0;
    targetExtension = 0.0;
    currentWristAngle = 0.0;
    targetAngle = 0.0;
    percentControl = 0.0;
    gravityOffset = 0.15;

    isInSlowZone = false;

    rightMotor.configFactoryDefault();
    leftMotor.configFactoryDefault();
    wristMotor.configFactoryDefault();

    rightMotor.clearStickyFaults();
    leftMotor.clearStickyFaults();
    wristMotor.clearStickyFaults();

    rightMotor.setInverted(true);
    leftMotor.setInverted(InvertType.FollowMaster);

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
    wristMotor.configForwardSoftLimitEnable(true);

    rightMotor.configMotionAcceleration(30000, 30);
    rightMotor.configMotionCruiseVelocity(15000, 30);

    rightMotor.config_kP(0, 0.2);
    rightMotor.config_kD(0, 0);
    rightMotor.config_kI(0, 0);
    rightMotor.config_kF(0, 0);

    canCoder.configMagnetOffset(Elevator.ANGULAR_OFFSET);

    canCoder.configSensorDirection(true);

    canCoder.setPositionToAbsolute(10); // ms

    filter = LinearFilter.movingAverage(30);

    tab.addDouble("Wrist Motor Position", () -> canCoder.getAbsolutePosition());
    tab.addDouble("Wrist Target Angle", () -> targetAngle);
    tab.addDouble("Wrist Current Angle", () -> currentWristAngle);
    tab.addDouble("Elevator Target Extension", () -> targetExtension);
    tab.addDouble("Elevator Current Extension", () -> currentExtension);
    tab.addDouble("Elevator Output", rightMotor::getMotorOutputPercent);
    tab.addDouble("velocity", rightMotor::getSelectedSensorVelocity);
    tab.addDouble("filter output", () -> filterOutput);
    tab.addDouble("Stator current", rightMotor::getStatorCurrent);
    // tab.add("PID", extensionController);
    tab.addString("mode", () -> currentMode.toString());
    tab.addBoolean("In slow zone", () -> isInSlowZone);
  }

  public void setMode(Modes mode) {
    currentMode = mode;
  }

  public Modes getMode() {
    return currentMode;
  }

  public static double extensionInchesToTicks(double inches) {
    return (Elevator.FALCON_CPR * inches)
        / (Elevator.CARRIAGE_RATIO
            * (Elevator.ELEVATOR_SPROCKET_DIAMETER_INCHES * Math.PI)
            * Elevator.ELEVATOR_GEAR_RATIO);
  }

  public double ticksToExtensionInches(double ticks) {
    return Elevator.CARRIAGE_RATIO
        * (Elevator.ELEVATOR_SPROCKET_DIAMETER_INCHES * Math.PI)
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

  public void setPercentControl(double percentControl) {
    this.percentControl = percentControl;
  }

  public double getPercentControl() {
    return percentControl;
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

  // private double applySlowZoneToPercent(double percentControl) {
  //   if (getExtensionInches() < 15) {
  //     isInSlowZone = true;
  //     return percentControl * 0.25;
  //   } else {
  //     isInSlowZone = false;
  //     return percentControl;
  //   }
  // }

  private void percentDrivePeriodic() {
    if (currentExtension < 15 && percentControl < 0) {
      rightMotor.set(TalonFXControlMode.PercentOutput, percentControl * 0.4);
      isInSlowZone = true;
    } else {
      rightMotor.set(TalonFXControlMode.PercentOutput, percentControl);
      isInSlowZone = false;
    }

    // rightMotor.set(TalonFXControlMode.MotionMagic, extensionInchesToTicks(targetExtension));

    wristMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(wristController.calculate(currentWristAngle, targetAngle), -0.25, 0.25));
  }

  private void positionDrivePeriodic() {
    // if (filterOutput < statorCurrentLimit) {
    //   double motorPower = extensionController.calculate(currentExtension, targetExtension);
    //   final double gravityOffset = calculateElevatorGravityOffset();

    //   rightMotor.set(
    //       TalonFXControlMode.PercentOutput, MathUtil.clamp(motorPower + gravityOffset, -0.8,
    // 0.8));
    // } else {
    //   rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    // }
    rightMotor.set(TalonFXControlMode.MotionMagic, extensionInchesToTicks(targetExtension));

    wristMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(wristController.calculate(currentWristAngle, targetAngle), -0.25, 0.25));
  }

  private void zeroPeriodic() {
    rightMotor.configForwardSoftLimitEnable(false, 20);
    rightMotor.configReverseSoftLimitEnable(false, 20);

    rightMotor.set(ControlMode.PercentOutput, Elevator.ZERO_MOTOR_POWER);

    if (filterOutput > Elevator.ZERO_STATOR_LIMIT) {
      rightMotor.setSelectedSensorPosition(0);
      leftMotor.setSelectedSensorPosition(0);

      rightMotor.set(ControlMode.PercentOutput, 0);

      currentMode = Modes.POSITION_CONTROL;

      rightMotor.configForwardSoftLimitEnable(true, 20);
      rightMotor.configReverseSoftLimitEnable(true, 20);

      targetExtension = Elevator.MIN_EXTENSION_INCHES;
    }
  }

  @Override
  public void periodic() {
    currentExtension = getExtensionInches();
    currentWristAngle = getCurrentAngleDegrees();
    filterOutput = filter.calculate(rightMotor.getStatorCurrent());

    switch (currentMode) {
      case ZERO:
        zeroPeriodic();
        break;
      case POSITION_CONTROL:
        positionDrivePeriodic();
        break;
      case PERCENT_CONTROL:
        percentDrivePeriodic();
        break;
    }
  }
}
