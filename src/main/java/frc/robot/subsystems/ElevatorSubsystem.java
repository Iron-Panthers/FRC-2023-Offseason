// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//need: have a way for elevator to go up, take in double and goes up
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Config;
import frc.robot.Constants.Elevator;

/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase {
  
  private TalonFX left_motor;
  private TalonFX right_motor;
  private TalonFX wristMotor;
  private double currentHeight;
  private double targetHeight;
  private double desiredAngle;

  private PIDController heightController;
  private PIDController wristController;
  private CANCoder canCoder;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

  private double filterOutput;

  // // FIXME: Is that what you need/want??
  // public static record ArmState(double angle, double extension) {}

  // stator limits
  private LinearFilter filter;

  public ElevatorSubsystem() {

    heightController = new PIDController(0.0001, 0, 0);

    left_motor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_LEFT_MOTOR_PORT);
    right_motor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_RIGHT_MOTOR_PORT);
    wristMotor = new TalonFX(Constants.Elevator.Ports.WRIST_MOTOR_PORT);
    left_motor.follow(right_motor);

    wristController = new PIDController(0, 0, 0);
    canCoder = new CANCoder(0);

    currentHeight = 0.0;
    targetHeight = 0.0;

    right_motor.configFactoryDefault();
    left_motor.configFactoryDefault();

    right_motor.clearStickyFaults();
    left_motor.clearStickyFaults();

    right_motor.configOpenloopRamp(.5);

    right_motor.setNeutralMode(NeutralMode.Brake);
    left_motor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setNeutralMode(NeutralMode.Brake);

    filter = LinearFilter.movingAverage(35);

    tab.addDouble("Motor Position", () -> canCoder.getAbsolutePosition());
  }

    //FIX ME: all the numbers wrong 
    public static double heightToTicks(double height) {
      return height * ((Elevator.ELEVATOR_GEAR_RATIO * Elevator.ELEVATOR_TICKS) / (Elevator.ELEVATOR_GEAR_CIRCUMFERENCE));
    }
  
    public static double ticksToHeight(double ticks) {
      return (ticks * Elevator.ELEVATOR_GEAR_CIRCUMFERENCE) / (Elevator.ELEVATOR_TICKS * Elevator.ELEVATOR_GEAR_RATIO);
    }

    private double getCurrentTicks(){
      return wristMotor.getSelectedSensorPosition();
    }

    public double getCurrentRotation() {
      return (getCurrentTicks() / Elevator.WRIST_TICKS) * Elevator.WRIST_GEAR_RATIO;
    }

    public double getCurrentAngleDegrees() {
      return getCurrentRotation() * Elevator.WRIST_DEGREES;
    }

    public static double ticksToAngleDegree(double ticks) {
      return (ticks / Elevator.WRIST_TICKS) * Elevator.WRIST_GEAR_RATIO * Elevator.WRIST_DEGREES;
    }

    public void setTargetHeight(double targetHeight) {
      this.targetHeight = targetHeight;
    }

    public void setDesiredAngle(double desiredAngle) {
      this.desiredAngle = desiredAngle;
    }

    public double getTargetHeight(){
      return targetHeight;
    }

    public double getHeight() {
      return ticksToHeight(right_motor.getSelectedSensorPosition());
    }

    public double getAngle() {
      return ticksToAngleDegree(canCoder.getAbsolutePosition());
    }

    // Add debug table
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {}
    
    @Override
    public void periodic() {
      currentHeight = getHeight();
      double motorPower = heightController.calculate(getHeight(), targetHeight);
      right_motor.set(TalonFXControlMode.PercentOutput, motorPower);
      wristMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(
            wristController.calculate(canCoder.getAbsolutePosition(), desiredAngle), -0.25, 0.25));
      }

}
