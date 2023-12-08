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
import frc.robot.subsystems.IntakeIO.IntakeIOInputs;

public class IntakeIOTalonFX extends SubsystemBase {
  private TalonFX intakeMotor;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Intake Subsystem");
  private LinearFilter filter;
  private double filterOutput;
  private boolean isCone;
  // private final TimeOfFlight coneToF, cubeToF;

  /** Creates a new IntakeIOTalonFX. */
  public IntakeIOTalonFX() {

    intakeMotor = new TalonFX(Constants.Intake.Ports.INTAKE_MOTOR_PORT);

    intakeMotor.configFactoryDefault();
    intakeMotor.clearStickyFaults();

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true);

    filter = LinearFilter.movingAverage(30);

    filterOutput = 0.0;

    isCone = false;
  }

  public void setMotorPower(double power) {
    intakeMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  // @AutoLogOutput
  public double getMotorPower() {
    return intakeMotor.getMotorOutputPercent();
  }

  public void setIsCone(boolean isCone) {
    this.isCone = isCone;
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.currentIntakeMode =
        inputs.currentIntakeMode
            .toString(); // The modes are updated in the subsystem instead of here
    inputs.filterOutput = filter.calculate(inputs.filterOutput);
    inputs.isCone = inputs.isCone; // This is also updated in the subsystem
    inputs.motorOutput = getMotorPower();
  }
}
