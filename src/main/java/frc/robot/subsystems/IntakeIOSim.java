package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  // Note: Intakes have no simulation feature, so this simulation uses the DC motor sim instead. To
  // see other implementations of simulations, go to
  // https://github.dev/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems
  // and look at the simulations that have been implemented
  private DCMotorSim sim = new DCMotorSim(null, 1, 4.69);
}
