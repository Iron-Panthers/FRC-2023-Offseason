package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  // Note: Intakes have no simulation feature, so this simulation class is only here as a
  // placeholder. To see an actual simulation, go to
  // https://github.dev/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems and look at the simulations that have been implemented
  private FlywheelSim sim = new FlywheelSim(null, 0, 0);
}
