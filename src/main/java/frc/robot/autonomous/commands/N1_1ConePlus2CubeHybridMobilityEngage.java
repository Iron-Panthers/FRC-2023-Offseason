// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EngageCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.Modes;
import frc.util.pathing.LoadMirrorPath;
import java.util.Map;
import java.util.function.Supplier;

public class N1_1ConePlus2CubeHybridMobilityEngage extends SequentialCommandGroup {
  public N1_1ConePlus2CubeHybridMobilityEngage(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      Map<String, Command> eventMap,
      IntakeSubsystem intakeSubsystem,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    Supplier<PathPlannerTrajectory> path =
        LoadMirrorPath.loadPath(
            "n1 3cube hybrid", maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    addCommands(
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(path, true, drivebaseSubsystem),
            path.get().getMarkers(),
            eventMap),
        new EngageCommand(
                drivebaseSubsystem, intakeSubsystem, EngageCommand.EngageDirection.GO_BACKWARD)
            .alongWith(new SetOuttakeModeCommand(outtakeSubsystem, Modes.OFF))
            .alongWith(new SetZeroModeCommand(armSubsystem)));
  }
}
