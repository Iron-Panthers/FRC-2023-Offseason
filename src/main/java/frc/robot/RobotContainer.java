// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive;
import frc.robot.commands.AlignGamepieceCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.DriveToPlaceCommand;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.EngageCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.HashMapCommand;
import frc.robot.commands.IntakeModeCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.ScoreStep;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.subsystems.CANWatchdogSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeMode;
import frc.robot.subsystems.NetworkWatchdogSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.util.ControllerUtil;
import frc.util.Layer;
import frc.util.MacUtil;
import frc.util.NodeSelectorUtility;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeSelection;
import frc.util.NodeSelectorUtility.NodeType;
import frc.util.SharedReference;
import frc.util.Util;
import frc.util.pathing.AlliancePose2d;
import frc.util.pathing.RubenManueverGenerator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(visionSubsystem);

  private final RGBSubsystem rgbSubsystem = new RGBSubsystem();

  private final NetworkWatchdogSubsystem networkWatchdogSubsystem =
      new NetworkWatchdogSubsystem(Optional.of(rgbSubsystem));

  private final CANWatchdogSubsystem canWatchdogSubsystem = new CANWatchdogSubsystem(rgbSubsystem);

  private final RubenManueverGenerator manueverGenerator = new RubenManueverGenerator();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final SharedReference<NodeSelection> currentNodeSelection =
      new SharedReference<>(new NodeSelection(NodeSelectorUtility.defaultNodeStack, Height.HIGH));

  /** controller 1 */
  private final CommandXboxController jason = new CommandXboxController(1);
  /** controller 1 layer */
  private final Layer jasonLayer = new Layer(jason.rightBumper());
  /** controller 0 */
  private final CommandXboxController will = new CommandXboxController(0);

  /** the sendable chooser to select which auto to run. */
  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  private GenericEntry autoDelay;

  private final ShuffleboardTab driverView = Shuffleboard.getTab("DriverView");

  /* drive joystick "y" is passed to x because controller is inverted */
  private final DoubleSupplier translationXSupplier =
      () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);
  private final DoubleSupplier translationYSupplier =
      () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivebaseSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivebaseSubsystem,
            translationXSupplier,
            translationYSupplier,
            will.rightBumper(),
            will.leftBumper()));

    // // FIXME: This error is here to kind of guide you...
    // elevatorSubsystem.setDefaultCommand(
    //     new ArmManualCommand(
    //         elevatorSubsystem,
    //         () -> ControllerUtil.deadband(-jason.getLeftY(), 0.2),
    //         () -> ControllerUtil.deadband(jason.getRightY(), 0.2)));

    elevatorSubsystem.setDefaultCommand(
        new ElevatorManualCommand(
            elevatorSubsystem, () -> ControllerUtil.deadband(jason.getLeftY(), 0.2)));

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);
    SmartDashboard.putBoolean("show debug data", Config.SHOW_SHUFFLEBOARD_DEBUG_DATA);
    SmartDashboard.putBoolean("don't init swerve modules", Config.DISABLE_SWERVE_MODULE_INIT);

    // Configure the button bindings
    configureButtonBindings();

    // Create and put autonomous selector to dashboard
    setupAutonomousCommands();
  }

  /**
   * Use this method to do things as the drivers gain control of the robot. We use it to vibrate the
   * driver b controller to notice accidental swaps.
   *
   * <p>Please use this very, very sparingly. It doesn't exist by default for good reason.
   */
  public void containerTeleopInit() {
    // runs when teleop happens
    CommandScheduler.getInstance().schedule(new VibrateHIDCommand(jason.getHID(), 5, .5));
  }

  /**
   * Use this method to do things as soon as the robot starts being used. We use it to stop doing
   * things that could be harmful or undesirable during game play--rebooting the network switch is a
   * good example. Subsystems need to be explicitly wired up to this method.
   *
   * <p>Depending on which mode the robot is enabled in, this will either be called before auto or
   * before teleop, whichever is first.
   *
   * <p>Please use this very, very sparingly. It doesn't exist by default for good reason.
   */
  public void containerMatchStarting() {
    // runs when the match starts
    networkWatchdogSubsystem.matchStarting();
    canWatchdogSubsystem.matchStarting();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    jason
        .y()
        .onTrue(new ElevatorPositionCommand(elevatorSubsystem, Constants.Elevator.MAX_HEIGHT, 0));
    jason
        .a()
        .onTrue(new ElevatorPositionCommand(elevatorSubsystem, Constants.Elevator.MIN_HEIGHT, 20));

    // vibrate jason controller when in layer
    jasonLayer.whenChanged(
        (enabled) -> {
          final double power = enabled ? .1 : 0;
          jason.getHID().setRumble(RumbleType.kLeftRumble, power);
          jason.getHID().setRumble(RumbleType.kRightRumble, power);
        });

    will.start().onTrue(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem));
    will.back()
        .onTrue(new InstantCommand(drivebaseSubsystem::smartZeroGyroscope, drivebaseSubsystem));

    // pov(-1) is the case when no pov is pressed, so doing while false will bind this command to
    // any pov angle
    will.pov(-1).whileFalse(new DefenseModeCommand(drivebaseSubsystem));

    will.leftStick().onTrue(new HaltDriveCommandsCommand(drivebaseSubsystem));
    jason.leftStick().onTrue(new InstantCommand(() -> {}, elevatorSubsystem));

    DoubleSupplier rotation =
        exponential(
            () ->
                ControllerUtil.deadband(
                    (will.getRightTriggerAxis() + -will.getLeftTriggerAxis()), .1),
            2);
    DoubleSupplier rotationVelocity =
        () ->
            rotation.getAsDouble()
                * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                *
                /** percent of fraction power */
                (will.getHID().getAButton() ? .3 : .8);

    new Trigger(() -> Math.abs(rotation.getAsDouble()) > 0)
        .whileTrue(
            new RotateVelocityDriveCommand(
                drivebaseSubsystem,
                translationXSupplier,
                translationYSupplier,
                rotationVelocity,
                will.rightBumper(),
                will.leftBumper()));

    new Trigger(
            () ->
                Util.vectorMagnitude(will.getRightY(), will.getRightX())
                    > Drive.ROTATE_VECTOR_MAGNITUDE)
        .onTrue(
            new RotateVectorDriveCommand(
                drivebaseSubsystem,
                translationXSupplier,
                translationYSupplier,
                will::getRightY,
                will::getRightX,
                will.rightBumper()));

    // start driving to score
    will.b()
        .onTrue(
            new DriveToPlaceCommand(
                    drivebaseSubsystem,
                    manueverGenerator,
                    () -> currentNodeSelection.get().nodeStack().position().get(),
                    translationXSupplier,
                    translationYSupplier,
                    will.rightBumper(),
                    Optional.of(rgbSubsystem),
                    Optional.of(will.getHID()))
                .andThen(
                    new AlignGamepieceCommand(
                        drivebaseSubsystem,
                        manueverGenerator,
                        () -> currentNodeSelection.get().nodeStack().position().get(),
                        translationXSupplier,
                        translationYSupplier,
                        will.rightBumper(),
                        Optional.of(rgbSubsystem),
                        Optional.of(will.getHID()))));

    will.y()
        .onTrue(
            new DriveToPlaceCommand(
                drivebaseSubsystem,
                manueverGenerator,
                (new AlliancePose2d(15.3639 - 1.5, 7.3965, Rotation2d.fromDegrees(0)))::get,
                translationXSupplier,
                translationYSupplier,
                will.rightBumper(),
                Optional.of(rgbSubsystem),
                Optional.of(will.getHID())));

    will.x()
        .onTrue(
            new EngageCommand(
                drivebaseSubsystem, intakeSubsystem, EngageCommand.EngageDirection.GO_BACKWARD));

    // outtake states
    jasonLayer
        .off(jason.leftTrigger())
        .whileTrue(new IntakeModeCommand(intakeSubsystem, IntakeMode.INTAKE));
    jasonLayer
        .off(jason.rightTrigger())
        .onTrue(new IntakeModeCommand(intakeSubsystem, IntakeSubsystem.IntakeMode.OUTTAKE));
    jasonLayer.off(jason.x()).onTrue(new IntakeModeCommand(intakeSubsystem, IntakeMode.OFF));

    // intake presets
    // jasonLayer
    //     .off(jason.a())
    //     .onTrue(new ScoreCommand(outtakeSubsystem, elevatorSubsystem, Setpoints.GROUND_INTAKE))
    //     .whileTrue(
    //         new ForceOuttakeSubsystemModeCommand(outtakeSubsystem,
    // IntakeMode.INTAKE));

    // shelf intake
    jasonLayer
        .off(jason.b())
        // FIXME: This error is here to kind of guide you...
        .onTrue(
            new ElevatorPositionCommand(
                elevatorSubsystem, Constants.Elevator.Setpoints.SHELF_INTAKE));
    //     .whileTrue(
    //         new ForceOuttakeSubsystemModeCommand(
    //             intakeSubsystem, IntakeSubsystem.IntakeMode.INTAKE));
    // FIXME delete or replace outtake stuff with new intake stuff

    // ground pickup
    jasonLayer
        .off(jason.a())
        .onTrue(
            // FIXME make a ground pickup command
            new GroundPickupCommand(
                intakeSubsystem,
                elevatorSubsystem,
                () ->
                    jason.getHID().getPOV() == 180
                        ? IntakeSubsystem.IntakeMode.INTAKE_LOW
                        : IntakeSubsystem.IntakeMode.INTAKE));

    // scoring
    // jasonLayer
    //     .on(jason.a())
    // low

    jasonLayer
        .on(jason.a())
        .onTrue(
            new InstantCommand(
                () ->
                    currentNodeSelection.apply(n -> n.withHeight(NodeSelectorUtility.Height.LOW))));

    jasonLayer
        .on(jason.b())
        .onTrue(
            new InstantCommand(
                () -> currentNodeSelection.apply(n -> n.withHeight(NodeSelectorUtility.Height.MID)),
                elevatorSubsystem));

    jasonLayer
        .on(jason.y())
        .onTrue(
            new InstantCommand(
                () ->
                    currentNodeSelection.apply(n -> n.withHeight(NodeSelectorUtility.Height.HIGH)),
                elevatorSubsystem));

    var scoreCommandMap = new HashMap<NodeSelectorUtility.ScoreTypeIdentifier, Command>();

    for (var scoreType : Constants.SCORE_STEP_MAP.keySet())
      scoreCommandMap.put(
          scoreType,
          new ScoreCommand(
              intakeSubsystem,
              elevatorSubsystem,
              Constants.SCORE_STEP_MAP.get(scoreType),
              jason.leftBumper()));

    jasonLayer
        .on(jason.x())
        .onTrue(
            new HashMapCommand<>(
                scoreCommandMap, () -> currentNodeSelection.get().getScoreTypeIdentifier()));

    jason.povRight().onTrue(new InstantCommand(() -> currentNodeSelection.apply(n -> n.shift(1))));
    jason.povLeft().onTrue(new InstantCommand(() -> currentNodeSelection.apply(n -> n.shift(-1))));

    // control the lights
    currentNodeSelection.subscribe(
        nodeSelection ->
            currentNodeSelection.subscribeOnce(
                rgbSubsystem.showMessage(
                        nodeSelection.nodeStack().type() == NodeSelectorUtility.NodeType.CUBE
                            ? Constants.Lights.Colors.PURPLE
                            : Constants.Lights.Colors.YELLOW,
                        RGBSubsystem.PatternTypes.PULSE,
                        RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR)
                    ::expire));

    // show the current node selection
    driverView
        .addString("Node Selection", () -> currentNodeSelection.get().toString())
        .withPosition(0, 1)
        .withSize(2, 1);
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() {
    if (Config.RUN_PATHPLANNER_SERVER) {
      PathPlannerServer.startServer(5811);
    }

    driverView.addString("NOTES", () -> "...win?").withSize(3, 1).withPosition(0, 0);

    final List<ScoreStep> drivingCubeOuttake =
        List.of(
            new ScoreStep(ElevatorState(35.0, Constants.Elevator.MIN_HEIGHT)).canWaitHere(),
            new ScoreStep(IntakeMode.OUTTAKE));
    final boolean[] intakeLow = {false};
    final Map<String, Command> eventMap =
        Map.of(
            "stow arm",
            "zero everything",
            "intake",
            new CommandBase() {
              private double lastTime = Timer.getFPGATimestamp();

              @Override
              public void initialize() {
                lastTime = Timer.getFPGATimestamp();
                intakeLow[0] = true;
              }

              @Override
              public boolean isFinished() {
                return Timer.getFPGATimestamp() - lastTime > 0.5;
              }

              @Override
              public void end(boolean interrupted) {
                intakeLow[0] = false;
              }
            },
            "stage outtake",
            new ScoreCommand(
                intakeSubsystem, elevatorSubsystem, drivingCubeOuttake.subList(0, 1), 1),
            "stage outtake high",
            new ScoreCommand(
                intakeSubsystem,
                elevatorSubsystem,
                Constants.SCORE_STEP_MAP.get(NodeType.CUBE.atHeight(Height.HIGH)).subList(0, 1)),
            "stage outtake mid",
            new ScoreCommand(
                intakeSubsystem,
                elevatorSubsystem,
                Constants.SCORE_STEP_MAP.get(NodeType.CUBE.atHeight(Height.MID)).subList(0, 1)),

            // FIXME: How can we get this working again?
            "outtake",
            new ScoreCommand(
                    intakeSubsystem, elevatorSubsystem, drivingCubeOuttake.subList(1, 2), 1)
                .andThen(
                    new ElevatorPositionCommand(
                            elevatorSubsystem, Constants.Elevator.Setpoints.STOWED)
                        .andThen(new IntakeModeCommand(intakeSubsystem, IntakeMode.OFF))),
            "armbat preload",
            new ElevatorPositionCommand(elevatorSubsystem, 30, 0)
                .andThen(
                    new ElevatorPositionCommand(
                        elevatorSubsystem, Constants.Elevator.Setpoints.STOWED)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    double delay = autoDelay.getDouble(0);
    return delay == 0
        ? autoSelector.getSelected()
        : new WaitCommand(delay).andThen(autoSelector.getSelected());
  }

  /**
   * applies deadband and squares axis
   *
   * @param value the axis value to be modified
   * @return the modified axis values
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = ControllerUtil.deadband(value, 0.07);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static DoubleSupplier exponential(DoubleSupplier supplier, double exponential) {
    return () -> {
      double val = supplier.getAsDouble();
      return Math.copySign(Math.pow(val, exponential), val);
    };
  }
}
