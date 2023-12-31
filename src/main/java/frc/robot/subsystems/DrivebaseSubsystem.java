// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.playingwithfusion.TimeOfFlight.Status;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Config;
import frc.robot.Constants.PoseEstimator;
import frc.robot.subsystems.VisionSubsystem.VisionMeasurement;
import frc.util.AdvancedSwerveTrajectoryFollower;
import frc.util.Util;
import java.util.Optional;

public class DrivebaseSubsystem extends SubsystemBase {
  private final AdvancedSwerveTrajectoryFollower follower =
      new AdvancedSwerveTrajectoryFollower(
          new PIDController(1.3853, 0, 0),
          new PIDController(1.3853, 0, 0),
          new ProfiledPIDController(
              // FIXME: RETUNE WITH CARPET NOT LINOLEUM
              .147,
              0,
              0.005,
              new TrapezoidProfile.Constraints(
                  MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                  0.5 * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

  // NOTE: I'm still not sure what's the way to profile this ^
  // Not mission critical as it "technically" drives fine as of now; but I suspect this is a site
  // for future improvements

  public AdvancedSwerveTrajectoryFollower getFollower() {
    return follower;
  }

  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  /**
   * The kinematics object allows us to encode our relationship between desired speeds (represented
   * by a ChassisSpeeds object) and our actual drive outputs (what speeds and angles we apply to
   * each module)
   */
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          // Front right
          new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0),
          // Front left
          new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0));

  /** The SwerveDriveOdometry class allows us to estimate the robot's "pose" over time. */
  private final SwerveDrivePoseEstimator swervePoseEstimator;

  private final VisionSubsystem visionSubsystem;

  /**
   * Keeps track of the current estimated pose (x,y,theta) of the robot, as estimated by odometry.
   */
  private Pose2d robotPose = new Pose2d();

  /** The current ChassisSpeeds goal for the drivetrain */
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); // defaults to zeros

  /** The modes of the drivebase subsystem */
  public enum Modes {
    DRIVE,
    DRIVE_ANGLE,
    DEFENSE
  }

  /** The current mode */
  private Modes mode = Modes.DRIVE;

  private Field2d field = new Field2d();

  /** Contains each swerve module. Order: FR, FL, BL, BR. Or in Quadrants: I, II, III, IV */
  private final SwerveModule[] swerveModules;

  private final PIDController rotController;

  private double targetAngle = 0; // default target angle to zero

  private final TimeOfFlight zappyThing;

  private Pair<Double, Double> xyInput = new Pair<>(0d, 0d); // the x and y for using target angles
  /**
   * The Shuffleboard tab which all things related to the drivebase can be put for easy access and
   * organization
   */
  private final ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

  /**
   * Initialize a falcon with a shuffleboard tab, and mk4 default gear ratio
   *
   * @param tab the shuffleboard tab to use
   * @param title the shuffleboard title
   * @param pos the shuffleboard x position, which is <b>multiplied by 2</b>
   * @param drive the drive motor port const
   * @param steer the steer motor port const
   * @param encoder the encoder port const
   * @param offset the steer offset const, found experimentally
   * @return SDS/swerve-lib SwerveModule object
   */
  private SwerveModule createModule(
      String title, int pos, int drive, int steer, int encoder, double offset) {
    // NOTE: our team uses the MK4 configuration with L2 gearing and Falcon 500s
    // if this changes, update the helper/method/GearRatio used, as needed.
    return Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout(title, BuiltInLayouts.kList).withSize(2, 4).withPosition(pos * 2, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        drive,
        steer,
        encoder,
        offset);
  }

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;

    if (!Config.DISABLE_SWERVE_MODULE_INIT) {

      final SwerveModule module1 =
          createModule(
              "Module #1",
              0,
              Modules.Module1.DRIVE_MOTOR,
              Modules.Module1.STEER_MOTOR,
              Modules.Module1.STEER_ENCODER,
              Modules.Module1.STEER_OFFSET);

      final SwerveModule module2 =
          createModule(
              "Module #2",
              1,
              Modules.Module2.DRIVE_MOTOR,
              Modules.Module2.STEER_MOTOR,
              Modules.Module2.STEER_ENCODER,
              Modules.Module2.STEER_OFFSET);

      final SwerveModule module3 =
          createModule(
              "Module #3",
              2,
              Modules.Module3.DRIVE_MOTOR,
              Modules.Module3.STEER_MOTOR,
              Modules.Module3.STEER_ENCODER,
              Modules.Module3.STEER_OFFSET);

      final SwerveModule module4 =
          createModule(
              "Module #4",
              3,
              Modules.Module4.DRIVE_MOTOR,
              Modules.Module4.STEER_MOTOR,
              Modules.Module4.STEER_ENCODER,
              Modules.Module4.STEER_OFFSET);

      swerveModules = // modules are always initialized and passed in this order
          new SwerveModule[] {module1, module2, module3, module4};
    } else {
      swerveModules = null;
    }

    rotController = new PIDController(0.03, 0.001, 0.003);
    rotController.setSetpoint(0);
    rotController.setTolerance(ANGULAR_ERROR); // degrees error
    // tune pid with:
    // tab.add(rotController);

    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getConsistentGyroscopeRotation(),
            getSwerveModulePositions(),
            // FIXME: FIXME FIXME GOOD GOD FIX ME, USE A REAL VALUE HERE
            new Pose2d(3.5, 2.2, Rotation2d.fromDegrees(0)),
            PoseEstimator.STATE_STANDARD_DEVIATIONS,
            PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS);

    zeroGyroscope();

    zappyThing = new TimeOfFlight(Constants.Drive.ZAPPY_THING_PORT);

    // tab.addNumber("target angle", () -> targetAngle);
    // tab.addNumber("current angle", () -> getGyroscopeRotation().getDegrees());
    // tab.addNumber(
    //     "angular difference",
    //     () -> -Util.relativeAngularDifference(targetAngle, getGyroscopeRotation().getDegrees()));

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.addDouble("pitch", navx::getPitch);
      tab.addDouble("roll", navx::getRoll);
      tab.addDouble("sensorDistance", this::getSensorDistance);
    }

    Shuffleboard.getTab("DriverView").add(field).withPosition(0, 2).withSize(8, 4);
  }

  /** Return the current pose estimation of the robot */
  public Pose2d getPose() {
    return robotPose;
  }

  /** Return the kinematics object, for constructing a trajectory */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    return Config.DISABLE_SWERVE_MODULE_INIT
        ? new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
        }
        : new SwerveModulePosition[] {
          swerveModules[0].getPosition(),
          swerveModules[1].getPosition(),
          swerveModules[2].getPosition(),
          swerveModules[3].getPosition()
        };
  }

  private Rotation2d driverGyroOffset = Rotation2d.fromDegrees(0);

  /** Sets the gyro angle to zero, resetting the forward direction */
  public void zeroGyroscope() {
    driverGyroOffset = getConsistentGyroscopeRotation();
  }

  /** Aligns gyro heading with pose estimation */
  public void smartZeroGyroscope() {
    driverGyroOffset =
        getConsistentGyroscopeRotation()
            .minus(
                swervePoseEstimator
                    .getEstimatedPosition()
                    .getRotation()
                    .plus(
                        DriverStation.getAlliance() == Alliance.Blue
                            ? new Rotation2d()
                            : Rotation2d.fromDegrees(180)));
  }

  /**
   * Resets the odometry estimate to a specific pose.
   *
   * @param pose The pose to reset to.
   */
  public void resetOdometryToPose(Pose2d pose) {

    // "Zero" the driver gyro heading
    driverGyroOffset =
        getConsistentGyroscopeRotation()
            .minus(pose.getRotation())
            .plus(
                DriverStation.getAlliance() == Alliance.Blue
                    ? new Rotation2d()
                    : Rotation2d.fromDegrees(180));

    swervePoseEstimator.resetPosition(
        getConsistentGyroscopeRotation(), getSwerveModulePositions(), pose);
  }

  /**
   * Gets the current angle of the robot, relative to boot position. This value will not be reset,
   * and is used for odometry.
   *
   * <p>Use this value for odometry.
   *
   * @return The current angle of the robot, relative to boot position.
   */
  public Rotation2d getConsistentGyroscopeRotation() {
    return Rotation2d.fromDegrees(Util.normalizeDegrees(-navx.getAngle()));
  }

  /**
   * Gets the current angle of the robot, relative to the last time zeroGyroscope() was called. This
   * is not the same as the angle of the robot on the field, which is what getPose().getRotation()
   * returns. This is the angle of the robot as the driver sees it.
   *
   * <p>Use this value for driving the robot.
   *
   * @return The current angle of the robot, relative to the last time zeroGyroscope() was called.
   */
  public Rotation2d getDriverGyroscopeRotation() {

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
    // the angle increase.
    double angle = Util.normalizeDegrees(-navx.getAngle());

    // We need to subtract the offset here so that the robot drives forward based on auto
    // positioning or manual reset
    return Rotation2d.fromDegrees(angle).minus(driverGyroOffset);
  }

  public double getRotVelocity() {
    return navx.getRate();
  }

  /**
   * Tells the subsystem to drive, and puts the state machine in drive mode
   *
   * @param chassisSpeeds the speed of the chassis desired
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    this.mode = Modes.DRIVE;
    this.chassisSpeeds = chassisSpeeds;
  }

  public void driveAngle(Pair<Double, Double> xyInput, double targetAngle) {
    this.xyInput = xyInput;
    this.targetAngle = targetAngle;
    if (mode != Modes.DRIVE_ANGLE) rotController.reset();
    mode = Modes.DRIVE_ANGLE;
  }

  public double getSensorDistance() {
    return zappyThing.getRange();
  }

  public Status getSensorStatus() {
    return zappyThing.getStatus();
  }

  public RangingMode getSensorRangingMode() {
    return zappyThing.getRangingMode();
  }

  /*
   * Configure the ranging mode as well as the sample rate of the
   * time of flight sensor The ranging mode specifies the
   * trade off between maximum measure distance verses
   * reliablity in bright situations.
   * -from docs
   */
  public void setSensorRangingMode(RangingMode mode, double sampleTime) {
    zappyThing.setRangingMode(mode, sampleTime);
  }

  // makes the sensor flash red and green
  public void flashSensor() {
    zappyThing.identifySensor();
  }

  // is last measurement valid
  public boolean isSensorRangeValid() {
    return zappyThing.isRangeValid();
  }

  public void setSensorRangeOfInterest(
      int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    zappyThing.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

  /**
   * gets the current mode of the drivebase subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  /**
   * Angles the swerve modules in a cross shape, to make the robot hard to push. This function sets
   * the state machine to defense mode, so it only needs to be called once
   */
  public void setDefenseMode() {
    mode = Modes.DEFENSE;
  }

  /**
   * Updates the robot pose estimation for newly written module states. Should be called on every
   * periodic
   */
  private void odometryPeriodic() {
    this.robotPose =
        swervePoseEstimator.update(getConsistentGyroscopeRotation(), getSwerveModulePositions());

    VisionMeasurement measurement;
    while ((measurement = visionSubsystem.drainVisionMeasurement()) != null) {
      swervePoseEstimator.addVisionMeasurement(
          measurement.estimation().estimatedPose.toPose2d(),
          measurement.estimation().timestampSeconds,
          measurement.confidence());
    }
  }

  private void drivePeriodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    // sets swerve module speeds and angles, for each swerve module, using kinematics
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].set(
          states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[i].angle.getRadians());
    }
  }

  // called in drive to angle mode
  private void driveAnglePeriodic() {
    double angularDifference =
        -Util.relativeAngularDifference(getDriverGyroscopeRotation(), targetAngle);

    double rotationValue = rotController.calculate(angularDifference);

    // we are treating this like a joystick, so -1 and 1 are its lower and upper bound
    rotationValue = MathUtil.clamp(rotationValue, -1, 1);

    // this value makes our unit-less [-1, 1] into [-max angular, max angular]
    double omegaRadiansPerSecond = rotationValue * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // initialize chassis speeds but add our desired angle
    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xyInput.getFirst(),
            xyInput.getSecond(),
            omegaRadiansPerSecond,
            getDriverGyroscopeRotation());

    // use the existing drive periodic logic to assign to motors ect
    drivePeriodic();
  }

  @SuppressWarnings("java:S1121")
  private void defensePeriodic() {
    int angle = 45;
    for (SwerveModule module : swerveModules) {
      // the *= -1 operation multiplies the current variable by -1, stores it, and also returns
      // the value. We can use this to alternate between 45 and -45 for each module.
      module.set(0, angle *= -1);
    }

    // No need to call odometry periodic
  }

  public record RollPitch(double roll, double pitch) {
    public static RollPitch fromAHRS(AHRS navx) {
      return new RollPitch(navx.getRoll(), navx.getPitch());
    }

    public double absRoll() {
      return Math.abs(roll);
    }

    public double absPitch() {
      return Math.abs(pitch);
    }
  }

  public RollPitch getRollPitch() {
    return RollPitch.fromAHRS(navx);
  }

  /**
   * Based on the current Mode of the drivebase, perform the mode-specific logic such as writing
   * outputs (may vary per mode).
   *
   * @param mode The mode to use (should use the current mode value)
   */
  public void updateModules(Modes mode) {
    if (Config.DISABLE_SWERVE_MODULE_INIT) return;
    switch (mode) {
      case DRIVE -> drivePeriodic();
      case DRIVE_ANGLE -> driveAnglePeriodic();
      case DEFENSE -> defensePeriodic();
    }
  }

  /** For use in #periodic, to calculate the timestamp between motor writes */
  private double lastTimestamp = 0.0;

  @Override
  public void periodic() {
    /* Calculate time since last run and update odometry accordingly */
    final double timestamp = Timer.getFPGATimestamp();
    final double dt = timestamp - lastTimestamp;
    lastTimestamp = timestamp;

    /* get the current set-points for the drivetrain */
    Modes currentMode = getMode();
    Pose2d pose = getPose();

    field.setRobotPose(swervePoseEstimator.getEstimatedPosition());
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      SmartDashboard.putString(
          "pose",
          String.format(
              "(%2f %2f %2f)",
              swervePoseEstimator.getEstimatedPosition().getX(),
              swervePoseEstimator.getEstimatedPosition().getY(),
              swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees()));
    }

    /*
     * See if there is a new drive signal from the trajectory follower object.
     * An Optional means that this value might be "present" or not exist (be null),
     * but provides somewhat more convenient semantics for checking if there is a
     * value or not without a great risk of causing an Exception.
     */
    Optional<ChassisSpeeds> trajectorySpeeds = follower.update(pose, timestamp, dt);

    /* If there is a trajectory signal, overwrite the current chassis speeds setpoint to that trajectory value*/
    if (trajectorySpeeds.isPresent()) {
      this.chassisSpeeds = trajectorySpeeds.get();
    }

    /* Write outputs, corresponding to our current Mode of operation */
    updateModules(currentMode);

    /* Update odometry */
    odometryPeriodic();
  }

  public static ChassisSpeeds produceChassisSpeeds(
      boolean isRobotRelativeForward,
      boolean isRobotRelativeBackward,
      double x,
      double y,
      double rotationVelocity,
      Rotation2d currentGyroAngle) {
    if (isRobotRelativeForward) return new ChassisSpeeds(x, y, rotationVelocity);
    if (isRobotRelativeBackward) return new ChassisSpeeds(-x, -y, rotationVelocity);
    return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotationVelocity, currentGyroAngle);
  }
}
