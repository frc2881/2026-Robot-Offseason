import wpilib
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.path import FlippingUtil
from lib import logger, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID,
  Zone,
  Range,
  MotorModel,
  FeedForwardGains,
  SwerveModuleGearKit,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  RelativePositionControlModuleConstants,
  RelativePositionControlModuleConfig,
  VelocityControlModuleConfig,
  VelocityControlModuleConstants,
  FollowerModuleConfig,
  FollowerModuleConstants,
  ButtonControllerConfig,
  PoseSensorConfig,
  BinarySensorConfig,
  DistanceSensorConfig
)
from core.classes import Target, LaunchMetric, FuelLevel
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-rebuilt-andymark.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(31.0)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(37.0)
    WHEEL_BASE: units.meters = units.inchesToMeters(20.5)
    TRACK_WIDTH: units.meters = units.inchesToMeters(26.5)

    _drivingMotorModel = MotorModel.NEOVortex
    _swerveModuleGearKit = SwerveModuleGearKit.Low
    
    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorFreeSpeed = lib.constants.Motors.MOTOR_FREE_SPEEDS[_drivingMotorModel],
      drivingMotorReduction = lib.constants.Drive.SWERVE_MODULE_GEAR_RATIOS[_swerveModuleGearKit],
      drivingMotorCurrentLimit = 60,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0),
      turningMotorAbsoluteEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2()
    )

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, SwerveModuleConfig, SwerveModuleConfig, SwerveModuleConfig] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = lib.constants.Drive.SWERVE_MODULE_FREE_SPEEDS[_drivingMotorModel][_swerveModuleGearKit] * 1.0
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 540.0

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationPID = PID(4.0, 0, 0),
      translationMaxVelocity = 2.4,
      translationPositionTolerance = 0.1,
      rotationPID = PID(4.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationPositionTolerance = 3.0
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    DRIFT_CORRECTION_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

  class Intake:
    ARM_CONFIG = RelativePositionControlModuleConfig("Intake/Arm", 18, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorPID = PID(1.0, 0, 0),
      motorOutputRange = Range(-0.9, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 12000.0,
      motorMotionMaxAcceleration = 24000.0,
      motorMotionAllowedProfileError = 0.5,
      motorRelativeEncoderPositionConversionFactor = 1.0,
      motorSoftLimitForward = 50.0,
      motorSoftLimitReverse = 0,
      motorHomingSpeed = 0.5,
      motorHomedPosition = 0
    ))
        
    ROLLERS_LEADER_CONFIG = VelocityControlModuleConfig("Intake/Rollers/Leader", 17, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 100, 
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 12000.0,
      motorMotionMaxAcceleration = 24000.0
    ))

    ROLLERS_FOLLOWER_CONFIG = FollowerModuleConfig("Intake/Rollers/Follower", 19, 17, True, FollowerModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = ROLLERS_LEADER_CONFIG.constants.motorCurrentLimit
    ))

    ARM_RETRACT_POSITION: float = 10.0
    ARM_INTAKE_POSITION: float = 48.0
    ARM_AGITATE_RANGE = Range(28.0, 44.0)
    ARM_AGITATE_TIMEOUT: units.seconds = 1.0
    ROLLERS_INTAKE_SPEED: units.percent = 1.0
    ROLLERS_AGITATE_SPEED: units.percent = 0.1

  class Hopper:
    INDEXER_CONFIG = VelocityControlModuleConfig("Hopper/Indexer", 14, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0, 
      motorMotionMaxAcceleration = 12000.0
    ))

    ELEVATOR_CONFIG = VelocityControlModuleConfig("Hopper/Elevator", 16, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 12000.0
    ))
    
    INDEXER_SPEED: units.percent = 0.75
    ELEVATOR_SPEED: units.percent = 1.0
    INDEXER_REVERSE_SPEED: units.percent = 0.8
    ELEVATOR_REVERSE_SPEED: units.percent = 0.8

    INDEXER_RUN_DELAY: units.seconds = 0.25
    REVERSE_TIMEOUT: units.seconds = 2.0
    FUEL_LEVEL_SENSOR_DISTANCES: dict[FuelLevel, units.millimeters] = {
      FuelLevel.Full: 225,
      FuelLevel.Mid: 350,
      FuelLevel.Low: 475
    }

  class Turret:
    TURRET_CONFIG = RelativePositionControlModuleConfig("Turret", 13, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorRelativeEncoderPositionConversionFactor = 360.0 / 21.0,
      motorPID = PID(0.02, 0, 0.002),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains  = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 30000.0, 
      motorMotionMaxAcceleration = 60000.0,
      motorMotionAllowedProfileError = 0.25,
      motorSoftLimitForward = 320.0,
      motorSoftLimitReverse = -10.0,
      motorHomingSpeed = 0.1,
      motorHomedPosition = -20.25
    ))

    ROTATION_RANGE = Range(-10.0, 320.0)
    WRAP_ANGLE_INPUT_RANGE = Range(-10.0, 350.0)

  class Launcher:
    LAUNCHER_LEADER_CONFIG = VelocityControlModuleConfig("Launcher/Leader", 10, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 12000.0
    ))

    LAUNCHER_FOLLOWER_CONFIG = FollowerModuleConfig("Launcher/Follower", 11, 10, True, FollowerModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = LAUNCHER_LEADER_CONFIG.constants.motorCurrentLimit
    ))

    LAUNCHER_ACCELERATOR_CONFIG = VelocityControlModuleConfig("Launcher/Accelerator", 12, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 12000.0
    ))

    LAUNCHER_TRANSFORM = Transform3d(units.inchesToMeters(-4.75), units.inchesToMeters(7.875), units.inchesToMeters(25.3375), Rotation3d())
    LAUNCHER_ACCELERATOR_SPEED_RATIO: units.percent = 1.5

class Services:
  class Localization:
    MAX_TARGET_AMBIGUITY: units.percent = 0.2
    MAX_TARGET_REPROJECTION_ERROR: float = 1.0
    MAX_TARGET_DISTANCE: units.meters = 5.0
    MAX_POSE_CHANGE: units.meters = 1.0
    STDDEV_XY_COEFF: float = 0.08
    STDDEV_Z_COEFF: float = 0.1
    STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR: float = 5.0
    STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR: float = 2.5
    VALID_POSE_SENSOR_RESULT_TIMEOUT: units.seconds = 0.3

  class Targeting:
    LAUNCH_METRICS: tuple[LaunchMetric, ...] = (
      LaunchMetric(distance = 2.0, speed = 0.39, time = 0.95),
      LaunchMetric(distance = 2.5, speed = 0.42, time = 1.00),
      LaunchMetric(distance = 3.0, speed = 0.45, time = 1.05),
      LaunchMetric(distance = 3.5, speed = 0.48, time = 1.10),
      LaunchMetric(distance = 4.0, speed = 0.51, time = 1.15),
      LaunchMetric(distance = 4.5, speed = 0.54, time = 1.20),
      LaunchMetric(distance = 5.0, speed = 0.57, time = 1.25),
      LaunchMetric(distance = 6.0, speed = 0.63, time = 1.35),
      LaunchMetric(distance = 7.0, speed = 0.69, time = 1.45),
      LaunchMetric(distance = 8.0, speed = 0.75, time = 1.55),
      LaunchMetric(distance = 9.0, speed = 0.81, time = 1.65),
      LaunchMetric(distance = 10.0, speed = 0.87, time = 1.75)
    )
    LATENCY_COMPENSATION: units.seconds = 0.05
    VELOCITY_COMPENSATION_RANGE: Range = Range(0.1, 3.5)
    FUEL_DRAG_COEFFICIENT: float = 0.15
    TURRET_HEADING_TOLERANCE: units.degrees = 5.0

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "FrontLeft", 
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-0.5), y = units.inchesToMeters(14.5), z = units.inchesToMeters(18.0)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-5.5), yaw = units.degreesToRadians(88.0))
        ),
        stream = "http://10.28.81.6:1186/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "FrontRight",
        transform = Transform3d(
        Translation3d(x = units.inchesToMeters(1.0), y = units.inchesToMeters(-14.0), z = units.inchesToMeters(8.75)),
        Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-17.0), yaw = units.degreesToRadians(-90.0))
      ),
        stream = "http://10.28.81.7:1184/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "RearLeft",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-33.5), yaw = units.degreesToRadians(160.0))
        ),
        stream = "http://10.28.81.6:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "RearRight",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(-12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-34.0), yaw = units.degreesToRadians(-160.0))
        ),
        stream = "http://10.28.81.7:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      )
    )

  class Binary:
    INDEXER_SENSOR_CONFIG = BinarySensorConfig(
      name = "Indexer", 
      channel = 2
    )

  class Distance:
    HOPPER_SENSOR_CONFIG = DistanceSensorConfig(
      name = "Hopper", 
      channel = 1, 
      pulseWidthConversionFactor = 2.0, 
      minTargetDistance = 0, 
      maxTargetDistance = 580
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1184/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1
  HOMING_BUTTON_CONFIG = ButtonControllerConfig(name = "Homing", channel = 0)

class Game:
  class Robot:
    TYPE = RobotType.Competition
    NAME: str = "Rosetta Stone (Offseason)"

  class Commands:
    LAUNCHER_READY_TIMEOUT: units.seconds = 0.75
    BUMP_TRAVERSAL_DISTANCE: units.meters = 4.5
    INTAKE_AGITATE_DELAY: units.seconds = 1.75

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    ZONE = Zone(start = Translation2d(0, 0), end = Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Blue: {
          Target.Hub: Pose3d(4.625, 4.030, 1.263, Rotation3d(Rotation2d.fromDegrees(0))), 
          Target.ShuttleLeft: Pose3d(3.0, 5.25, 0, Rotation3d(Rotation2d.fromDegrees(180.0))),
          Target.ShuttleRight: Pose3d(3.0, 3.0, 0, Rotation3d(Rotation2d.fromDegrees(180.0))), 
          Target.BumpLeftAZ: Pose3d(2.800, 5.600, 0, Rotation3d(Rotation2d.fromDegrees(-135.0))),
          Target.BumpLeftNZ: Pose3d(6.400, 5.400, 0, Rotation3d(Rotation2d.fromDegrees(45.0))),
          Target.BumpRightAZ: Pose3d(2.800, 2.600, 0, Rotation3d(Rotation2d.fromDegrees(-135.0))),
          Target.BumpRightNZ: Pose3d(6.400, 2.400, 0, Rotation3d(Rotation2d.fromDegrees(45.0))),
        },
        Alliance.Red: {}
      }

      for target in TARGETS[Alliance.Blue]:
        pose = FlippingUtil.flipFieldPose(TARGETS[Alliance.Blue][target].toPose2d())
        TARGETS[Alliance.Red][target] = Pose3d(pose.X(), pose.Y(), TARGETS[Alliance.Blue][target].Z(), Rotation3d(pose.rotation()))

      TARGET_ZONES: dict[Alliance, dict[Target, Zone]] = {
        Alliance.Blue: {
          Target.Hub: Zone(start = Translation2d(0.0, 0.0), end = Translation2d(4.4, 8.0)),
          Target.ShuttleLeft: Zone(start = Translation2d(5.6, 5.5), end = Translation2d(16.5, 8.0)),
          Target.ShuttleRight: Zone(start = Translation2d(5.6, 0.0), end = Translation2d(16.5, 2.6))
        },
        Alliance.Red: {}
      }

      for target in TARGET_ZONES[Alliance.Blue]:
        zone = TARGET_ZONES[Alliance.Blue][target]
        TARGET_ZONES[Alliance.Red][target] = Zone(
          FlippingUtil.flipFieldPose(Pose2d(zone.end.X(), zone.end.Y(), Rotation2d())).translation(), 
          FlippingUtil.flipFieldPose(Pose2d(zone.start.X(), zone.start.Y(), Rotation2d())).translation()
        )