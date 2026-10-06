import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d, Rectangle2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
import navx
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.path import FlippingUtil
from lib import logger, telemetry, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID,
  Range,
  State,
  SpeedMode,
  DriveOrientation,
  MotorModel,
  FeedForwardGains,
  SwerveDriveModuleGearKit,
  SwerveDriveModuleConfigConstants, 
  SwerveDriveModuleConfig, 
  SwerveDriveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  RelativePositionControlModuleConfig,
  VelocityControlModuleConfig,
  FollowerControlModuleConfig,
  XboxControllerConfig,
  ButtonControllerConfig,
  PoseSensorConfig,
  BinarySensorConfig,
  DistanceSensorConfig
)
import lib.constants
from core.classes import Target, Zone, LaunchMetric, FuelLevel

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-rebuilt-andymark.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(31.0)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(37.0)
    WHEEL_BASE: units.meters = units.inchesToMeters(20.5)
    TRACK_WIDTH: units.meters = units.inchesToMeters(26.5)

    _drivingMotorModel = MotorModel.NEO_VORTEX
    _swerveDriveModuleGearKit = SwerveDriveModuleGearKit.LOW
    _swerveDriveModuleConstants = SwerveDriveModuleConfigConstants(
      drivingControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingFreeSpeed = lib.constants.Motors.FREE_SPEEDS[_drivingMotorModel],
      drivingGearReduction = lib.constants.Drive.Swerve.GEAR_RATIOS[_swerveDriveModuleGearKit],
      drivingCurrentLimit = 60,
      drivingControlPID = PID(0.04, 0, 0),
      turningCurrentLimit = 20,
      turningControlPID = PID(1.0, 0, 0),
      turningEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2(),
      wheelDiameter = units.inchesToMeters(3.0),
      telemetryName = "Robot/Subsystems/Drive/Modules"
    )
    SWERVE_DRIVE_MODULE_CONFIGS: tuple[SwerveDriveModuleConfig, SwerveDriveModuleConfig, SwerveDriveModuleConfig, SwerveDriveModuleConfig] = (
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.FRONT_LEFT, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.FRONT_RIGHT, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.REAR_LEFT, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.REAR_RIGHT, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveDriveModuleConstants)
    )
    SWERVE_DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.chassisTranslation for c in SWERVE_DRIVE_MODULE_CONFIGS))

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = lib.constants.Drive.Swerve.FREE_SPEEDS[_drivingMotorModel][_swerveDriveModuleGearKit] * 1.0
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 720.0

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationControlPID = PID(4.0, 0, 0),
      translationMaxVelocity = 3.2,
      translationPositionTolerance = 0.15,
      rotationControlPID = PID(4.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationPositionTolerance = 5.0
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationControlPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 1.0
    )

    DRIFT_CORRECTION_CONSTANTS = HeadingAlignmentConstants(
      rotationControlPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

    SPEED_MODE = SpeedMode.COMPETITION
    DRIVE_ORIENTATION = DriveOrientation.FIELD
    DRIFT_CORRECTION = State.ENABLED

  class Intake:
    ARM_CONFIG = RelativePositionControlModuleConfig(
      id = 18, 
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = False,
      softLimitForward = 50.0,
      softLimitReverse = 0,
      controlPID = PID(1.0, 0, 0),
      outputRange = Range(-1.0, 0.8),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]),
      cruiseVelocity = 12000.0,
      maxAcceleration = 24000.0,
      allowedProfileError = 0.5,
      homingPosition = 0,
      homingSpeed = 0.5,
      positionConversionFactor = 1.0,
      telemetryName = "Robot/Subsystems/Intake/Arm"
    )
        
    ROLLERS_LEADER_CONFIG = VelocityControlModuleConfig(
      id = 17,
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 100,
      isInverted = False,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]),
      cruiseVelocity = 12000.0,
      maxAcceleration = 24000.0,
      allowedProfileError = 0.1,
      telemetryName = "Robot/Subsystems/Intake/Rollers/Leader"
    )

    ROLLERS_FOLLOWER_CONFIG = FollowerControlModuleConfig(
      id = 19,
      leaderId = 17,
      controllerType = ROLLERS_LEADER_CONFIG.controllerType,
      motorType = ROLLERS_LEADER_CONFIG.motorType,
      currentLimit = ROLLERS_LEADER_CONFIG.currentLimit,
      isInverted = True,
      telemetryName = "Robot/Subsystems/Intake/Rollers/Follower"
    )

    ARM_RETRACT_POSITION: float = 10.0
    ARM_INTAKE_POSITION: float = 48.0
    ARM_AGITATE_RANGE = Range(24.0, 48.0)
    ARM_AGITATE_TIMEOUT: units.seconds = 1.0
    ROLLERS_INTAKE_SPEED: units.percent = 1.0
    ROLLERS_AGITATE_SPEED: units.percent = 0.1

  class Hopper:
    INDEXER_CONFIG = VelocityControlModuleConfig(
      id = 14,
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = True,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]),
      cruiseVelocity = 6000.0,
      maxAcceleration = 12000.0,
      allowedProfileError = 0.1,
      telemetryName = "Robot/Subsystems/Hopper/Indexer"
    )

    ELEVATOR_CONFIG = VelocityControlModuleConfig(
      id = 16,
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = False,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]),
      cruiseVelocity = 6000.0,
      maxAcceleration = 12000.0,
      allowedProfileError = 0.1,
      telemetryName = "Robot/Subsystems/Hopper/Elevator"
    )    
    
    INDEXER_RUN_SPEED: units.percent = 0.8
    ELEVATOR_RUN_SPEED: units.percent = 1.0
    AGITATE_SPEED: units.percent = 0.75
    INDEXER_RUN_DELAY: units.seconds = 0.5

    FUEL_LEVEL_SENSOR_DISTANCES: dict[FuelLevel, units.millimeters] = {
      FuelLevel.FULL: 220,
      FuelLevel.MID: 340,
      FuelLevel.LOW: 460
    }

  class Turret:
    TURRET_CONFIG = RelativePositionControlModuleConfig( 
      id = 13, 
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = False,
      softLimitForward = 320.0,
      softLimitReverse = -10.0,
      controlPID = PID(0.02, 0, 0.002),
      outputRange = Range(-1.0, 1.0),
      feedForwardGains  = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]),
      cruiseVelocity = 30000.0, 
      maxAcceleration = 60000.0,
      allowedProfileError = 0.25,
      homingPosition = -20.25,
      homingSpeed = 0.1,
      positionConversionFactor = 360.0 / 21.0,
      telemetryName = "Robot/Subsystems/Turret"
    )

    ROTATION_RANGE = Range(-10.0, 320.0)
    WRAP_ANGLE_INPUT_RANGE = Range(-10.0, 350.0)

  class Launcher:
    LAUNCHER_LEADER_CONFIG = VelocityControlModuleConfig(
      id = 10,
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = True,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]),
      cruiseVelocity = 6000.0,
      maxAcceleration = 12000.0,
      allowedProfileError = 0.1,
      telemetryName = "Robot/Subsystems/Launcher/Leader"
    )     

    LAUNCHER_FOLLOWER_CONFIG = FollowerControlModuleConfig(
      id = 11,
      leaderId = 10,
      controllerType = LAUNCHER_LEADER_CONFIG.controllerType,
      motorType = LAUNCHER_LEADER_CONFIG.motorType,
      currentLimit = LAUNCHER_LEADER_CONFIG.currentLimit,
      isInverted = True,
      telemetryName = "Robot/Subsystems/Launcher/Follower"
    )

    LAUNCHER_ACCELERATOR_CONFIG = VelocityControlModuleConfig(
      id = 12,
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = False,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]),
      cruiseVelocity = 6000.0,
      maxAcceleration = 12000.0,
      allowedProfileError = 0.1,
      telemetryName = "Robot/Subsystems/Launcher/Accelerator"
    )       

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
    LATENCY_COMPENSATION: units.seconds = 0.1
    VELOCITY_COMPENSATION_RANGE: Range = Range(0.1, 3.5)
    FUEL_DRAG_COEFFICIENT: float = 0.15
    TURRET_HEADING_TOLERANCE: units.degrees = 8.0

class Sensors: 
  class Gyro:
    NAVX_PORT = navx.AHRS.NavXComType.kUSB1
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        cameraName = "FrontLeft", 
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-0.5), y = units.inchesToMeters(14.5), z = units.inchesToMeters(18.0)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-5.5), yaw = units.degreesToRadians(88.0))
        ),
        stream = "http://10.28.81.6:1186/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout,
        telemetryName = "Robot/Sensors/Pose"
      ),
      PoseSensorConfig(
        cameraName = "FrontRight",
        transform = Transform3d(
        Translation3d(x = units.inchesToMeters(1.0), y = units.inchesToMeters(-14.0), z = units.inchesToMeters(8.75)),
        Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-17.0), yaw = units.degreesToRadians(-90.0))
      ),
        stream = "http://10.28.81.7:1184/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout,
        telemetryName = "Robot/Sensors/Pose"
      ),
      PoseSensorConfig(
        cameraName = "RearLeft",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-33.5), yaw = units.degreesToRadians(160.0))
        ),
        stream = "http://10.28.81.6:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout,
        telemetryName = "Robot/Sensors/Pose"
      ),
      PoseSensorConfig(
        cameraName = "RearRight",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(-12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-34.0), yaw = units.degreesToRadians(-160.0))
        ),
        stream = "http://10.28.81.7:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout,
        telemetryName = "Robot/Sensors/Pose"
      )
    )

  class Proximity:
    INDEXER_FUEL_SENSOR_CONFIG = BinarySensorConfig( 
      channel = 2,
      telemetryName = "Robot/Sensors/Indexer/Fuel"
    )

    HOPPER_FUEL_SENSOR_CONFIG = DistanceSensorConfig( 
      channel = 1, 
      pulseWidthConversionFactor = 2.0, 
      minTargetDistance = 0, 
      maxTargetDistance = 580,
      telemetryName = "Robot/Sensors/Hopper/Fuel"
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1184/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_CONFIG = XboxControllerConfig(port = 0, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Driver")
  OPERATOR_CONTROLLER_CONFIG = XboxControllerConfig(port = 1, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Operator")
  HOMING_BUTTON_CONFIG = ButtonControllerConfig(channel = 0, telemetryName = "Robot/Controllers/Homing")

class Game:
  class Robot:
    TYPE = RobotType.COMPETITION
    NAME: str = "Rosetta Stone (Offseason)"

  class Commands:
    pass

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = Rectangle2d(Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
      Alliance.BLUE: {
        Target.HUB: Pose3d(4.625, 4.030, 1.263, Rotation3d(Rotation2d.fromDegrees(180.0))), 
        Target.SHUTTLE_RIGHT: Pose3d(4.200, 2.400, 0, Rotation3d(Rotation2d.fromDegrees(180.0))), 
        Target.SHUTTLE_LEFT: Pose3d(4.200, 5.600, 0, Rotation3d(Rotation2d.fromDegrees(180.0))),
        Target.BUMP_ALLIANCE_ZONE_RIGHT: Pose3d(3.300, 2.600, 0, Rotation3d(Rotation2d.fromDegrees(-135.0))),
        Target.BUMP_ALLIANCE_ZONE_LEFT: Pose3d(3.300, 5.700, 0, Rotation3d(Rotation2d.fromDegrees(-135.0))),
        Target.BUMP_NEUTRAL_ZONE_RIGHT: Pose3d(5.800, 2.400, 0, Rotation3d(Rotation2d.fromDegrees(45.0))),
        Target.BUMP_NEUTRAL_ZONE_LEFT: Pose3d(5.800, 5.500, 0, Rotation3d(Rotation2d.fromDegrees(45.0)))
      },
      Alliance.RED: {}
    }
    for target in TARGETS[Alliance.BLUE]:
      pose = FlippingUtil.flipFieldPose(TARGETS[Alliance.BLUE][target].toPose2d())
      TARGETS[Alliance.RED][target] = Pose3d(pose.X(), pose.Y(), TARGETS[Alliance.BLUE][target].Z(), Rotation3d(pose.rotation()))

    ZONES: dict[Alliance, dict[Zone, Rectangle2d]] = {
      Alliance.BLUE: {
        Zone.ALLIANCE_ZONE_RIGHT: Rectangle2d(Translation2d(0, 0), Translation2d(4.400, 4.022)),
        Zone.ALLIANCE_ZONE_LEFT: Rectangle2d(Translation2d(0, 4.022), Translation2d(4.400, 8.043)),
        Zone.NEUTRAL_ZONE_RIGHT: Rectangle2d(Translation2d(5.600, 0), Translation2d(11.350, 4.022)),
        Zone.NEUTRAL_ZONE_LEFT: Rectangle2d(Translation2d(5.600, 4.022), Translation2d(11.350, 8.043))
      },
      Alliance.RED: {}
    }
    for zone in ZONES[Alliance.BLUE]:
      rectangle = ZONES[Alliance.BLUE][zone]
      ZONES[Alliance.RED][zone] = Rectangle2d(FlippingUtil.flipFieldPose(rectangle.center()), rectangle.xwidth, rectangle.ywidth)