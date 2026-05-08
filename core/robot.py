from commands2 import cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.controllers.xbox import XboxController
from lib.controllers.button import ButtonController
from lib.classes import RobotState
from lib.sensors.gyro_navx2 import Gyro_NAVX2
from lib.sensors.pose import PoseSensor
from lib.sensors.distance import DistanceSensor
from lib.sensors.binary import BinarySensor
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive
from core.subsystems.intake import Intake
from core.subsystems.hopper import Hopper
from core.subsystems.turret import Turret
from core.subsystems.launcher import Launcher
from core.services.localization import Localization
from core.services.targeting import Targeting
from core.services.match import Match
from core.services.lights import Lights
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initCommands()
    self._initControllers()
    self._initTriggers()
    self._initTelemetry()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyro = Gyro_NAVX2(constants.Sensors.Gyro.NAVX2.COM_TYPE)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.POSE_SENSOR_CONFIGS)
    self.hopperSensor = DistanceSensor(constants.Sensors.Distance.HOPPER_SENSOR_CONFIG)
    self.indexerSensor = BinarySensor(constants.Sensors.Binary.INDEXER_SENSOR_CONFIG)

  def _initSubsystems(self) -> None:
    self.drive = Drive(lambda: self.gyro.getHeading())
    self.intake = Intake(lambda: self.hopper.getFuelLevel())
    self.hopper = Hopper(lambda: self.hopperSensor.getDistance(), lambda: self.indexerSensor.hasTarget())
    self.turret = Turret()
    self.launcher = Launcher()
    
  def _initServices(self) -> None:
    self.localization = Localization(lambda: self.gyro.getHeading(), lambda: self.drive.getModulePositions(), self.poseSensors)
    self.targeting = Targeting(lambda: self.localization.getRobotPose(), lambda: self.drive.getChassisSpeeds(), lambda: self.turret.getHeading())
    self.match = Match()
    self.lights = Lights(
      lambda: self.isHoming(), 
      lambda: self.isHomed(), 
      lambda: self.localization.hasValidPoseSensorResult(), 
      lambda: self.match.getMatchState(), 
      lambda: self.match.getMatchStateTime(), 
      lambda: self.match.getHubState(),
      lambda: self.targeting.isActiveTargetInRange()
    )

  def _initCommands(self) -> None:
    self.game = Game(self)
    self.auto = Auto(self)

  def _initControllers(self) -> None:
    DriverStation.silenceJoystickConnectionWarning(not utils.isCompetitionMode())
    self.driver = XboxController(constants.Controllers.DRIVER_CONTROLLER_PORT, constants.Controllers.INPUT_DEADBAND)
    self.operator = XboxController(constants.Controllers.OPERATOR_CONTROLLER_PORT, constants.Controllers.INPUT_DEADBAND)
    self.homingButton = ButtonController(constants.Controllers.HOMING_BUTTON_CONFIG)

  def _initTriggers(self) -> None:
    self._setupDriver()
    self._setupOperator()

    self.homingButton.pressed().debounce(1.0).whileTrue(
      cmd.parallel(
        self.intake.resetToHome(), 
        self.turret.resetToHome(),
        self.drive.holdCoastMode()
      ).onlyWhile(lambda: utils.getRobotState() == RobotState.Disabled)
      .ignoringDisable(True)
    .withName("HomingButton:Pressed"))

  def _setupDriver(self) -> None:
    self.drive.setDefaultCommand(self.drive.drive(self.driver.getLeftY, self.driver.getLeftX, self.driver.getRightX))
    self.driver.leftStick().whileTrue(self.drive.lockSwerveModules())
    # self.driver.rightStick().whileTrue(cmd.none())
    self.driver.leftTrigger().whileTrue(self.game.alignRobotRotationToNearestBump())
    self.driver.rightTrigger().whileTrue(self.game.runIntake())
    self.driver.rightBumper().whileTrue(self.game.retractIntake())
    # self.driver.rightBumper().whileTrue(cmd.none())
    # self.driver.a().whileTrue(cmd.none())
    # self.driver.b().whileTrue(cmd.none())
    # self.driver.y().whileTrue(cmd.none())
    # self.driver.x().whileTrue(cmd.none())
    # self.driver.povLeft().whileTrue(cmd.none())
    # self.driver.povRight().whileTrue(cmd.none())
    # self.driver.povUp().whileTrue(cmd.none())
    # self.driver.povDown().whileTrue(cmd.none())
    # self.driver.start().whileTrue(cmd.none())
    self.driver.back().debounce(0.5).whileTrue(self.gyro.reset().ignoringDisable(True))

  def _setupOperator(self) -> None:
    # self.operator.leftStick().whileTrue(cmd.none())
    # self.operator.rightStick().whileTrue(cmd.none())
    # self.operator.leftTrigger().whileTrue(cmd.none())
    self.operator.rightTrigger().whileTrue(self.game.launchFuel())
    self.operator.leftBumper().whileTrue(self.game.retractIntake())
    self.operator.rightBumper().whileTrue(self.game.agitateHopper())
    self.operator.a().whileTrue(self.game.alignTurretToActiveTarget())
    # self.operator.b().whileTrue(cmd.none())
    self.operator.y().whileTrue(self.game.alignTurretToHeading(0))
    # self.operator.x().whileTrue(cmd.none())
    # self.operator.povLeft().whileTrue(cmd.none())
    # self.operator.povRight().whileTrue(cmd.none())
    self.operator.povUp().debounce(0.5).whileTrue(self.turret.resetToHome())
    self.operator.povDown().debounce(0.5).whileTrue(self.intake.resetToHome())
    # self.operator.start().whileTrue(cmd.none())
    # self.operator.back().whileTrue(cmd.none())

  def _initTelemetry(self) -> None:
    SmartDashboard.putString("Game/Robot/Type", constants.Game.Robot.TYPE.name)
    SmartDashboard.putString("Game/Robot/Name", constants.Game.Robot.NAME)
    SmartDashboard.putNumber("Game/Field/Length", constants.Game.Field.LENGTH)
    SmartDashboard.putNumber("Game/Field/Width", constants.Game.Field.WIDTH)
    SmartDashboard.putNumber("Robot/Drive/Length", constants.Subsystems.Drive.BUMPER_LENGTH)
    SmartDashboard.putNumber("Robot/Drive/Width", constants.Subsystems.Drive.BUMPER_WIDTH)
    SmartDashboard.putString("Robot/Cameras/Driver", constants.Cameras.DRIVER_STREAM)
    SmartDashboard.putStringArray("Robot/Sensors/Pose/Names", tuple(c.name for c in constants.Sensors.Pose.POSE_SENSOR_CONFIGS))

  def _periodic(self) -> None:
    self._updateTelemetry()

  def disabledInit(self) -> None:
    self.reset()

  def autoInit(self) -> None:
    self.reset()

  def autoExit(self) -> None: 
    self.gyro.resetRobotToField(self.localization.getRobotPose())

  def teleopInit(self) -> None:
    self.reset()

  def testInit(self) -> None:
    self.reset()

  def simulationInit(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.drive.reset()

  def isHoming(self) -> bool:
    return self.intake.isHoming() or self.turret.isHoming()

  def isHomed(self) -> bool:
    return self.intake.isHomed() and self.turret.isHomed()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/IsHoming", self.isHoming())
    SmartDashboard.putBoolean("Robot/Status/IsHomed", self.isHomed())
