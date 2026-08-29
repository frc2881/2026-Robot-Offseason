from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d, Rotation2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from lib import logger, utils
from lib.classes import Alliance
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class AutoPath(Enum):
  BUMP_LEFT_LOOP = auto()
  BUMP_RIGHT_LOOP = auto()
  HUB_DEPOT = auto()
  CUSTOM = auto()

class Auto:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }
    self._auto = cmd.none()

    AutoBuilder.configure(
      self._robot.localization.getRobotPose, 
      self._robot.localization.resetRobotPose,
      self._robot.drive.getChassisSpeeds, 
      self._robot.drive.setChassisSpeeds, 
      constants.Subsystems.Drive.PATHPLANNER_CONTROLLER,
      constants.Subsystems.Drive.PATHPLANNER_ROBOT_CONFIG,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.drive
    )

    self._autos = SendableChooser()
    self._autos.setDefaultOption("0: None", self.auto_NONE)
    
    self._autos.addOption("1: Bump Left Loop", self.auto_BUMP_LEFT_LOOP)
    self._autos.addOption("2: Bump Right Loop", self.auto_BUMP_RIGHT_LOOP)
    self._autos.addOption("3: Hub Depot", self.auto_HUB_DEPOT)
    # self._autos.addOption("6: Custom", self.auto_CUSTOM)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def set(self, auto: Command) -> None:
    self._auto = auto
    SmartDashboard.putString("Robot/Auto/command", auto.getName().replace("Auto:", ""))

  def _getPath(self, path: AutoPath) -> PathPlannerPath:
    return self._paths.get(path, PathPlannerPath([], PathConstraints(0, 0, 0, 0), None, GoalEndState(0, Rotation2d())))
  
  def _reset(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.resetOdom(self._getPath(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._getPath(path).getInitialHeading())))
      .andThen(cmd.waitSeconds(0.1))
    ).deadlineFor(logger.log_("Auto:Reset"))
  
  def _move(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.followPath(self._getPath(path))
    ).deadlineFor(logger.log_(f'Auto:Move:{path.name}'))
  
  def auto_NONE(self) -> Command:
    return cmd.none().withName("Auto:NONE")

  def auto_BUMP_LEFT_LOOP(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BUMP_LEFT_LOOP).deadlineFor(
        cmd.waitSeconds(1.5).andThen(self._robot.game.runIntake().deadlineFor(self._robot.game.alignTurretToHeading(200.0)))
      ),
      self._robot.game.launchFuel().deadlineFor(
        cmd.waitSeconds(2.0).andThen(self._robot.game.agitateRobot())
      )
    ).withName("Auto:BUMP_LEFT_LOOP")

  def auto_BUMP_RIGHT_LOOP(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BUMP_RIGHT_LOOP).deadlineFor(
        cmd.waitSeconds(1.5).andThen(self._robot.game.runIntake().deadlineFor(self._robot.game.alignTurretToHeading(165.0)))
      ),
      self._robot.game.launchFuel().deadlineFor(
        cmd.waitSeconds(2.0).andThen(self._robot.game.agitateRobot())
      )
    ).withName("Auto:BUMP_RIGHT_LOOP")

  def auto_HUB_DEPOT(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.HUB_DEPOT).deadlineFor(
        cmd.waitSeconds(0.25).andThen(self._robot.game.runIntake().deadlineFor(self._robot.game.alignTurretToHeading(100.0)))
      ),
      self._robot.game.launchFuel()
    ).withName("Auto:HUB_DEPOT")

  def auto_CUSTOM(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.CUSTOM).deadlineFor(
        self._robot.game.alignTurretToHeading(180.0)
      ),
      self._robot.game.launchFuel()
    ).withName("Auto:CUSTOM")
