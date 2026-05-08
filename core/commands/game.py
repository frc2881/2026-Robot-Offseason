from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from wpimath import units
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot
    
  def alignRobotToTargetPose(self, target: Target, alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target), alignRotationOnly)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTargetPose:{ target.name }')
    )

  def alignRobotToNearestTargetPose(self, targets: list[Target], alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getNearestTargetPose(targets), alignRotationOnly)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName("Game:AlignRobotToNearestTargetPose")
    )

  def alignRobotRotationToNearestBump(self) -> Command:
    return (
      self.alignRobotToNearestTargetPose([Target.BumpLeftInOut, Target.BumpLeftOutIn, Target.BumpRightInOut, Target.BumpRightOutIn], alignRotationOnly = True)
      .withName("Game:RotateRobotToNearestBump")
    )

  def alignTurretToActiveTarget(self) -> Command:
    return (
      self._robot.turret.setHeading(lambda: self._robot.targeting.getActiveTargetInfo().heading)
      .withName("Game:AlignTurretToActiveTarget")
    )
  
  def alignTurretToHeading(self, heading: units.degrees) -> Command:
    return (
      self._robot.turret.setHeading(lambda: heading)
      .withName(f'Game:AlignTurretToHeading:{ heading }deg')
    )
  
  def runIntake(self) -> Command:
    return (
      self._robot.intake.run_()
      .withName("Game:RunIntake")
    )
  
  def retractIntake(self) -> Command:
    return (
      self._robot.intake.retract()
      .withName("Game:RetractIntake")
    )

  def agitateHopper(self) -> Command:
    return (
      self._robot.hopper.reverse().withTimeout(constants.Subsystems.Hopper.AGITATION_TIMEOUT)
      .withName("Game:AgitateHopper")
    )
  
  def agitateRobot(self) -> Command:
    return (
      (
        (self._robot.drive.drive(lambda: 0.15, lambda: 0, lambda: 0).withTimeout(0.1))
        .andThen(self._robot.drive.drive(lambda: -0.15, lambda: 0, lambda: 0).withTimeout(0.1))
        .andThen(self._robot.drive.drive(lambda: 0, lambda: 0, lambda: 0).withTimeout(0.02))
      )
      .finallyDo(lambda end: self._robot.drive.reset())
      .withName("Game:AgitateRobot")
    )

  def launchFuel(self) -> Command:
    return (
      cmd.startEnd(
        lambda: self._robot.targeting.setIsActiveTargetEngaged(True),
        lambda: self._robot.targeting.setIsActiveTargetEngaged(False)
      )
      .deadlineFor(
        self.alignTurretToActiveTarget(),
        self._robot.launcher.run_(lambda: self._robot.targeting.getActiveTargetInfo().speed),
        cmd.waitUntil(lambda: self._robot.launcher.isAtTargetSpeed()).withTimeout(constants.Game.Commands.LAUNCHER_READY_TIMEOUT).andThen(
          self._robot.hopper.run_(lambda: self._robot.targeting.isActiveTargetInRange())
          .deadlineFor(self._robot.intake.agitate())
        )
      )
      .onlyIf(lambda: self._robot.targeting.getActiveTarget() is not None)
      .onlyWhile(lambda: self._robot.targeting.getActiveTarget() is not None)
      .withName("Game:LaunchFuel")
    )

  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.Both, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.Short
  ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Operator),
      self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Driver)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
