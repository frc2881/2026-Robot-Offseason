from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from wpimath import units
from wpimath.geometry import Pose3d, Rotation3d
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import AutoPath, Target, Zone
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

  def alignRobotToTargetPose(self, target: Target, alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target), alignRotationOnly)
      .withName(f'Game:AlignRobotToTargetPose:{ target.name }')
    )

  def alignRobotToNearestTargetPose(self, targets: list[Target], alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getNearestTargetPose(targets), alignRotationOnly)
      .withName("Game:AlignRobotToNearestTargetPose")
    )

  def alignRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target))
      .withName(f'Game:AlignRobotToTargetHeading:{ target.name }')
    )

  def driveRobotOverBump(self) -> Command:
    return (
      self.alignRobotToNearestTargetPose([Target.BumpAllianceZoneRight, Target.BumpAllianceZoneLeft, Target.BumpNeutralZoneRight, Target.BumpNeutralZoneLeft])
      .andThen(
        cmd.select({
          Zone.AllianceZoneRight: self._robot.auto.followPath(AutoPath.AZ_NZ_RIGHT).deadlineFor(self.alignTurretToHeading(0)),
          Zone.AllianceZoneLeft: self._robot.auto.followPath(AutoPath.AZ_NZ_LEFT).deadlineFor(self.alignTurretToHeading(0)),
          Zone.NeutralZoneRight: self._robot.auto.followPath(AutoPath.NZ_AZ_RIGHT).deadlineFor(self.alignTurretToHeading(225.0)),
          Zone.NeutralZoneLeft: self._robot.auto.followPath(AutoPath.NZ_AZ_LEFT).deadlineFor(self.alignTurretToHeading(135.0)),
        }, lambda: self._robot.localization.getRobotZone())
      )
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName("Game:DriveRobotOverBump")
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

  def ejectIntake(self) -> Command:
    return (
      self._robot.intake.eject()
      .withName("Game:EjectIntake")
    )

  def agitateHopper(self) -> Command:
    return (
      self._robot.hopper.agitate()
      .withName("Game:AgitateHopper")
    )
  
  def agitateRobot(self) -> Command:
    return (
      (
        (self._robot.drive.drive(lambda: -0.3, lambda: -0.3, lambda: 0).withTimeout(0.2))
        .andThen(self._robot.drive.drive(lambda: 0.2, lambda: 0.2, lambda: 0).withTimeout(0.2))
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
        self.agitateHopper().withTimeout(0.75).andThen(
          self._robot.hopper.run_(lambda: self._robot.targeting.isActiveTargetInRange()).deadlineFor(
            cmd.waitSeconds(constants.Game.Commands.INTAKE_AGITATE_DELAY).andThen(
              self._robot.intake.agitate()
            )
          )
        )
      )
      .onlyIf(lambda: self._robot.targeting.getActiveTarget() is not None)
      .onlyWhile(lambda: self._robot.targeting.getActiveTarget() is not None)
      .withName("Game:LaunchFuel")
    )

  def launchFuelDemo(self) -> Command:
    return (
      self.alignTurretToHeading(0)
      .deadlineFor(
        self._robot.launcher.run_(lambda: 0.35),
        self._robot.hopper.run_(lambda: True)
      )
      .withName("Game:LaunchFuelDemo")
    )

  def resetGyro(self) -> Command:
    return (
      self._robot.gyro.reset()
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .ignoringDisable(True)
      .withName("Game:ResetGyro")
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
