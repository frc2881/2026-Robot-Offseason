from typing import TYPE_CHECKING, Callable, Optional
import math
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d, Twist2d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from lib import logger, utils
from lib.classes import Alliance, Zone
from core.classes import Target, TargetInfo
import core.constants as constants

class Targeting():
  def __init__(
      self,
      getRobotPose: Callable[[], Pose2d],
      getChassisSpeeds: Callable[[], ChassisSpeeds],
      getTurretHeading: Callable[[], units.degrees]
    ) -> None:
    self._constants = constants.Services.Targeting
    self._getRobotPose = getRobotPose
    self._getChassisSpeeds = getChassisSpeeds
    self._getTurretHeading = getTurretHeading

    self._alliance: Optional[Alliance] = None
    self._targets: dict[Target, Pose3d] = {}
    self._targetZones: dict[Target, Zone] = {}

    self._launchDistances = tuple(t.distance for t in self._constants.LAUNCH_METRICS)
    self._launchSpeeds = tuple(t.speed for t in self._constants.LAUNCH_METRICS)
    self._launchTimes = tuple(t.time for t in self._constants.LAUNCH_METRICS)
    self._launchDistanceMin = self._launchDistances[0]
    self._launchDistanceMax = self._launchDistances[-1]
    self._launchHeadingMin = constants.Subsystems.Turret.ROTATION_RANGE.min
    self._launchHeadingMax = constants.Subsystems.Turret.ROTATION_RANGE.max

    self._activeTarget: Optional[Target] = None
    self._activeTargetInfo = TargetInfo()
    self._isActiveTargetEngaged: bool = False

    self._prvx: units.meters_per_second = 0
    self._prvy: units.meters_per_second = 0
    self._prvo: units.radians_per_second = 0

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTargets()
    self._updateActiveTarget()
    self._updateActiveTargetInfo()
    self._updateTelemetry()

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.TARGETS[self._alliance]
      self._targetZones = constants.Game.Field.Targets.TARGET_ZONES[self._alliance]

  def _updateActiveTarget(self) -> None:
    for target in self._targetZones:
      if utils.isPoseWithinZone(self._getRobotPose(), self._targetZones[target]):
        self._activeTarget = target
        return
    self._activeTarget = None

  def _updateActiveTargetInfo(self) -> None:
    if self._activeTarget is not None:
      robotPose = self._getRobotPose()
      rrv = self._getChassisSpeeds()
      frv = ChassisSpeeds.fromRobotRelativeSpeeds(rrv, robotPose.rotation())
      llc = self._constants.LATENCY_COMPENSATION
      prp = robotPose.exp(
        Twist2d(
          rrv.vx * llc + 0.5 * ((rrv.vx - self._prvx) / 0.02) * llc * llc, 
          rrv.vy * llc + 0.5 * ((rrv.vy - self._prvy) / 0.02) * llc * llc, 
          rrv.omega * llc + 0.5 * ((rrv.omega - self._prvo) / 0.02) * llc * llc
        )
      )
      self._prvx = rrv.vx
      self._prvy = rrv.vy
      self._prvo = rrv.omega
      h = prp.rotation().radians()
      cos = math.cos(h)
      sin = math.sin(h)
      ltx = constants.Subsystems.Launcher.LAUNCHER_TRANSFORM.X()
      lty = constants.Subsystems.Launcher.LAUNCHER_TRANSFORM.Y()
      lx = prp.X() + ltx * cos - lty * sin
      ly = prp.Y() + ltx * sin + lty * cos
      vx = frv.vx + (-(ltx * sin + lty * cos)) * frv.omega
      vy = frv.vy + (ltx * cos - lty * sin) * frv.omega
      tt = self.getTargetPose(self._activeTarget).translation().toTranslation2d()
      rx = tt.X() - lx
      ry = tt.Y() - ly
      distance: units.meters = math.hypot(rx, ry)
      speed: units.percent = 0
      rotation = Rotation2d()
      if math.hypot(vx, vy) < self._constants.VELOCITY_COMPENSATION_RANGE.min:
        speed = utils.getInterpolatedValue(distance, self._launchDistances, self._launchSpeeds)
        rotation = Rotation2d(tt.X() - lx, tt.Y() - ly)
      else:
        drag = self._constants.FUEL_DRAG_COEFFICIENT
        tof = utils.getInterpolatedValue(distance, self._launchDistances, self._launchTimes)
        dtof = (1.0 - math.exp(-drag * tof)) / drag
        distance = math.hypot((rx - vx * dtof), (ry - vy * dtof))
        speed = utils.getInterpolatedValue(distance, self._launchDistances, self._launchSpeeds)
        rotation = Rotation2d((tt.X() - vx * dtof) - lx, (tt.Y() - vy * dtof) - ly)
      heading = utils.wrapAngle(rotation.degrees() - robotPose.rotation().degrees(), constants.Subsystems.Turret.WRAP_ANGLE_INPUT_RANGE)
      isDistanceValid = False
      isHeadingValid = False
      if math.hypot(vx, vy) < self._constants.VELOCITY_COMPENSATION_RANGE.max:
        isDistanceValid = utils.isValueWithinRange(distance, self._launchDistanceMin, self._launchDistanceMax)
        isHeadingValid = utils.isValueWithinRange(heading, self._launchHeadingMin, self._launchHeadingMax)
      self._activeTargetInfo.distance = distance
      self._activeTargetInfo.speed = speed
      self._activeTargetInfo.heading = heading
      self._activeTargetInfo.isDistanceValid = isDistanceValid
      self._activeTargetInfo.isHeadingValid = isHeadingValid
    else:
      self._activeTargetInfo.distance = 0
      self._activeTargetInfo.speed = 0
      self._activeTargetInfo.heading = 0
      self._activeTargetInfo.isDistanceValid = False
      self._activeTargetInfo.isHeadingValid = False

  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])

  def getActiveTarget(self) -> Optional[Target]:
    return self._activeTarget 
  
  def getActiveTargetInfo(self) -> TargetInfo:
    return self._activeTargetInfo

  def setIsActiveTargetEngaged(self, isEngaged: bool) -> None:
    self._isActiveTargetEngaged = isEngaged

  def isActiveTargetInRange(self) -> bool:
    return ( 
      (
        self._activeTargetInfo.isDistanceValid and
        self._activeTargetInfo.isHeadingValid and
        utils.isValueWithinTolerance(
          self._getTurretHeading(), 
          self._activeTargetInfo.heading, 
          self._constants.TURRET_HEADING_TOLERANCE
        )
      ) if self._isActiveTargetEngaged else True
    )

  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Robot/Targeting/ActiveTarget", self._activeTarget.name if self._activeTarget is not None else "")
    SmartDashboard.putBoolean("Robot/Targeting/IsActiveTargetEngaged", self._isActiveTargetEngaged)
    SmartDashboard.putBoolean("Robot/Targeting/IsActiveTargetInRange", self.isActiveTargetInRange())
    SmartDashboard.putNumber("Robot/Targeting/ActiveTargetInfo/Distance", self._activeTargetInfo.distance)
    SmartDashboard.putNumber("Robot/Targeting/ActiveTargetInfo/Speed", self._activeTargetInfo.speed)
    SmartDashboard.putNumber("Robot/Targeting/ActiveTargetInfo/Heading", self._activeTargetInfo.heading)
    SmartDashboard.putBoolean("Robot/Targeting/ActiveTargetInfo/IsDistanceValid", self._activeTargetInfo.isDistanceValid)
    SmartDashboard.putBoolean("Robot/Targeting/ActiveTargetInfo/IsHeadingValid", self._activeTargetInfo.isHeadingValid)
