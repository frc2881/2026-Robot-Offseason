from typing import Callable
from wpimath import units
from commands2 import Subsystem, Command
from lib import logger, telemetry, utils
from lib.classes import IdleMode
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_control_module import FollowerControlModule
import core.constants as constants

class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._telemetryName = "Robot/Subsystems/Launcher"

    self._launcherLeader = VelocityControlModule(self._constants.LAUNCHER_LEADER_CONFIG)
    self._launcherFollower = FollowerControlModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)
    self._launcherAccelerator = VelocityControlModule(self._constants.LAUNCHER_ACCELERATOR_CONFIG)

    self._launcherLeader.setIdleMode(IdleMode.COAST)
    self._launcherFollower.setIdleMode(IdleMode.COAST)
    self._launcherAccelerator.setIdleMode(IdleMode.COAST)

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self, getSpeed: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._setSpeed(getSpeed()),
      lambda: self.reset()
    )
  
  def _setSpeed(self, speed: units.percent) -> None:
    self._launcherLeader.setSpeed(speed)
    self._launcherAccelerator.setSpeed(speed * self._constants.LAUNCHER_ACCELERATOR_SPEED_RATIO)
  
  def isAtTargetSpeed(self) -> bool:
    return self._launcherLeader.isAtTargetSpeed() and self._launcherAccelerator.isAtTargetSpeed()

  def reset(self) -> None:
    self._launcherLeader.reset()
    self._launcherAccelerator.reset()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/Speed', self._launcherLeader.getSpeed())
    telemetry.log(f'{self._telemetryName}/TargetSpeed', self._launcherLeader.getTargetSpeed())
    telemetry.log(f'{self._telemetryName}/IsAtTargetSpeed', self.isAtTargetSpeed())