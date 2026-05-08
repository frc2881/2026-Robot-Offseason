from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.classes import MotorIdleMode
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_module import FollowerModule
import core.constants as constants

class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._launcherLeader = VelocityControlModule(self._constants.LAUNCHER_LEADER_CONFIG)
    self._launcherFollower = FollowerModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)
    self._launcherAccelerator = VelocityControlModule(self._constants.LAUNCHER_ACCELERATOR_CONFIG)

    self._launcherLeader.setIdleMode(MotorIdleMode.Coast)
    self._launcherFollower.setIdleMode(MotorIdleMode.Coast)
    self._launcherAccelerator.setIdleMode(MotorIdleMode.Coast)

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self, getSpeed: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: [
        speed := getSpeed(),
        self._launcherLeader.setSpeed(speed),
        self._launcherAccelerator.setSpeed(speed * self._constants.LAUNCHER_ACCELERATOR_SPEED_RATIO)
      ],
      lambda: self.reset()
    )
  
  def isAtTargetSpeed(self) -> bool:
    return self._launcherLeader.isAtTargetSpeed() and self._launcherAccelerator.isAtTargetSpeed()

  def reset(self) -> None:
    self._launcherLeader.reset()
    self._launcherAccelerator.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Speed", self._launcherLeader.getSpeed())
    SmartDashboard.putNumber("Robot/Launcher/TargetSpeed", self._launcherLeader.getTargetSpeed())
    SmartDashboard.putBoolean("Robot/Launcher/IsAtTargetSpeed", self.isAtTargetSpeed())