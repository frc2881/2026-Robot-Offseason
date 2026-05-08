from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
import core.constants as constants

class Turret(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Turret

    self._turret = RelativePositionControlModule(self._constants.TURRET_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setHeading(self, getHeading: Callable[[], units.degrees]) -> Command:
    return self.runEnd(
      lambda: self._turret.setPosition(utils.clampValue(getHeading(), self._constants.ROTATION_RANGE.min, self._constants.ROTATION_RANGE.max)),
      lambda: self.reset()
    )

  def getHeading(self) -> units.degrees:
    return self._turret.getPosition()

  def getTargetHeading(self) -> units.degrees:
    return self._turret.getTargetPosition()

  def isAtTargetHeading(self) -> bool:
    return self._turret.isAtTargetPosition()
  
  def resetToHome(self) -> Command:
    return self._turret.resetToHome(self).withName("Turret:ResetToHome")

  def isHoming(self) -> bool:
    return self._turret.isHoming()

  def isHomed(self) -> bool:
    return self._turret.isHomed()

  def reset(self) -> None:
    self._turret.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Turret/Heading", self.getHeading())
    SmartDashboard.putNumber("Robot/Turret/TargetHeading", self.getTargetHeading())
    SmartDashboard.putBoolean("Robot/Turret/IsAtTargetHeading", self.isAtTargetHeading())
