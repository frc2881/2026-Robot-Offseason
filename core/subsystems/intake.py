from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, Timer
from wpimath import units
from lib import logger, utils
from lib.classes import Range
from lib.components.relative_position_control_module import RelativePositionControlModule
from lib.components.velocity_control_module import VelocityControlModule
from core.classes import FuelLevel
import core.constants as constants

class Intake(Subsystem):
  def __init__(
      self,
      getFuelLevel: Callable[[], FuelLevel]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake
    self._getFuelLevel = getFuelLevel

    self._arm = RelativePositionControlModule(self._constants.ARM_CONFIG)
    self._rollers = VelocityControlModule(self._constants.ROLLERS_CONFIG)

    self._isRunning: bool = False
    self._isAgitating: bool = False
    self._isRetracting: bool = False

    self._agitationTimer = Timer()

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    if self._isRunning:
      if self._arm.getTargetPosition() != self._constants.ARM_INTAKE_HOLD_POSITION:
        self._arm.setPosition(self._constants.ARM_INTAKE_HOLD_POSITION)
      self._rollers.setSpeed(self._constants.ROLLERS_INTAKE_SPEED if self.isExtended() else 0)
    elif self._isRetracting:
      if self._arm.getTargetPosition() != self._constants.ARM_RETRACT_POSITION:
        self._arm.setPosition(self._constants.ARM_RETRACT_POSITION)
      self._rollers.setSpeed(0)
    elif self._isAgitating:
      time: units.seconds = 1.0
      range = Range(0.1, 0.9)
      speed: units.percent = 0.3
      match self._getFuelLevel():
        case FuelLevel.Full:
          range = Range(0.7, 1.0)
          speed = 0.1
        case FuelLevel.Mid:
          range = Range(0.3, 0.6)
          speed = 0.2
        case _:
          range = Range(0.1, 0.4)
          speed = 0.3
      self._agitationTimer.advanceIfElapsed(time)
      position = self._constants.ARM_INTAKE_HARDSTOP_POSITION * (range.min if self._agitationTimer.get() < time * 0.6 else range.max)
      if self._arm.getTargetPosition() != position:
          self._arm.setPosition(position)  
      self._rollers.setSpeed(speed)
    else:
      if not self.isHoming():
        self.reset()

  def run_(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isRunning", True),
      lambda: setattr(self, "_isRunning", False)
    )
  
  def retract(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isRetracting", True),
      lambda: setattr(self, "_isRetracting", False)
    )

  def agitate(self) -> Command:
    return cmd.runEnd(
      lambda: setattr(self, "_isAgitating", True),
      lambda: setattr(self, "_isAgitating", False)
    ).beforeStarting(lambda: self._agitationTimer.restart())

  def isExtended(self) -> bool:
    return self._arm.getPosition() > self._constants.ARM_INTAKE_HARDSTOP_POSITION * 0.75
  
  def isRunning(self) -> bool:
    return self._rollers.getSpeed() > 0.01
  
  def resetToHome(self) -> Command:
    return self._arm.resetToHome(self).withName("Intake:ResetToHome")

  def isHoming(self) -> bool:
    return self._arm.isHoming()

  def isHomed(self) -> bool:
    return self._arm.isHomed()

  def reset(self) -> None:
    self._arm.reset()
    self._rollers.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsExtended", self.isExtended())
    SmartDashboard.putBoolean("Robot/Intake/IsRunning", self.isRunning())