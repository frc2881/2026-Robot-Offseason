from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, Timer
from wpimath import units
from lib import logger, utils
from core.classes import FuelLevel
import core.constants as constants
from lib.components.velocity_control_module import VelocityControlModule

class Hopper(Subsystem):
  def __init__(
      self,
      getHopperSensorDistance: Callable[[], units.millimeters],
      getIndexerSensorHasTarget: Callable[[], bool]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Hopper
    self._getHopperSensorDistance = getHopperSensorDistance
    self._getIndexerSensorHasTarget = getIndexerSensorHasTarget

    self._indexer = VelocityControlModule(self._constants.INDEXER_CONFIG)
    self._elevator = VelocityControlModule(self._constants.ELEVATOR_CONFIG)

    self._isReversing: bool = False
    self._isRunning: bool = False

    self._jamDetectionTimer = Timer()

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    if self._isReversing:
      self._indexer.setSpeed(-self._constants.INDEXER_REVERSE_SPEED)
      self._elevator.setSpeed(-self._constants.ELEVATOR_REVERSE_SPEED)
    elif self._isRunning:
      self._indexer.setSpeed(self._constants.INDEXER_SPEED)
      self._elevator.setSpeed(self._constants.ELEVATOR_SPEED)
    else:
      self.reset()

  def run_(self, isEnabled: Callable[[], bool]) -> Command:
    return cmd.runEnd(
      lambda: setattr(self, "_isRunning", isEnabled()),
      lambda: setattr(self, "_isRunning", False)
    )
  
  def reverse(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isReversing", True),
      lambda: setattr(self, "_isReversing", False)
    )

  def isRunning(self) -> bool:
    return self._isRunning

  def reset(self) -> None:
    self._indexer.reset()
    self._elevator.reset()

  def getFuelLevel(self) -> FuelLevel:
    distance = self._getHopperSensorDistance()
    if distance > -1:
      if distance <= self._constants.FUEL_LEVEL_SENSOR_DISTANCES[FuelLevel.Full]:
        return FuelLevel.Full
      if distance <= self._constants.FUEL_LEVEL_SENSOR_DISTANCES[FuelLevel.Mid]:
        return FuelLevel.Mid
      if distance <= self._constants.FUEL_LEVEL_SENSOR_DISTANCES[FuelLevel.Low] or self._getIndexerSensorHasTarget():
        return FuelLevel.Low
    return FuelLevel.Empty
  
  def isJammed(self) -> bool:
    if (
      self.isRunning() and 
      not self._getIndexerSensorHasTarget() and 
      self.getFuelLevel() != FuelLevel.Empty and 
      self._jamDetectionTimer.hasElapsed(self._constants.JAM_DETECTION_TIMEOUT)
    ):
      return True
    else:
      self._jamDetectionTimer.restart()
      return False

  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Robot/Hopper/FuelLevel", self.getFuelLevel().name)
    SmartDashboard.putBoolean("Robot/Hopper/IsRunning", self.isRunning())
    SmartDashboard.putBoolean("Robot/Hopper/IsJammed", self.isJammed())
