from typing import Callable
from enum import Enum, auto
from wpilib import Timer
from wpimath import units
from commands2 import Subsystem, Command, cmd
from lib import logger, telemetry, utils
from lib.classes import IdleMode
from lib.modules.velocity_control import VelocityControlModule
from core.classes import FuelLevel
import core.constants as constants

class HopperState(Enum):
  IDLE = auto()
  RUNNING = auto()
  AGITATING = auto()

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

    self._telemetryName = "Robot/Subsystems/Hopper"

    self._elevator = VelocityControlModule(self._constants.ELEVATOR_CONFIG)
    self._indexer = VelocityControlModule(self._constants.INDEXER_CONFIG)

    self._elevator.setIdleMode(IdleMode.COAST)
    self._indexer.setIdleMode(IdleMode.COAST)

    self._state = HopperState.IDLE

    self._indexerDelayTimer = Timer()

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    match self._state:
      case HopperState.RUNNING:
        self._elevator.setSpeed(self._constants.ELEVATOR_RUN_SPEED)
        if self._indexerDelayTimer.hasElapsed(self._constants.INDEXER_RUN_DELAY):
          self._indexer.setSpeed(self._constants.INDEXER_RUN_SPEED)
      case HopperState.AGITATING:
        self._elevator.setSpeed(-self._constants.AGITATE_SPEED)
        self._indexer.setSpeed(-self._constants.AGITATE_SPEED)
      case HopperState.IDLE:
        self.reset()

  def _setState(self, state: HopperState):
    self._state = state

  def run_(self, isEnabled: Callable[[], bool]) -> Command:
    return cmd.runEnd(
      lambda: self._setState(HopperState.RUNNING if isEnabled() else HopperState.IDLE),
      lambda: self._setState(HopperState.IDLE)
    ).beforeStarting(lambda: self._indexerDelayTimer.restart())
  
  def agitate(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(HopperState.AGITATING),
      lambda: self._setState(HopperState.IDLE)
    )

  def isRunning(self) -> bool:
    return self._elevator.getSpeed() > 0.1 and self._indexer.getSpeed() > 0.1

  def reset(self) -> None:
    self._indexer.reset()
    self._elevator.reset()

  def getFuelLevel(self) -> FuelLevel:
    distance = self._getHopperSensorDistance()
    if distance > -1:
      if distance <= self._constants.FUEL_LEVEL_SENSOR_DISTANCES[FuelLevel.FULL]:
        return FuelLevel.FULL
      if distance <= self._constants.FUEL_LEVEL_SENSOR_DISTANCES[FuelLevel.MID]:
        return FuelLevel.MID
      if distance <= self._constants.FUEL_LEVEL_SENSOR_DISTANCES[FuelLevel.LOW] or self._getIndexerSensorHasTarget():
        return FuelLevel.LOW
    return FuelLevel.EMPTY
  
  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/State', self._state.name)
    telemetry.log(f'{self._telemetryName}/FuelLevel', self.getFuelLevel().name)
    telemetry.log(f'{self._telemetryName}/IsRunning', self.isRunning())
