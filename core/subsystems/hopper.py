from typing import Callable
from enum import Enum, auto
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, Timer
from wpimath import units
from lib import logger, utils
from lib.classes import MotorIdleMode
from core.classes import FuelLevel
import core.constants as constants
from lib.components.velocity_control_module import VelocityControlModule

class HopperState(Enum):
  Idle = auto()
  Running = auto()
  Agitating = auto()

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

    self._elevator = VelocityControlModule(self._constants.ELEVATOR_CONFIG)
    self._indexer = VelocityControlModule(self._constants.INDEXER_CONFIG)

    self._elevator.setIdleMode(MotorIdleMode.Coast)
    self._indexer.setIdleMode(MotorIdleMode.Coast)

    self._state = HopperState.Idle

    self._indexerDelayTimer = Timer()

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    match self._state:
      case HopperState.Running:
        self._elevator.setSpeed(self._constants.ELEVATOR_RUN_SPEED)
        if self._indexerDelayTimer.hasElapsed(self._constants.INDEXER_RUN_DELAY):
          self._indexer.setSpeed(self._constants.INDEXER_RUN_SPEED)
      case HopperState.Agitating:
        self._elevator.setSpeed(-self._constants.AGITATE_SPEED)
        self._indexer.setSpeed(-self._constants.AGITATE_SPEED)
      case HopperState.Idle:
        self.reset()

  def _setState(self, state: HopperState):
    self._state = state

  def run_(self, isEnabled: Callable[[], bool]) -> Command:
    return cmd.runEnd(
      lambda: self._setState(HopperState.Running if isEnabled() else HopperState.Idle),
      lambda: self._setState(HopperState.Idle)
    ).beforeStarting(lambda: self._indexerDelayTimer.restart())
  
  def agitate(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(HopperState.Agitating),
      lambda: self._setState(HopperState.Idle)
    )

  def isRunning(self) -> bool:
    return self._elevator.getSpeed() > 0.1 and self._indexer.getSpeed() > 0.1

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
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Robot/Hopper/State", self._state.name)
    SmartDashboard.putString("Robot/Hopper/FuelLevel", self.getFuelLevel().name)
    SmartDashboard.putBoolean("Robot/Hopper/IsRunning", self.isRunning())
