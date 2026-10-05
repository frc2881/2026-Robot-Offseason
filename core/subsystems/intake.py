from typing import Callable
from enum import Enum, auto
from wpilib import Timer
from commands2 import Subsystem, Command, cmd
from lib import logger, telemetry, utils
from lib.classes import IdleMode
from lib.modules.relative_position_control import RelativePositionControlModule
from lib.modules.velocity_control import VelocityControlModule
from lib.modules.follower_control import FollowerControlModule
from core.classes import FuelLevel
import core.constants as constants

class IntakeState(Enum):
  IDLE = auto()
  RUNNING = auto()
  AGITATING = auto()
  EJECTING = auto()
  RETRACTING = auto()

class Intake(Subsystem):
  def __init__(
      self,
      getFuelLevel: Callable[[], FuelLevel]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake
    self._getFuelLevel = getFuelLevel

    self._telemetryName = "Robot/Subsystems/Intake"

    self._arm = RelativePositionControlModule(self._constants.ARM_CONFIG)
    self._rollersLeader = VelocityControlModule(self._constants.ROLLERS_LEADER_CONFIG)
    self._rollersFollower = FollowerControlModule(self._constants.ROLLERS_FOLLOWER_CONFIG)

    self._arm.setIdleMode(IdleMode.COAST)
    self._rollersLeader.setIdleMode(IdleMode.COAST)
    self._rollersFollower.setIdleMode(IdleMode.COAST)

    self._state = IntakeState.IDLE
    self._isRunning: bool = False
    self._isAgitatingIn: bool = True
    self._agitationTimer = Timer()

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    if self._isRunning:
      self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
      self._rollersLeader.setSpeed(self._constants.ROLLERS_INTAKE_SPEED if self.isExtended() else 0)
    else:
      match self._state:
        case IntakeState.AGITATING:
          self._arm.setPosition(self._constants.ARM_AGITATE_RANGE.min if self._isAgitatingIn else self._constants.ARM_AGITATE_RANGE.max)
          if self._arm.isAtTargetPosition() or self._agitationTimer.hasElapsed(self._constants.ARM_AGITATE_TIMEOUT):
            self._isAgitatingIn = not self._isAgitatingIn
            self._agitationTimer.restart()
          self._rollersLeader.setSpeed(self._constants.ROLLERS_AGITATE_SPEED)
        case IntakeState.EJECTING:
          self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
          self._rollersLeader.setSpeed(-self._constants.ROLLERS_INTAKE_SPEED)
        case IntakeState.RETRACTING:
          self._arm.setPosition(self._constants.ARM_RETRACT_POSITION)
          self._rollersLeader.setSpeed(0)
        case _:
          if not self.isHoming():
            self.reset()

  def _setState(self, state: IntakeState) -> None:
    self._state = state

  def _setIsRunning(self, isRunning: bool) -> None:
    self._isRunning = isRunning

  def run_(self) -> Command:
    return cmd.startEnd(
      lambda: self._setIsRunning(True),
      lambda: self._setIsRunning(False)
    )

  def agitate(self) -> Command:
    return cmd.runEnd(
      lambda: self._setState(IntakeState.AGITATING),
      lambda: self._setState(IntakeState.IDLE)
    ).beforeStarting(lambda: self._resetAgitation())
  
  def retract(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(IntakeState.RETRACTING),
      lambda: self._setState(IntakeState.IDLE)
    )

  def eject(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(IntakeState.EJECTING),
      lambda: self._setState(IntakeState.IDLE)
    )

  def _resetAgitation(self) -> None:
    self._isAgitatingIn = True
    self._agitationTimer.restart()

  def isExtended(self) -> bool:
    return self._arm.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.75
  
  def isRunning(self) -> bool:
    return self._rollersLeader.getSpeed() > 0.1
  
  def resetToHome(self) -> Command:
    return self._arm.resetToHome(self).withName("Intake:ResetToHome")

  def isHoming(self) -> bool:
    return self._arm.isHoming()

  def isHomed(self) -> bool:
    return self._arm.isHomed()

  def reset(self) -> None:
    self._arm.reset()
    self._rollersLeader.reset()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/State', self._state.name)
    telemetry.log(f'{self._telemetryName}/IsExtended', self.isExtended())
    telemetry.log(f'{self._telemetryName}/IsRunning', self.isRunning())