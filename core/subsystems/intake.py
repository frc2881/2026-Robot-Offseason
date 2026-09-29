from typing import Callable
from enum import Enum, auto
from wpilib import Timer
from commands2 import Subsystem, Command, cmd
from lib import logger, telemetry, utils
from lib.classes import IdleMode
from lib.components.relative_position_control_module import RelativePositionControlModule
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_control_module import FollowerControlModule
from core.classes import FuelLevel
import core.constants as constants

class IntakeState(Enum):
  Idle = auto()
  Running = auto()
  Agitating = auto()
  Ejecting = auto()
  Retracting = auto()

class Intake(Subsystem):
  def __init__(
      self,
      getFuelLevel: Callable[[], FuelLevel]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake
    self._getFuelLevel = getFuelLevel

    self._arm = RelativePositionControlModule(self._constants.ARM_CONFIG)
    self._rollersLeader = VelocityControlModule(self._constants.ROLLERS_LEADER_CONFIG)
    self._rollersFollower = FollowerControlModule(self._constants.ROLLERS_FOLLOWER_CONFIG)

    self._arm.setIdleMode(IdleMode.Coast)
    self._rollersLeader.setIdleMode(IdleMode.Coast)
    self._rollersFollower.setIdleMode(IdleMode.Coast)

    self._state = IntakeState.Idle
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
        case IntakeState.Agitating:
          self._arm.setPosition(self._constants.ARM_AGITATE_RANGE.min if self._isAgitatingIn else self._constants.ARM_AGITATE_RANGE.max)
          if self._arm.isAtTargetPosition() or self._agitationTimer.hasElapsed(self._constants.ARM_AGITATE_TIMEOUT):
            self._isAgitatingIn = not self._isAgitatingIn
            self._agitationTimer.restart()
          self._rollersLeader.setSpeed(self._constants.ROLLERS_AGITATE_SPEED)
        case IntakeState.Ejecting:
          self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
          self._rollersLeader.setSpeed(-self._constants.ROLLERS_INTAKE_SPEED)
        case IntakeState.Retracting:
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
      lambda: self._setState(IntakeState.Agitating),
      lambda: self._setState(IntakeState.Idle)
    ).beforeStarting(lambda: self._resetAgitation())
  
  def retract(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(IntakeState.Retracting),
      lambda: self._setState(IntakeState.Idle)
    )

  def eject(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(IntakeState.Ejecting),
      lambda: self._setState(IntakeState.Idle)
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
    telemetry.log("Robot/Subsystems/Intake/State", self._state.name)
    telemetry.log("Robot/Subsystems/Intake/IsExtended", self.isExtended())
    telemetry.log("Robot/Subsystems/Intake/IsRunning", self.isRunning())