from typing import Callable
from enum import Enum, auto
from commands2 import Subsystem, Command, cmd
from wpilib import Timer
from lib import logger, telemetry, utils
from lib.classes import MotorIdleMode
from lib.components.relative_position_control_module import RelativePositionControlModule
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_module import FollowerModule
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
    self._rollers = VelocityControlModule(self._constants.ROLLERS_LEADER_CONFIG)
    self._rollersFollower = FollowerModule(self._constants.ROLLERS_FOLLOWER_CONFIG)

    self._arm.setIdleMode(MotorIdleMode.Coast)
    self._rollers.setIdleMode(MotorIdleMode.Coast)
    self._rollersFollower.setIdleMode(MotorIdleMode.Coast)

    self._state = IntakeState.Idle

    self._isAgitatingIn: bool = True
    self._agitationTimer = Timer()

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    match self._state:
      case IntakeState.Running:
        self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
        self._rollers.setSpeed(self._constants.ROLLERS_INTAKE_SPEED if self.isExtended() else 0)
      case IntakeState.Agitating:
        self._arm.setPosition(self._constants.ARM_AGITATE_RANGE.min if self._isAgitatingIn else self._constants.ARM_AGITATE_RANGE.max)
        if self._arm.isAtTargetPosition() or self._agitationTimer.hasElapsed(self._constants.ARM_AGITATE_TIMEOUT):
          self._isAgitatingIn = not self._isAgitatingIn
          self._agitationTimer.restart()
        self._rollers.setSpeed(self._constants.ROLLERS_AGITATE_SPEED)
      case IntakeState.Ejecting:
        self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
        self._rollers.setSpeed(-self._constants.ROLLERS_INTAKE_SPEED)
      case IntakeState.Retracting:
        self._arm.setPosition(self._constants.ARM_RETRACT_POSITION)
        self._rollers.setSpeed(0)
      case IntakeState.Idle:
        if not self.isHoming():
          self.reset()

  def _setState(self, state: IntakeState):
    self._state = state

  def run_(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(IntakeState.Running),
      lambda: self._setState(IntakeState.Idle)
    )
  
  def retract(self) -> Command:
    return cmd.startEnd(
      lambda: self._setState(IntakeState.Retracting),
      lambda: self._setState(IntakeState.Idle)
    )

  def agitate(self) -> Command:
    return cmd.runEnd(
      lambda: self._setState(IntakeState.Agitating),
      lambda: self._setState(IntakeState.Idle)
    ).beforeStarting(lambda: self._resetAgitation())
  
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
    return self._rollers.getSpeed() > 0.1
  
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
    telemetry.log("Robot/Intake/State", self._state.name)
    telemetry.log("Robot/Intake/IsExtended", self.isExtended())
    telemetry.log("Robot/Intake/IsRunning", self.isRunning())