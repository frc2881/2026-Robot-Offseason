from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from lib import logger, utils
from lib.classes import RobotState, MotorIdleMode
from lib.components.relative_position_control_module import RelativePositionControlModule
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_module import FollowerModule
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
    self._rollersLeader = VelocityControlModule(self._constants.ROLLERS_LEADER_CONFIG)
    self._rollersFollower = FollowerModule(self._constants.ROLLERS_FOLLOWER_CONFIG)

    self._isRunning: bool = False
    self._isAgitating: bool = False
    self._isRetracting: bool = False
    self._isReversing: bool = False
    self._isEnabled: bool = False

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    if self._isEnabled != utils.getRobotState() == RobotState.Enabled:
      self._isEnabled = utils.getRobotState() == RobotState.Enabled
      self._arm.setIdleMode(MotorIdleMode.Coast if self._isEnabled else MotorIdleMode.Brake)
    if self._isRunning:
      if self._arm.getTargetPosition() != self._constants.ARM_INTAKE_POSITION:
        self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
      self._rollersLeader.setSpeed(self._constants.ROLLERS_INTAKE_SPEED if self.isExtended() else 0)
    elif self._isRetracting:
      if self._arm.getTargetPosition() != self._constants.ARM_RETRACT_POSITION:
        self._arm.setPosition(self._constants.ARM_RETRACT_POSITION)
    elif self._isAgitating:
      if self._arm.isAtTargetPosition():
        self._arm.setPosition(
          self._constants.ARM_AGITATE_RANGE.min
          if self._arm.getTargetPosition() == self._constants.ARM_AGITATE_RANGE.max else
          self._constants.ARM_AGITATE_RANGE.max
        )
      else:
        self._arm.setPosition(
          self._constants.ARM_AGITATE_RANGE.max
          if self._arm.getTargetPosition() == self._constants.ARM_AGITATE_RANGE.max else
          self._constants.ARM_AGITATE_RANGE.min
        )
      # self._rollersLeader.setSpeed(self._constants.ROLLERS_AGITATE_SPEED)
    elif self._isReversing:
      self._rollersLeader.setSpeed(-self._constants.ROLLERS_INTAKE_SPEED)
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
    )
  
  def reverse(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isReversing", True),
      lambda: setattr(self, "_isReversing", False)
    )

  def isExtended(self) -> bool:
    return self._arm.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.9
  
  def isRunning(self) -> bool:
    return self._rollersLeader.getSpeed() > 0.01
  
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
    SmartDashboard.putBoolean("Robot/Intake/IsExtended", self.isExtended())
    SmartDashboard.putBoolean("Robot/Intake/IsRunning", self.isRunning())