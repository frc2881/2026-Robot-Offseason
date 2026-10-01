from typing import Callable
from wpilib import DriverStation
from wpimath import units
from lib import logger, telemetry, utils
from lib.classes import RobotState
from lib.controllers.lights import LightsController
from core.classes import LightsMode, MatchState, HubState

class Lights():
  def __init__(
      self,
      isHoming: Callable[[], bool],
      isHomed: Callable[[], bool],
      hasValidPoseSensorResult: Callable[[], bool],
      getMatchState: Callable[[], MatchState],
      getMatchStateTime: Callable[[], units.seconds],
      getHubState: Callable[[], HubState],
      isActiveTargetInRange: Callable[[], bool]
    ) -> None:
    self._isHoming = isHoming
    self._isHomed = isHomed
    self._hasValidPoseSensorResult = hasValidPoseSensorResult
    self._getMatchState = getMatchState
    self._getMatchStateTime = getMatchStateTime
    self._getHubState = getHubState
    self._isActiveTargetInRange = isActiveTargetInRange
    
    self._lightsController = LightsController()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateLights()

  def _updateLights(self) -> None:
    if not DriverStation.isDSAttached():
      self._lightsController.setMode(LightsMode.ROBOT_NOT_CONNECTED)
      return
    
    if utils.getRobotState() == RobotState.DISABLED:
      if self._isHoming():
        self._lightsController.setMode(LightsMode.ROBOT_IS_HOMING)
        return
      if not self._isHomed():
        self._lightsController.setMode(LightsMode.ROBOT_NOT_HOMED)
        return
      if not self._hasValidPoseSensorResult():
        self._lightsController.setMode(LightsMode.VISION_NOT_READY)
        return
    
    if utils.getRobotState() == RobotState.ENABLED:
      if not self._isActiveTargetInRange():
        self._lightsController.setMode(LightsMode.ACTIVE_TARGET_NOT_IN_RANGE)
        return
      if self._getMatchState() != MatchState.STOPPED:
        isMatchStateEnding = self._getMatchStateTime() < 5
        self._lightsController.setMode(
          (LightsMode.HUB_STATE_ACTIVE_ENDING if isMatchStateEnding else LightsMode.HUB_STATE_ACTIVE)
          if self._getHubState() == HubState.ACTIVE else 
          (LightsMode.HUB_STATE_INACTIVE_ENDING if isMatchStateEnding else LightsMode.HUB_STATE_INACTIVE)
        )
        return

    self._lightsController.setMode(LightsMode.DEFAULT)