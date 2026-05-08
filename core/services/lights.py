from typing import Callable
from wpilib import DriverStation
from wpimath import units
from lib.classes import RobotState
from lib.controllers.lights import LightsController
from lib import logger, utils
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
      self._lightsController.setMode(LightsMode.RobotNotConnected)
      return
    
    if utils.getRobotState() == RobotState.Disabled:
      if self._isHoming():
        self._lightsController.setMode(LightsMode.RobotIsHoming)
        return
      if not self._isHomed():
        self._lightsController.setMode(LightsMode.RobotNotHomed)
        return
      if not self._hasValidPoseSensorResult():
        self._lightsController.setMode(LightsMode.VisionNotReady)
        return
    
    if utils.getRobotState() == RobotState.Enabled:
      if not self._isActiveTargetInRange():
        self._lightsController.setMode(LightsMode.ActiveTargetNotInRange)
        return
      if self._getMatchState() != MatchState.Stopped:
        isMatchStateEnding = self._getMatchStateTime() < 5
        self._lightsController.setMode(
          (LightsMode.HubStateActiveEnding if isMatchStateEnding else LightsMode.HubStateActive)
          if self._getHubState() == HubState.Active else 
          (LightsMode.HubStateInactiveEnding if isMatchStateEnding else LightsMode.HubStateInactive)
        )
        return

    self._lightsController.setMode(LightsMode.Default)