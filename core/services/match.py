from typing import Optional
from wpilib import DriverStation
from wpimath import units
from lib import logger, telemetry, utils
from lib.classes import Alliance, RobotState, RobotMode
from core.classes import MatchState, HubState

class Match():
  def __init__(self) -> None:
    self._selectedAlliance: Optional[Alliance] = None
    self._matchState = MatchState.STOPPED
    self._matchStateTime: units.seconds = 0
    self._hubState = HubState.INACTIVE

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateMatch()
    self._updateTelemetry()

  def _updateMatch(self) -> None:
    match DriverStation.getGameSpecificMessage()[:1]:
      case "R": self._selectedAlliance = Alliance.RED
      case "B": self._selectedAlliance = Alliance.BLUE
      case _: self._selectedAlliance = None

    if utils.getRobotState() == RobotState.ENABLED:
      matchTime = utils.getMatchTime()
      alliance = utils.getAlliance()
      if utils.getRobotMode() == RobotMode.AUTO:
        self._matchState = MatchState.AUTO
        self._matchStateTime = matchTime
        self._hubState = HubState.ACTIVE
      if utils.getRobotMode() == RobotMode.TELEOP:
        if utils.isValueWithinRange(matchTime, 131, 140):
          self._matchState = MatchState.TRANSITION
          self._matchStateTime = matchTime - 130
          self._hubState = HubState.ACTIVE
        elif utils.isValueWithinRange(matchTime, 106, 131):
          self._matchState = MatchState.SHIFT_1
          self._matchStateTime = matchTime - 105
          self._hubState = HubState.ACTIVE if alliance != self._selectedAlliance and self._selectedAlliance is not None else HubState.INACTIVE
        elif utils.isValueWithinRange(matchTime, 81, 106):
          self._matchState = MatchState.SHIFT_2
          self._matchStateTime = matchTime - 80
          self._hubState = HubState.ACTIVE if alliance == self._selectedAlliance and self._selectedAlliance is not None else HubState.INACTIVE
        elif utils.isValueWithinRange(matchTime, 56, 81):
          self._matchState = MatchState.SHIFT_3
          self._matchStateTime = matchTime - 55
          self._hubState = HubState.ACTIVE if alliance != self._selectedAlliance and self._selectedAlliance is not None else HubState.INACTIVE
        elif utils.isValueWithinRange(matchTime, 31, 56):
          self._matchState = MatchState.SHIFT_4
          self._matchStateTime = matchTime - 30
          self._hubState = HubState.ACTIVE if alliance == self._selectedAlliance and self._selectedAlliance is not None else HubState.INACTIVE
        elif utils.isValueWithinRange(matchTime, 0, 31):
          self._matchState = MatchState.END_GAME
          self._matchStateTime = matchTime
          self._hubState = HubState.ACTIVE
    else:
      self._matchState = MatchState.STOPPED
      self._hubState = HubState.INACTIVE
      self._matchStateTime = 0

  def getMatchState(self) -> MatchState:
    return self._matchState
  
  def getMatchStateTime(self) -> units.seconds:
    return self._matchStateTime
  
  def getHubState(self) -> HubState:
    return self._hubState

  def _updateTelemetry(self) -> None:
    telemetry.log("Match/Time",  utils.getMatchTime())
    telemetry.log("Match/SelectedAlliance", self._selectedAlliance.name if self._selectedAlliance is not None else "None")
    telemetry.log("Match/State", self.getMatchState().name)
    telemetry.log("Match/StateTime", self.getMatchStateTime())
    telemetry.log("Match/HubState", self.getHubState().name)
