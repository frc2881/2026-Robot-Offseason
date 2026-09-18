from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units

class AutoPath(Enum):
  BUMP_RIGHT_LOOP = auto()
  BUMP_LEFT_LOOP = auto()
  HUB_DEPOT = auto()
  CUSTOM = auto()
  AZ_NZ_RIGHT = auto()
  AZ_NZ_LEFT = auto()
  NZ_AZ_RIGHT = auto()
  NZ_AZ_LEFT = auto()

class Target(Enum):
  Hub = auto()
  ShuttleRight = auto()
  ShuttleLeft = auto()
  BumpAllianceZoneRight = auto()
  BumpAllianceZoneLeft = auto()
  BumpNeutralZoneRight = auto()
  BumpNeutralZoneLeft = auto()

class Zone(Enum):
  AllianceZoneRight = auto()
  AllianceZoneLeft = auto()
  NeutralZoneRight = auto()
  NeutralZoneLeft = auto()

@dataclass(frozen=False, slots=True)
class TargetInfo:
  distance: units.meters = 0
  speed: units.percent = 0
  heading: units.degrees = 0
  isDistanceValid: bool = False
  isHeadingValid: bool = False

@dataclass(frozen=True, slots=True)
class LaunchMetric:
  distance: units.meters
  speed: units.percent
  time: units.seconds

class FuelLevel(IntEnum):
  Empty = 0
  Low = 1
  Mid = 2
  Full = 3

class MatchState(Enum):
  Stopped = auto()
  Auto = auto()
  Transition = auto()
  Shift1 = auto()
  Shift2 = auto()
  Shift3 = auto()
  Shift4 = auto()
  EndGame = auto()

class HubState(Enum):
  Inactive = auto()
  Active = auto()

class LightsMode(Enum):
  Default = auto()
  RobotNotConnected = auto()
  RobotNotHomed = auto()
  RobotIsHoming = auto()
  VisionNotReady = auto()
  HubStateActive = auto()
  HubStateActiveEnding = auto()
  HubStateInactive = auto()
  HubStateInactiveEnding = auto()
  ActiveTargetNotInRange = auto()
