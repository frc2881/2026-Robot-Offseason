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
  HUB = auto()
  SHUTTLE_RIGHT = auto()
  SHUTTLE_LEFT = auto()
  BUMP_ALLIANCE_ZONE_RIGHT = auto()
  BUMP_ALLIANCE_ZONE_LEFT = auto()
  BUMP_NEUTRAL_ZONE_RIGHT = auto()
  BUMP_NEUTRAL_ZONE_LEFT = auto()

class Zone(Enum):
  ALLIANCE_ZONE_RIGHT = auto()
  ALLIANCE_ZONE_LEFT = auto()
  NEUTRAL_ZONE_RIGHT = auto()
  NEUTRAL_ZONE_LEFT = auto()

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
  EMPTY = 0
  LOW = 1
  MID = 2
  FULL = 3

class MatchState(Enum):
  STOPPED = auto()
  AUTO = auto()
  TRANSITION = auto()
  SHIFT_1 = auto()
  SHIFT_2 = auto()
  SHIFT_3 = auto()
  SHIFT_4 = auto()
  END_GAME = auto()

class HubState(Enum):
  INACTIVE = auto()
  ACTIVE = auto()

class LightsMode(Enum):
  DEFAULT = auto()
  ROBOT_NOT_CONNECTED = auto()
  ROBOT_NOT_HOMED = auto()
  ROBOT_IS_HOMING = auto()
  VISION_NOT_READY = auto()
  HUB_STATE_ACTIVE = auto()
  HUB_STATE_ACTIVE_ENDING = auto()
  HUB_STATE_INACTIVE = auto()
  HUB_STATE_INACTIVE_ENDING = auto()
  ACTIVE_TARGET_NOT_IN_RANGE = auto()
