from typing import Optional


__doc__: str


def load_soccar() -> None:
    """
    Loads the geometry of a standard soccar field
    """


def load_soccer() -> None:
    """
    Loads the geometry of a standard soccer field
    """


def load_dropshot() -> None:
    """
    Loads the geometry of a standard dropshot field
    """


def load_hoops() -> None:
    """
    Loads the geometry of a standard hoops field
    """


def load_soccar_throwback() -> None:
    """
    Loads the geometry of the field Throwback Stadium
    """


def load_soccer_throwback() -> None:
    """
    Loads the geometry of the field Throwback Stadium
    """


try:
    from rlbot.utils.structures.game_data_struct import GameTickPacket
except ImportError:
    pass


def tick(packet: GameTickPacket, prediction_time: float=6.) -> None:
    """
    Parses the game tick packet from RLBot

    prediction_time: The number of seconds into the future to generate the ball prediction struct
    """


class ShotType:
    GROUND: int = 0
    JUMP: int = 1
    # DOUBLE_JUMP: int = 2
    # AERIAL: int = 3


class TargetOptions:
    min_slice: Optional[int]
    max_slice: Optional[int]
    use_absolute_max_values: Optional[bool]
    all: Optional[bool]

    def __init__(self, min_slice: Optional[int]=None, max_slice: Optional[int]=None, use_absolute_max_values: Optional[bool]=None, all: Optional[bool]=None) -> TargetOptions:
        """
        This class doesn't actually have a custom constructor.
        Due to limitations in PyO3, __new__ must be used instead of __init__.
        This function is for IDE type hints only.

        NOTE:
        You can still call TargetOptions() and pass in parameters to make a new instance.
        """
    def __new__(self, min_slice: Optional[int]=None, max_slice: Optional[int]=None, use_absolute_max_values: Optional[bool]=None, all: Optional[bool]=None) -> TargetOptions: ...
    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


class BallSlice:
    time: float
    location: tuple[float, float, float]
    velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]

    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


def get_slice(time: float) -> BallSlice:
    """
    Gets the ball information at some time game time in the future
    """


def new_target(left_target: tuple[float, float, float], right_target: tuple[float, float, float], car_index: int, options: Optional[TargetOptions]=None) -> int:
    """
    Creates a new target and returns the target's I.D.

    Targets get automatically deleted upon calling tick() if it hasn't been confirmed.
    """


def confirm_target(target_id: int) -> None:
    """
    Confirms a target so it isn't deleted upon calling tick()

    The target will persist until remove_target() is called
    """


def remove_target(target_id: int) -> None:
    """
    Removes a target, letting it's I.D. be reused 
    """


def print_targets() -> None:
    """
    Prints the current list of targets
    """


def get_targets_length() -> int:
    """
    Gets the number of targets
    """


class BasicShotInfo:
    found: bool
    time: float
    shot_type: Optional[ShotType]

    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


def get_shot_with_target(target_id: int, temporary: bool=False, may_ground_shot: Optional[bool]=None, may_jump_shot: Optional[bool]=None, only: bool=False) -> BasicShotInfo:
    """
    Searches the ball prediction struct for a shot

    temporary: Setting this to False will only return the time of the shot, if found
    may_ground_shot: Setting this to True will enable searching for ground shots, default is the opposite of only
    may_ground_shot: Setting this to True will enable searching for jump shots, default is the opposite of only
    only: Default False, set to True if you only want to search for the specified shot(s)
    """


class AdvancedShotInfo:
    shot_vector: tuple[float, float, float]
    final_target: tuple[float, float, float]
    distance_remaining: float
    required_jump_time: Optional[float]
    path_samples: list[tuple[float, float]]

    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


def get_data_for_shot_with_target(target_id: int) -> AdvancedShotInfo:
    """
    Gets information about the found shot
    """
