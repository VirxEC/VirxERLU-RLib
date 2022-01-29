from typing import Optional


__doc__: str


def load_soccar() -> None:
    """
    Loads the geometry of a standard soccar field
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


try:
    from rlbot.utils.structures.game_data_struct import GameTickPacket
except ImportError:
    pass


def tick(packet: GameTickPacket, prediction_time: float=6.) -> None:
    """
    Parses the game tick packet from RLBot

    prediction_time: The number of seconds into the future to generate the ball prediction struct
    """


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


def new_target(left_target: tuple[float, float, float], right_target: tuple[float, float, float], car_index: int, min_slice: int=0, max_slice: int=720, use_absolute_max_values: bool=False, all: bool=False) -> int:
    """
    Creates a new target and returns the target's I.D.

    Targets get automatically deleted upon calling tick() if it hasn't been confirmed.

    OPTIONAL arguments (defaults recommended):

    min_slice: the prediction slice to start the search from
    max_slice: the prediction slice to end the search at
    use_absolute_max_values: whether to use the absolute max values for the search
    all: whether to search for all slices or just the first slice

    NOTE: max_slice isn't always 720, but is set to the current number of slices in the ball prediction struct upon creation 
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


class BasicShotInfo:
    found: bool
    time: Optional[float]

    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


def get_shot_with_target(target_id: int, temporary: bool=False) -> BasicShotInfo:
    """
    Searches the ball prediction struct for a shot

    temporary: Setting this to False will only return the time of the shot, if found
    """


class AdvancedShotInfo:
    final_target: tuple[float, float, float]
    distance_remaining: float
    path_samples: list[tuple[float, float]]

    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


def get_data_for_shot_with_target(target_id: int) -> AdvancedShotInfo:
    """
    Gets information about the found shot
    """
