__doc__: str


def load_soccar():
    """
    Loads the geometry of a standard soccar field
    """
    pass


def load_dropshot():
    """
    Loads the geometry of a standard dropshot field
    """
    pass


def load_hoops():
    """
    Loads the geometry of a standard hoops field
    """
    pass


def load_soccar_throwback():
    """
    Loads the geometry of the field Throwback Stadium
    """
    pass


def set_gravity(z: float):
    """
    Tells the library what the current gravity is
    """
    pass


try:
    from rlbot.utils.structures.game_data_struct import GameTickPacket
except ImportError:
    pass


def tick(packet: GameTickPacket):
    """
    Parses the game tick packet from RLBot
    """
    pass


def get_slice(time: float) -> dict:
    """
    Gets the ball information at some time game time in the future
    """
    pass


def new_target(target_left: tuple[float, float, float], target_right: tuple[float, float, float], car_index: int, options: dict) -> int:
    """
    Creates a new target and returns the target's I.D.

    Targets get automatically deleted upon calling tick() if it hasn't been confirmed.
    """
    pass


def confirm_target(target_id: int):
    """
    Confirms a target so it isn't deleted upon calling tick()

    The target will persist until remove_target() is called
    """
    pass


def remove_target(target_id: int):
    """
    Removes a target, letting it's I.D. be reused 
    """
    pass


def print_targets():
    """
    Prints the current list of targets
    """
    pass


def get_shot_with_target(target_id: int) -> dict:
    """
    Searches the ball prediction struct for a shot
    """
    pass


def get_data_for_shot_with_target(target_id: int) -> dict:
    """
    Gets information about the found shot
    """
    pass
