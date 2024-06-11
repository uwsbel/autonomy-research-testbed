def get_dynamics(dt, dyn):
    """Import the dynamics model from utils.

    The 4DOF dynamics used by some localization packages is defined in the shared Dynamics file. This shared function passes the dynamics object to whichever localization package requires it.
    """
    from .dynamics import Dynamics

    return Dynamics(dt, dyn)


def get_coordinate_transfer():
    """Import the coordinate transfer from utils.

    The graph object from `chrono_coordinate_transfer` defines the Local Tangent Plane on which the GPS coordinates are projected. This shared function passes the object to whichever localization package requires it.
    """
    from .chrono_coordinate_transfer import Graph

    return Graph()
