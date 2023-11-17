def get_dynamics(dt, dyn):
    """import the dynamics model from utils"""
    from .dynamics import Dynamics

    return Dynamics(dt, dyn)


def get_coordinate_transfer():
    """import the coordinate transfer from utils"""
    from .chrono_coordinate_transfer import Graph

    return Graph()
