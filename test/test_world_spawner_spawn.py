import math

import numpy as np

from simulation.world_spawner import WorldSpawner


def test_make_spawn_request_preserves_name_model_and_pose():
    request = WorldSpawner.make_spawn_request(
        model="<sdf version='1.8'><model name='test'/></sdf>",
        name="test_entity",
        position=np.asarray([1.0, -2.0, 3.0]),
        orientation=np.asarray([0.0, 0.0, math.pi / 2]),
    )

    factory = request.entity_factory
    assert factory.name == "test_entity"
    assert factory.sdf == "<sdf version='1.8'><model name='test'/></sdf>"
    assert factory.pose.position.x == 1.0
    assert factory.pose.position.y == -2.0
    assert factory.pose.position.z == 3.0
    assert math.isclose(factory.pose.orientation.x, 0.0, abs_tol=1e-9)
    assert math.isclose(factory.pose.orientation.y, 0.0, abs_tol=1e-9)
    assert math.isclose(factory.pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(factory.pose.orientation.w, math.sqrt(0.5))
