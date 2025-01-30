# entities/composition.py

from dataclasses import dataclass, field
from typing import List
from typing import Dict

from .brick import Brick
from .host import Host

@dataclass
class Composition:
    robot_name: str
    ros_version: str
    bricks: List[Brick] = field(default_factory=list)
    hosts: List[Host] = field(default_factory=list)

    def all_endpoints(self):
        pass

    def get_bricks_per_host(self, host_name):
        bricks_per_host = []

        # Iterate through each brick in the composition
        for brick in self.bricks:
            if brick.host == host_name:
                bricks_per_host.append(brick)

        return bricks_per_host


    def __str__(self):
        bricks_str = "\n  ".join(str(brick) for brick in self.bricks)
        return (f"Composition(robot_name='{self.robot_name}', ros_version='{self.ros_version}',\n"
                f"  bricks=[\n  {bricks_str}\n  ])")
