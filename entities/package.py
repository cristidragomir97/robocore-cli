# entities/package.py

from dataclasses import dataclass, field
from typing import List, Optional
import json

@dataclass
class Package:
    build: bool 
    type: str 
    path: Optional[str] = None
    repository: Optional[str] = None
    mounts: List[str] = field(default_factory=list)
    launch: Optional[str] = None
    run: Optional[str] = None
    params:  List[str] = field(default_factory=list)
    config_files: List[str] = field(default_factory=list)  # Paths to config files
    ros_workspace_path: Optional[str] = None  # Path to ROS workspace if applicable
    branch: Optional[str] = None
    preinstall: Optional[List[str]] = None
    postinstall: Optional[List[str]] = None
    dockerfile_path: Optional[str] = None
    docker_tag: Optional[str] = None


    def __post_init__(self):
        pass

    def toJSON(self):
        return "[bold red]" + json.dumps(
            self,
            default=lambda o: o.__dict__, 
            indent=4) + "[/bold red]"

