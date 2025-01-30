# entities/brick.py

from dataclasses import dataclass
from typing import Optional
import json

from .package import Package  # Import the Package class
from .endpoint import Endpoint

@dataclass
class Brick:
    id: str
    host: str
    package: Optional[Package] = None  # Package subfield
    endpoints: Optional[Endpoint] = None
    remote_path: Optional[str] = None
    docker_tag: Optional[str] = None


    def __post_init__(self):
        if self.package:
            pass

    def toJSON(self):
        return json.dumps(
            self,
            default=lambda o: o.__dict__, 
            sort_keys=True,
            indent=4)

