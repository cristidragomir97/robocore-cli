from dataclasses import dataclass, field
from typing import List, Optional
import json

@dataclass
class Endpoint:
    type: str
    topic: str
    message: str
    frame_id: Optional[str] = None

    def __post_init__(self):
        pass


    def toJSON(self):
        return  json.dumps(
            self,
            default=lambda o: o.__dict__, 
            indent=4)
