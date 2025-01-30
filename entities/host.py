# entities/host.py

from dataclasses import dataclass
from typing import Optional
import json 

@dataclass
class Host:
    name: str
    ip: str
    user: str
    arch: str
    password: Optional[str] = None  # Preferably use SSH keys
    keyfile: Optional[str] = None  # Path to SSH key file
    overlay_network: str = "shared-network"


    def toJSON(self):
        return  json.dumps(
            self,
            default=lambda o: o.__dict__, 
            indent=4)
