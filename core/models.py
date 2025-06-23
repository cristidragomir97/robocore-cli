# core/models.py
import os
from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class Host:
    name: str
    ip: str
    user: str
    arch: str
    port: int = 2375
    manager: bool = False  # ← NEW

@dataclass
class RepositorySpec:
    url: str
    version: str
    folder: str

    @classmethod
    def from_dict(cls, d):
        return cls(d['url'], d['version'], d['folder'])

@dataclass
class Component:
    name: str
    folder: str
    entrypoint: str = ""
    launch_args: str = ""
    devices: List[str] = field(default_factory=list)
    ports:   List[str] = field(default_factory=list)
    preinstall:  List[str] = field(default_factory=list)
    runs_on: Optional[str] = None      # <— new field
    postinstall: List[str] = field(default_factory=list)
    repositories: List[RepositorySpec] = field(default_factory=list)
    apt_packages: List[str] = field(default_factory=list)
    simulate: bool = False
    image: Optional[str] = None

    @classmethod
    def from_dict(cls, d, is_common=False):
        repos = [RepositorySpec.from_dict(r) for r in d.get('repositories', [])]
        return cls(
            name        = d['name'],
            folder      = d.get('folder', ""),
            entrypoint  = d.get('entrypoint', ""),
            launch_args = d.get('launch_args', ""),
            devices     = d.get('devices', []),
            ports       = d.get('ports', []),
            preinstall  = d.get('preinstall', []),
            postinstall = d.get('postinstall', []),
            repositories= repos,
            simulate    = d.get('simulate', False),
            apt_packages= d.get('apt_packages', []),        # <— NEW
            runs_on     = d.get('runs_on', None),
            image       = d.get('image', None),
        )

    @property
    def comp_src(self):
        return os.path.join(self.folder, "ros_ws", "src")

    @property
    def repos_file(self):
        return os.path.join(self.folder, "ros_ws", "repos.yaml")

    def image_tag(self, cfg):
        if self.image:
            return self.image
        return f"{cfg.registry}/{cfg.image_prefix}_{self.name}:{cfg.tag}"
