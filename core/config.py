# core/config.py
import os
import yaml
from .models import Host, Component, CommonPackage

class Config:
    def __init__(self, data, root):
        self.root            = root
        # … existing required/optional keys …


        # required
        self.ros_distro      = data['ros_distro']
        self.ros_domain_id   = data['ros_domain_id']
        self.registry        = data['registry']
        self.image_prefix    = data['image_prefix']

        self.enable_apt_caching = data.get('enable_apt_caching', False)

        # optional / with defaults
        self.deploy_mode     = data.get('deploy_mode', 'image')
        self.build_dir       = data.get('build_dir', '.robocore/build')
        self.components_dir  = data.get('components_dir', 'components')
        self.workspace_dir   = data.get('workspace_dir', 'ros_ws')
        self.compose_file    = data.get('compose_file', 'docker-compose.yml')
        self.mount_root      = data.get('mount_root', f"/home/{os.getlogin()}/ros_builds")
        self.docker_port     = data.get('docker_port', 2375)
        self.tag             = data.get('tag', 'latest')

        self.enable_dds_router = data.get('enable_dds_router', False)
        self.discovery_server = data.get('discovery_server', 'localhost')

        
        # NEW: system‐level apt packages for the base image
        self.apt_packages    = data.get('apt_packages', [])

        # coalesce None→[] for common & components
        common = data.get('common_packages') or []
        self.common_packages = [ CommonPackage.from_dict(c)
                                 for c in common ]

        comps  = data.get('components') or []
        self.components      = [ Component.from_dict(c, is_common=False, workspace_dir=self.workspace_dir)
                                 for c in comps ]

        # hosts: support either list or single dict
        raw_hosts = data.get('hosts') or data.get('host') or []
        self.hosts = [Host(**h) for h in raw_hosts]

    @classmethod
    def load(cls, project_root: str):
        path = os.path.join(project_root, 'config.yaml')
        if not os.path.isfile(path):
            raise FileNotFoundError(f"config.yaml not found in {project_root}")
        data = yaml.safe_load(open(path)) or {}
        return cls(data, project_root)

    @property
    def platforms(self):
        return [f"linux/{h.arch}" for h in self.hosts]

    @property
    def base_image(self):
        # Dynamically derived, no longer read from config.yaml
        return f"{self.registry}/{self.image_prefix}_base:{self.ros_distro}-{self.tag}"

    def filter_components(self, name=None, simulate=None):
        comps = self.components
        if name:
            comps = [c for c in comps if c.name == name]
        if simulate is True:
            comps = [c for c in comps if c.simulate]
        elif simulate is False:
            comps = [c for c in comps if not c.simulate]
        return comps
