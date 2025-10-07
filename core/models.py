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
    folder: Optional[str] = None

    @classmethod
    def from_dict(cls, d):
        url = d['url']
        version = d['version']
        folder = d.get('folder')

        # Auto-derive folder from URL if not specified
        if not folder:
            # Extract repo name from URL (e.g., https://github.com/user/repo.git -> repo)
            folder = url.rstrip('/').split('/')[-1]
            if folder.endswith('.git'):
                folder = folder[:-4]

        return cls(url=url, version=version, folder=folder)

@dataclass
class CommonPackage:
    """Represents a common package that can be either VCS-based or local source-based"""
    name: str
    repositories: List[RepositorySpec] = field(default_factory=list)  # VCS-based
    source: List[str] = field(default_factory=list)  # Local folder-based

    @classmethod
    def from_dict(cls, d):
        """Parse common_package from dict, supporting both repositories and source formats"""
        name = d['name']

        # Parse repositories if present
        repos = []
        if 'repositories' in d:
            repos = [RepositorySpec.from_dict(r) for r in d['repositories']]

        # Parse source folders if present
        source = d.get('source', [])
        if isinstance(source, str):
            source = [source]  # Convert single string to list

        return cls(name=name, repositories=repos, source=source)

    @property
    def is_vcs_based(self) -> bool:
        """Returns True if this package uses VCS repositories"""
        return bool(self.repositories)

    @property
    def is_local_based(self) -> bool:
        """Returns True if this package uses local source folders"""
        return bool(self.source)

@dataclass
class Component:
    name: str
    # New source fields
    sources: List[str] = field(default_factory=list)  # Multiple source paths
    source: Optional[str] = None                      # Single source path
    # Legacy field for backward compatibility
    folder: Optional[str] = None
    entrypoint: str = ""
    launch_args: str = ""
    devices: List[str] = field(default_factory=list)
    ports:   List[str] = field(default_factory=list)
    preinstall:  List[str] = field(default_factory=list)
    runs_on: Optional[str] = None
    postinstall: List[str] = field(default_factory=list)
    repositories: List[RepositorySpec] = field(default_factory=list)
    apt_packages: List[str] = field(default_factory=list)
    pip_packages: List[str] = field(default_factory=list)
    simulate: bool = False
    image: Optional[str] = None
    workspace_dir: str = "ros_ws"  # Configurable workspace directory name

    @classmethod
    def from_dict(cls, d, is_common=False, workspace_dir="ros_ws"):
        repos = [RepositorySpec.from_dict(r) for r in d.get('repositories', [])]

        # Handle new source fields and backward compatibility
        sources = d.get('sources', [])
        source = d.get('source')
        folder = d.get('folder')

        # Normalize source field: if source is a list, move to sources
        if source and isinstance(source, list):
            sources = source
            source = None

        # Backward compatibility warning
        if folder and not sources and not source:
            print(f"WARNING: Component '{d['name']}' uses deprecated 'folder' field. "
                  f"Consider migrating to 'source' or 'sources' fields.")
            source = folder  # Migrate folder to source
            folder = None

        return cls(
            name        = d['name'],
            sources     = sources,
            source      = source,
            folder      = folder,
            entrypoint  = d.get('entrypoint', ""),
            launch_args = d.get('launch_args', ""),
            devices     = d.get('devices', []),
            ports       = d.get('ports', []),
            preinstall  = d.get('preinstall', []),
            postinstall = d.get('postinstall', []),
            repositories= repos,
            simulate    = d.get('simulate', False),
            apt_packages= d.get('apt_packages', []),
            pip_packages= d.get('pip_packages', []),
            runs_on     = d.get('runs_on', None),
            image       = d.get('image', None),
            workspace_dir= workspace_dir,
        )

    @property
    def managed_workspace(self) -> str:
        """Path to the managed workspace in .robocore/workspaces/{name}"""
        return f".robocore/workspaces/{self.name}"

    @property
    def workspace_src(self) -> str:
        """Path to src directory in managed workspace"""
        return os.path.join(self.managed_workspace, "src")

    @property
    def comp_src(self):
        """Legacy property for backward compatibility"""
        if self.folder:
            # Legacy path for backward compatibility
            return os.path.join(self.folder, self.workspace_dir, "src")
        else:
            # New managed workspace path
            return self.workspace_src

    @property
    def repos_file(self):
        """Path to repos.yaml file in managed workspace"""
        if self.folder:
            # Legacy path for backward compatibility
            return os.path.join(self.folder, self.workspace_dir, "repos.yaml")
        else:
            # New managed workspace path
            return os.path.join(self.managed_workspace, "repos.yaml")

    def get_source_paths(self) -> List[str]:
        """Get all source paths for this component"""
        if self.sources:
            return self.sources
        elif self.source:
            return [self.source]
        elif self.folder:
            # Legacy support: assume folder contains workspace_dir/src
            return [os.path.join(self.folder, self.workspace_dir, "src")]
        return []

    def image_tag(self, cfg):
        if self.image:
            return self.image
        return f"{cfg.registry}/{cfg.image_prefix}_{self.name}:{cfg.tag}"
