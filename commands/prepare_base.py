# commands/prepare_base.py
from core.config import Config
from core.renderer import TemplateRenderer
from core.docker import DockerHelper
import os
import sys
import hashlib
import json

def compute_base_hash(cfg: Config, project_root: str) -> str:
    """Compute hash of base image inputs to detect changes."""
    hasher = hashlib.sha256()

    # Hash ROS distro and tag
    hasher.update(cfg.ros_distro.encode())
    hasher.update(cfg.tag.encode())

    # Hash common packages configuration
    for pkg in cfg.common_packages:
        hasher.update(pkg.name.encode())
        if pkg.repositories:
            for repo in pkg.repositories:
                hasher.update(repo.url.encode())
                hasher.update(repo.version.encode())
        if pkg.source:
            # Handle both string and list sources
            if isinstance(pkg.source, list):
                for src in pkg.source:
                    hasher.update(src.encode())
            else:
                hasher.update(pkg.source.encode())

    # Hash apt packages
    for apt_pkg in cfg.apt_packages:
        hasher.update(apt_pkg.encode())

    # Hash common package source files if they're local
    for pkg in cfg.common_packages:
        if pkg.source:
            # Handle both string and list sources
            sources = pkg.source if isinstance(pkg.source, list) else [pkg.source]
            for src in sources:
                source_path = os.path.join(project_root, src)
                if os.path.exists(source_path):
                    # Hash package.xml if it exists (contains version/dependencies)
                    pkg_xml = os.path.join(source_path, "package.xml")
                    if os.path.exists(pkg_xml):
                        with open(pkg_xml, 'rb') as f:
                            hasher.update(f.read())

    return hasher.hexdigest()

def load_base_hash(base_dir: str) -> str:
    """Load previously stored base image hash."""
    hash_file = os.path.join(base_dir, "base.hash")
    if os.path.exists(hash_file):
        with open(hash_file, 'r') as f:
            return f.read().strip()
    return ""

def save_base_hash(base_dir: str, hash_value: str):
    """Save base image hash."""
    hash_file = os.path.join(base_dir, "base.hash")
    with open(hash_file, 'w') as f:
        f.write(hash_value)

def prepare_base_main(project_root: str, force: bool = False):
    cfg = Config.load(project_root)
    tpl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'templates'))
    renderer = TemplateRenderer(tpl_dir)
    docker = DockerHelper()

    # Map ROS distro to Ubuntu release
    ubuntu_map = {
        "humble": "jammy",
        "galactic": "jammy",
        "foxy": "focal"
    }
    ubuntu = ubuntu_map.get(cfg.ros_distro, "jammy")

    # Generate Dockerfile.base
    base_dir = os.path.join(project_root, ".robocore", "base")
    os.makedirs(base_dir, exist_ok=True)
    base_dockerfile = os.path.join(base_dir, "Dockerfile.base")

    renderer.render_base(
        out_path=base_dockerfile,
        ros_distro=cfg.ros_distro,
        ubuntu=ubuntu,
        common_pkgs=cfg.common_packages,
        workspace_dir=cfg.workspace_dir,
    )

    base_tag = f"{cfg.registry}/{cfg.image_prefix}_base:{cfg.ros_distro}-{cfg.tag}"

    # Collect all unique target platforms from host definitions
    platforms = list({f"linux/{host.arch}" for host in cfg.hosts})

    # Check if we can skip rebuild
    if not force:
        current_hash = compute_base_hash(cfg, project_root)
        cached_hash = load_base_hash(base_dir)

        if current_hash == cached_hash:
            # Check if image exists locally
            try:
                docker.client.image.inspect(base_tag)
                print(f"[prepare_base] Base image {base_tag} is up to date (cached)")
                print(f"[prepare_base] Use 'stage --force-base' to rebuild")
                return
            except:
                print(f"[prepare_base] Base config unchanged but image not found locally, rebuilding...")
        else:
            print(f"[prepare_base] Base configuration changed, rebuilding...")

    print(f"[prepare_base] Building base image {base_tag} for: {', '.join(platforms)}")
    print(base_dockerfile)

    # Use project root as context so COPY commands in Dockerfile can access source paths
    docker.build_multiarch(
        image_tag=base_tag,
        context=project_root,
        dockerfile=base_dockerfile,
        platforms=platforms,
        push=True
    )

    # Save hash after successful build
    current_hash = compute_base_hash(cfg, project_root)
    save_base_hash(base_dir, current_hash)
    print(f"[prepare_base] Base image built and cached")
