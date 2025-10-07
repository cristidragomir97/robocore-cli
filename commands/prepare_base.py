# commands/prepare_base.py
from core.config import Config
from core.renderer import TemplateRenderer
from core.docker import DockerHelper
import os
import sys

def prepare_base_main(project_root: str):
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
    print(base_dockerfile)

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

    print(f"[prepare_base] Building base image {base_tag} for: {', '.join(platforms)}")
    # Use project root as context so COPY commands in Dockerfile can access source paths
    docker.build_multiarch(
        image_tag=base_tag,
        context=project_root,
        dockerfile=base_dockerfile,
        platforms=platforms,
        push=True
    )
