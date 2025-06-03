# commands/prepare_base.py
from core.config import Config
from core.renderer import TemplateRenderer
from core.docker import DockerHelper
import os
import sys

def prepare_base_main(project_root: str):
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    cfg = Config.load(project_root)
    renderer = TemplateRenderer(os.path.join(
        os.path.dirname(__file__), '..', 'templates'))
    docker = DockerHelper()

    ubuntu_map = {"humble": "jammy", "galactic": "jammy", "foxy": "focal"}
    ubuntu     = ubuntu_map.get(cfg.ros_distro, "jammy")
    base_df    = os.path.join(".", "Dockerfile.base")

    renderer.render_base(
        out_path     = base_df,
        ros_distro   = cfg.ros_distro,
        ubuntu       = ubuntu,
        common_pkgs  = cfg.common_packages,
        apt_packages = cfg.apt_packages
    )
    if not os.path.isfile(base_df):
        sys.exit(f"[prepare_base] ERROR: {base_df} missing after render")

    base_tag = f"{cfg.registry}/{cfg.image_prefix}_base:{cfg.ros_distro}-{cfg.tag}"
    print(f"[prepare_base] Building base image → {base_tag}")
    docker.build_multiarch(
        image_tag  = base_tag,
        context    = ".",
        dockerfile = "Dockerfile.base",
        platforms  = cfg.platforms,
        push       = True
    )
