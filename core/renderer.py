# core/renderer.py
import os
import platform
from jinja2 import Environment, FileSystemLoader, StrictUndefined


def is_macos() -> bool:
    """Check if running on macOS."""
    return platform.system() == 'Darwin'

class TemplateRenderer:
    def __init__(self, template_dir: str):
        self.env = Environment(
            loader=FileSystemLoader(template_dir),
            undefined=StrictUndefined,
            trim_blocks=True,
            lstrip_blocks=True
        )

    def render(self, tpl_name: str, out_path: str, **ctx):
        tpl = self.env.get_template(tpl_name)
        rendered = tpl.render(**ctx)
        with open(out_path, 'w') as f:
            f.write(rendered)

    def render_dockerfile(self,
                        out_path: str,
                        base_image: str,
                        ros_distro: str,
                        ros_domain_id: int,
                        comp,
                        source_packages: list,
                        repos_file: str,
                        superclient_path: str,
                        apt_packages,
                        pip_packages,
                        postinstall,
                        has_repos: bool,
                        comp_src_exists: bool,
                        enable_apt_caching: bool,
                        dds_server_ip: str):
        self.render(
            "Dockerfile.j2",
            out_path,
            base_image=base_image,
            ros_distro=ros_distro,
            ros_domain_id=ros_domain_id,
            comp=comp,
            source_packages=source_packages,
            repos_file=repos_file,
            superclient_path=superclient_path,
            apt_packages=apt_packages,
            pip_packages=pip_packages,
            postinstall=postinstall,
            has_repos=has_repos,
            comp_src_exists=comp_src_exists,
            enable_apt_caching=enable_apt_caching,
            dds_server_ip=dds_server_ip
        )


    def render_superclient(self, out_path: str, *,
                        participantID: int,
                        this_host_ip: str,
                        dds_server_host_ip: str):
        self.render(
            "superclient.xml.j2",
            out_path,
            participantID=participantID,
            this_host_ip=this_host_ip,
            dds_server_host_ip=dds_server_host_ip
        )

    def render_compose(self, out_path, components, cfg, host=None, dds_manager=None):
        # Use dds_manager's effective_dds_ip if available, otherwise fall back to cfg.discovery_server
        discovery_server = dds_manager.effective_dds_ip if dds_manager else cfg.discovery_server

        # Check if host is localhost
        is_localhost = host and host.ip in ('localhost', '127.0.0.1', '::1')

        # For localhost, use local build directory; otherwise use remote mount_root
        if is_localhost:
            # Use absolute path to local build directory
            mount_root = os.path.abspath(os.path.join(cfg.root, cfg.build_dir))
        else:
            mount_root = host.effective_mount_root if host else cfg.mount_root

        self.render(
            "docker-compose.j2",
            out_path,
            components    = components,
            mount_root    = mount_root,
            ros_distro    = cfg.ros_distro,
            ros_domain_id = cfg.ros_domain_id,
            registry      = cfg.registry,
            base_image    = cfg.base_image,
            enable_dds_router = cfg.enable_dds_router,
            discovery_server = discovery_server,
            dds_manager       = dds_manager,
            has_common_packages = bool(cfg.common_packages),
            is_macos      = is_macos(),
            is_localhost  = is_localhost
        )

    def render_base(self, out_path: str, ros_distro: str, ubuntu: str, common_pkgs, workspace_dir: str = "ros_ws", apt_packages = [], apt_mirror: str = None, ros_apt_mirror: str = None):
            """
            Render Dockerfile.base.j2 → out_path

            Args:
                out_path: Output file path
                ros_distro: ROS distribution (e.g., 'humble')
                ubuntu: Ubuntu release (e.g., 'jammy')
                common_pkgs: List of common packages
                workspace_dir: Workspace directory name
                apt_packages: System apt packages to install
                apt_mirror: Custom Ubuntu apt mirror URL (optional)
                ros_apt_mirror: Custom ROS apt mirror URL (optional)
            """
            self.render(
                "Dockerfile.base.j2",
                out_path,
                ros_distro   = ros_distro,
                ubuntu       = ubuntu,
                common_pkgs  = common_pkgs,
                workspace_dir= workspace_dir,
                apt_packages = apt_packages or [],
                apt_mirror   = apt_mirror,
                ros_apt_mirror = ros_apt_mirror
            )
