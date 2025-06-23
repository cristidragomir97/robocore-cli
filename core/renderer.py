# core/renderer.py
import os
from jinja2 import Environment, FileSystemLoader, StrictUndefined

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
                        comp_src: str,
                        repos_file: str,
                        superclient_path: str,
                        apt_packages,
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
            comp_src=comp_src,
            repos_file=repos_file,
            superclient_path=superclient_path,
            apt_packages=apt_packages,
            postinstall=postinstall,
            has_repos=has_repos,
            comp_src_exists=comp_src_exists,
            enable_apt_caching=enable_apt_caching,
            dds_server_ip=dds_server_ip
        )


    def render_superclient(self, out_path: str, *,
                        component_name: str,
                        participantID: int,
                        this_host_ip: str,
                        dds_server_host_ip: str):
        self.render(
            "superclient.xml.j2",
            out_path,
            component_name=component_name,
            participantID=participantID,
            this_host_ip=this_host_ip,
            dds_server_host_ip=dds_server_host_ip
        )

    def render_compose(self, out_path, components, cfg, dds_manager=None):
        self.render(
            "docker-compose.j2",
            out_path,
            components    = components,
            mount_root    = cfg.mount_root,
            ros_distro    = cfg.ros_distro,
            ros_domain_id = cfg.ros_domain_id, 
            enable_dds_router = cfg.enable_dds_router,
            discovery_server = cfg.discovery_server,
            dds_manager       = dds_manager
        )

    def render_base(self, out_path: str, ros_distro: str, ubuntu: str, common_pkgs, apt_packages = []):
            """
            Render Dockerfile.base.j2 → out_path
            """
            self.render(
                "Dockerfile.base.j2",
                out_path,
                ros_distro   = ros_distro,
                ubuntu       = ubuntu,
                common_pkgs  = common_pkgs,
                apt_packages = apt_packages or []
            )
