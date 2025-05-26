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

    def render_dockerfile(self, component, cfg):
        self.render(
            "Dockerfile.j2",
            os.path.join(component.folder, "Dockerfile"),
            ros_distro    = cfg.ros_distro,
            ros_domain_id = cfg.ros_domain_id,
            extra_apt     = cfg.extra_apt,
            common_pkgs   = cfg.common_packages,
            comp          = component,
            comp_src      = os.path.relpath(component.comp_src, cfg.root),
            has_repos     = bool(component.repositories),
            repos_file    = os.path.relpath(component.repos_file, cfg.root)
        )

    def render_compose(self, out_path, components, cfg):
        self.render(
            "docker-compose.j2",
            out_path,
            components    = components,
            mount_root    = cfg.mount_root,
            ros_distro    = cfg.ros_distro,
            ros_domain_id = cfg.ros_domain_id
        )

    def render_base(self, out_path: str, ros_distro: str, ubuntu: str, common_pkgs, apt_packages):
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
