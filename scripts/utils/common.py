# utils/common.py

import os
import yaml
import subprocess
from jinja2 import Environment, FileSystemLoader, StrictUndefined
from dataclasses import dataclass

@dataclass
class Host:
    name: str
    ip: str
    user: str
    arch: str

def load_config() -> dict:
    cfg_path = os.path.join(os.getcwd(), 'config.yaml')
    if not os.path.isfile(cfg_path):
        raise FileNotFoundError(f"config.yaml not found in {os.getcwd()}")
    return yaml.safe_load(open(cfg_path))

def get_hosts(cfg: dict) -> list[Host]:
    hosts = []
    for h in cfg.get('hosts', []):
        hosts.append(Host(
            name=h['name'],
            ip=h['ip'],
            user=h.get('user', 'root'),
            arch=h['arch']
        ))
    return hosts




def render_template(template_path, out_path=None, **ctx):
    env = Environment(
        loader=FileSystemLoader(os.path.dirname(template_path)),
        undefined=StrictUndefined,
        trim_blocks=True,
        lstrip_blocks=True
    )
    tpl = env.get_template(os.path.basename(template_path))
    rendered = tpl.render(**ctx)

    if out_path:
        with open(out_path, 'w') as f:
            f.write(rendered)
    else:
        return rendered


def sh(cmd: str):
    print(f"$ {cmd}")
    res = subprocess.run(cmd, shell=True)
    if res.returncode != 0:
        raise RuntimeError(f"Command failed ({res.returncode}): {cmd}")
