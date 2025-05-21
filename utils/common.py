# utils/common.py

import os
import yaml
import subprocess
from jinja2 import Environment, FileSystemLoader, StrictUndefined
from dataclasses import dataclass
from typing import List, Dict, Any, Union

@dataclass
class Host:
    name: str
    ip: str
    user: str
    arch: str

@dataclass
class Component:
    name: str
    folder: str
    entrypoint: str
    launch_args: str = ""
    preinstall: List[Union[str,Dict[str,str]]] = None
    postinstall: List[str] = None
    devices: List[str] = None
    ports: List[str] = None

def load_config(path="config.yaml") -> Dict[str,Any]:
    return yaml.safe_load(open(path))

def get_hosts(cfg: Dict[str,Any]) -> List[Host]:
    if 'host' in cfg:
        return [Host(**cfg['host'])]
    if 'hosts' in cfg:
        return [Host(**h) for h in cfg['hosts']]
    raise KeyError("config.yaml must contain 'host' or 'hosts'")

def get_components(cfg: Dict[str,Any]) -> List[Component]:
    comps: List[Component] = []
    for c in cfg.get('components', []):
        comps.append(
            Component(
                name        = c['name'],
                folder      = c['folder'],
                entrypoint  = c['entrypoint'],
                launch_args = c.get('launch_args',''),
                preinstall  = c.get('preinstall',[]),
                postinstall = c.get('postinstall',[]),
                devices     = c.get('devices',[]),
                ports       = c.get('ports',[])
            )
        )
    return comps

def render_template(template_path: str, out_path: str=None, **ctx):
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
        return None
    return rendered

def sh(cmd: str):
    print(f"$ {cmd}")
    res = subprocess.run(cmd, shell=True)
    if res.returncode:
        raise RuntimeError(f"Command failed ({res.returncode}): {cmd}")
