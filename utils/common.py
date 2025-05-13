# utils/common.py

import os
import yaml
import subprocess
from jinja2 import Environment, FileSystemLoader, StrictUndefined
from dataclasses import dataclass
from dataclasses import dataclass
from typing import List, Dict, Any

@dataclass
class Host:
    name: str
    ip: str
    user: str
    arch: str

def load_config(path="config.yaml") -> Dict[str,Any]:
    import yaml
    return yaml.safe_load(open(path))

def get_hosts(cfg: Dict[str,Any]) -> List[Host]:
    """
    Return a list of Host objects.
     - If cfg['host'] is present, wrap it into a single‐element list.
     - Otherwise use cfg['hosts'] as a list.
    """
    if 'host' in cfg:
        return [Host(**cfg['host'])]
    if 'hosts' in cfg:
        return [Host(**h) for h in cfg['hosts']]
    raise KeyError("config.yaml must contain either 'host' or 'hosts'")



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
