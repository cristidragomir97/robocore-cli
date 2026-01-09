import os
from core.config import Config
from core.docker import DockerHelper

def viz_main(project_root: str, config_file: str = 'config.yaml'):
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    cfg = Config.load(project_root, config_file=config_file)
    docker = DockerHelper()

    image = f"tiryoh/ros2-desktop-vnc:{cfg.ros_distro}"
    container_name = f"robocore_desktop_{cfg.ros_distro}"

    print(f"[desktop] Launching local desktop container '{container_name}' with image '{image}'")

    docker.client.run(

        image       = image,
        name        = container_name,
        interactive = False,
        detach      = True,
        tty         = True,
        remove      = True,
        hostname    = container_name,
        #network_mode = "host",
        publish = [("6080", "80")],  # VNC port
        envs = {
            "ROS_DISCOVERY_SERVER": cfg.discovery_server,
            "ROS_DOMAIN_ID": str(cfg.ros_domain_id),
            "ROS_DISTRO": cfg.ros_distro
        },
        volumes = []  # You could mount logs or shared workspace here
    )

    print(f"[desktop] Open your browser to http://localhost:6080 to access RViz, rqt, etc.")
