"""
Unit tests for configuration validation module.

Tests the Pydantic schemas and ConfigValidator functionality.
"""

import os
import pytest
import tempfile
import yaml
from pathlib import Path

from core.validation import (
    ConfigValidator,
    RobocoreConfig,
    ComponentConfig,
    HostConfig,
    CommonPackageConfig,
    Architecture,
)
from core.exceptions import (
    ConfigurationError,
    SourceNotFoundError,
    ValidationError,
)


class TestHostConfig:
    """Test host configuration validation."""

    def test_valid_host_config(self):
        """Test valid host configuration."""
        host = HostConfig(
            name="test-host",
            ip="192.168.1.100",
            user="testuser",
            arch=Architecture.ARM64,
        )
        assert host.name == "test-host"
        assert host.port == 2375  # Default port
        assert host.manager is False  # Default manager

    def test_host_with_custom_port(self):
        """Test host with custom Docker port."""
        host = HostConfig(
            name="test-host",
            ip="192.168.1.100",
            user="testuser",
            arch=Architecture.ARM64,
            port=2376,
        )
        assert host.port == 2376

    def test_invalid_port_number(self):
        """Test that invalid port numbers are rejected."""
        with pytest.raises(ValueError, match="Port must be between"):
            HostConfig(
                name="test-host",
                ip="192.168.1.100",
                user="testuser",
                arch=Architecture.ARM64,
                port=70000,  # Invalid port
            )

    def test_manager_flag(self):
        """Test DDS manager flag."""
        host = HostConfig(
            name="test-host",
            ip="192.168.1.100",
            user="testuser",
            arch=Architecture.ARM64,
            manager=True,
        )
        assert host.manager is True

    def test_dds_ip_field(self):
        """Test DDS IP field for separate DDS network."""
        host = HostConfig(
            name="test-host",
            ip="192.168.1.100",
            user="testuser",
            arch=Architecture.ARM64,
            dds_ip="10.0.0.100",
        )
        assert host.dds_ip == "10.0.0.100"

    def test_dds_ip_optional(self):
        """Test that dds_ip is optional and defaults to None."""
        host = HostConfig(
            name="test-host",
            ip="192.168.1.100",
            user="testuser",
            arch=Architecture.ARM64,
        )
        assert host.dds_ip is None

    def test_mount_root_field(self):
        """Test mount_root field for per-host mount paths."""
        host = HostConfig(
            name="test-host",
            ip="192.168.1.100",
            user="testuser",
            arch=Architecture.ARM64,
            mount_root="/custom/mount/path",
        )
        assert host.mount_root == "/custom/mount/path"

    def test_mount_root_optional(self):
        """Test that mount_root is optional and defaults to None."""
        host = HostConfig(
            name="test-host",
            ip="192.168.1.100",
            user="testuser",
            arch=Architecture.ARM64,
        )
        assert host.mount_root is None


class TestComponentConfig:
    """Test component configuration validation."""

    def test_component_with_sources(self):
        """Test component with sources field."""
        comp = ComponentConfig(
            name="test-comp",
            sources=["src/package1", "src/package2"],
            runs_on="test-host",
        )
        assert comp.name == "test-comp"
        assert len(comp.sources) == 2

    def test_component_with_single_source(self):
        """Test component with source field."""
        comp = ComponentConfig(
            name="test-comp",
            source="src/package",
            runs_on="test-host",
        )
        assert comp.name == "test-comp"
        assert comp.source == ["src/package"]  # Should be normalized to list

    def test_component_with_repositories_only(self):
        """Test component with only repositories (no local sources)."""
        comp = ComponentConfig(
            name="test-comp",
            repositories=[
                {"url": "https://github.com/user/repo.git", "version": "main"}
            ],
            runs_on="test-host",
        )
        assert comp.name == "test-comp"
        assert len(comp.repositories) == 1

    def test_component_missing_runs_on(self):
        """Test that non-simulated components require runs_on."""
        with pytest.raises(ValueError, match="must specify 'runs_on'"):
            ComponentConfig(
                name="test-comp",
                source="src/package",
            )

    def test_simulated_component_without_runs_on(self):
        """Test that simulated components don't require runs_on."""
        comp = ComponentConfig(
            name="test-comp",
            source="src/package",
            simulate=True,
        )
        assert comp.simulate is True
        assert comp.runs_on is None

    def test_component_without_sources_or_repos(self):
        """Test that component must have sources or repositories."""
        with pytest.raises(ValueError, match="must specify one of"):
            ComponentConfig(
                name="test-comp",
                runs_on="test-host",
            )

    def test_component_with_multiple_source_fields(self):
        """Test that only one source configuration method is allowed."""
        with pytest.raises(ValueError, match="can only specify one of"):
            ComponentConfig(
                name="test-comp",
                sources=["src/package1"],
                source="src/package2",
                runs_on="test-host",
            )


class TestCommonPackageConfig:
    """Test common package configuration validation."""

    def test_common_package_with_repositories(self):
        """Test common package with VCS repositories."""
        pkg = CommonPackageConfig(
            name="common-msgs",
            repositories=[
                {"url": "https://github.com/user/msgs.git", "version": "main"}
            ],
        )
        assert pkg.is_vcs_based is True
        assert pkg.is_local_based is False

    def test_common_package_with_local_source(self):
        """Test common package with local sources."""
        pkg = CommonPackageConfig(
            name="common-pkg",
            source=["src/package"],
        )
        assert pkg.is_vcs_based is False
        assert pkg.is_local_based is True

    def test_common_package_without_sources(self):
        """Test that common package must specify sources or repositories."""
        with pytest.raises(ValueError, match="must specify either"):
            CommonPackageConfig(name="common-pkg")


class TestRobocoreConfig:
    """Test root configuration validation."""

    def test_minimal_valid_config(self):
        """Test minimal valid configuration."""
        config = RobocoreConfig(
            ros_distro="humble",
            ros_domain_id=0,
            registry="docker.io/myregistry",
            image_prefix="myproject",
            hosts=[
                {
                    "name": "host1",
                    "ip": "192.168.1.100",
                    "user": "user",
                    "arch": "arm64",
                }
            ],
            components=[
                {
                    "name": "comp1",
                    "source": "src/package",
                    "runs_on": "host1",
                }
            ],
        )
        assert config.ros_distro == "humble"
        assert len(config.hosts) == 1
        assert len(config.components) == 1

    def test_duplicate_component_names(self):
        """Test that duplicate component names are rejected."""
        with pytest.raises(ValueError, match="must be unique"):
            RobocoreConfig(
                ros_distro="humble",
                ros_domain_id=0,
                registry="docker.io/myregistry",
                image_prefix="myproject",
                hosts=[
                    {
                        "name": "host1",
                        "ip": "192.168.1.100",
                        "user": "user",
                        "arch": "arm64",
                    }
                ],
                components=[
                    {"name": "comp1", "source": "src/pkg1", "runs_on": "host1"},
                    {"name": "comp1", "source": "src/pkg2", "runs_on": "host1"},
                ],
            )

    def test_duplicate_host_names(self):
        """Test that duplicate host names are rejected."""
        with pytest.raises(ValueError, match="must be unique"):
            RobocoreConfig(
                ros_distro="humble",
                ros_domain_id=0,
                registry="docker.io/myregistry",
                image_prefix="myproject",
                hosts=[
                    {
                        "name": "host1",
                        "ip": "192.168.1.100",
                        "user": "user",
                        "arch": "arm64",
                    },
                    {
                        "name": "host1",
                        "ip": "192.168.1.101",
                        "user": "user",
                        "arch": "amd64",
                    },
                ],
            )

    def test_invalid_host_reference(self):
        """Test that component must reference valid host."""
        with pytest.raises(ValueError, match="references unknown host"):
            RobocoreConfig(
                ros_distro="humble",
                ros_domain_id=0,
                registry="docker.io/myregistry",
                image_prefix="myproject",
                hosts=[
                    {
                        "name": "host1",
                        "ip": "192.168.1.100",
                        "user": "user",
                        "arch": "arm64",
                    }
                ],
                components=[
                    {
                        "name": "comp1",
                        "source": "src/package",
                        "runs_on": "nonexistent-host",
                    }
                ],
            )

    def test_invalid_ros_domain_id(self):
        """Test that ROS domain ID must be in valid range."""
        with pytest.raises(ValueError):
            RobocoreConfig(
                ros_distro="humble",
                ros_domain_id=300,  # Invalid, must be 0-232
                registry="docker.io/myregistry",
                image_prefix="myproject",
            )


class TestConfigValidator:
    """Test ConfigValidator functionality."""

    @pytest.fixture
    def temp_project(self):
        """Create a temporary project directory with config file."""
        with tempfile.TemporaryDirectory() as tmpdir:
            project_root = Path(tmpdir)

            # Create source directories
            (project_root / "src" / "package1").mkdir(parents=True)
            (project_root / "src" / "package2").mkdir(parents=True)

            yield project_root

    def test_validate_valid_config_file(self, temp_project):
        """Test validation of valid config file."""
        config_data = {
            "ros_distro": "humble",
            "ros_domain_id": 0,
            "registry": "docker.io/test",
            "image_prefix": "test",
            "hosts": [
                {
                    "name": "host1",
                    "ip": "192.168.1.100",
                    "user": "user",
                    "arch": "arm64",
                }
            ],
            "components": [
                {"name": "comp1", "source": "src/package1", "runs_on": "host1"}
            ],
        }

        config_path = temp_project / "config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config_data, f)

        validator = ConfigValidator(str(temp_project))
        config, warnings = validator.validate_all(str(config_path))

        assert config.ros_distro == "humble"
        assert len(config.components) == 1

    def test_validate_missing_config_file(self, temp_project):
        """Test validation with missing config file."""
        validator = ConfigValidator(str(temp_project))

        with pytest.raises(FileNotFoundError):
            validator.validate_all(str(temp_project / "nonexistent.yaml"))

    def test_validate_invalid_yaml(self, temp_project):
        """Test validation with invalid YAML."""
        config_path = temp_project / "config.yaml"
        with open(config_path, "w") as f:
            f.write("invalid: yaml: syntax: ][")

        validator = ConfigValidator(str(temp_project))

        with pytest.raises(ConfigurationError, match="Failed to parse YAML"):
            validator.validate_all(str(config_path))

    def test_validate_missing_source_path(self, temp_project):
        """Test validation detects missing source paths."""
        config_data = {
            "ros_distro": "humble",
            "ros_domain_id": 0,
            "registry": "docker.io/test",
            "image_prefix": "test",
            "hosts": [
                {
                    "name": "host1",
                    "ip": "192.168.1.100",
                    "user": "user",
                    "arch": "arm64",
                }
            ],
            "components": [
                {
                    "name": "comp1",
                    "source": "src/nonexistent",  # Doesn't exist
                    "runs_on": "host1",
                }
            ],
        }

        config_path = temp_project / "config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config_data, f)

        validator = ConfigValidator(str(temp_project))

        with pytest.raises(ConfigurationError, match="validation failed"):
            validator.validate_all(str(config_path))

    def test_validate_deprecated_folder_field(self, temp_project):
        """Test validation warns about deprecated folder field."""
        # Create folder structure
        (temp_project / "components" / "comp1" / "ros_ws").mkdir(parents=True)

        config_data = {
            "ros_distro": "humble",
            "ros_domain_id": 0,
            "registry": "docker.io/test",
            "image_prefix": "test",
            "hosts": [
                {
                    "name": "host1",
                    "ip": "192.168.1.100",
                    "user": "user",
                    "arch": "arm64",
                }
            ],
            "components": [
                {
                    "name": "comp1",
                    "folder": "components/comp1",  # Deprecated
                    "runs_on": "host1",
                }
            ],
        }

        config_path = temp_project / "config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config_data, f)

        validator = ConfigValidator(str(temp_project))
        config, warnings = validator.validate_all(str(config_path))

        assert len(warnings) > 0
        assert any("deprecated" in w.lower() for w in warnings)


class TestEdgeCases:
    """Test edge cases and error conditions."""

    def test_unknown_field_in_component(self):
        """Test that unknown fields in components are rejected."""
        with pytest.raises(ValueError, match="Extra inputs are not permitted"):
            ComponentConfig(
                name="test-comp",
                source="src/package",
                runs_on="host1",
                unknown_field="value"  # Unknown field
            )

    def test_unknown_field_in_config(self):
        """Test that unknown fields in root config are rejected."""
        with pytest.raises(ValueError, match="Extra inputs are not permitted"):
            RobocoreConfig(
                ros_distro="humble",
                ros_domain_id=0,
                registry="docker.io/test",
                image_prefix="test",
                unknown_field="value"  # Unknown field
            )

    def test_component_with_pip_and_apt_packages(self):
        """Test component with both pip and apt packages."""
        comp = ComponentConfig(
            name="test-comp",
            source="src/package",
            runs_on="host1",
            apt_packages=["build-essential", "git"],
            pip_packages=["numpy", "scipy"],
        )
        assert len(comp.apt_packages) == 2
        assert len(comp.pip_packages) == 2

    def test_component_with_optimisations(self):
        """Test component with performance optimisations."""
        comp = ComponentConfig(
            name="test-comp",
            source="src/package",
            runs_on="host1",
            optimisations=[
                {"shm": "512m"},
                {"ipc": "host"},
                {"pinned_cores": [0, 1]},
            ],
        )
        assert len(comp.optimisations) == 3

    def test_empty_components_list(self):
        """Test config with no components."""
        config = RobocoreConfig(
            ros_distro="humble",
            ros_domain_id=0,
            registry="docker.io/test",
            image_prefix="test",
            components=[],
        )
        assert len(config.components) == 0

    def test_config_with_common_packages(self):
        """Test config with common packages."""
        config = RobocoreConfig(
            ros_distro="humble",
            ros_domain_id=0,
            registry="docker.io/test",
            image_prefix="test",
            common_packages=[
                {
                    "name": "common-msgs",
                    "repositories": [
                        {"url": "https://github.com/user/msgs.git", "version": "main"}
                    ],
                }
            ],
        )
        assert len(config.common_packages) == 1
        assert config.common_packages[0].is_vcs_based is True
