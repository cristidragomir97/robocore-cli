"""
Custom exception hierarchy for robocore-cli.

Replaces sys.exit() calls with proper exceptions for better error handling and cleanup.
"""

class RobocoreError(Exception):
    """Base exception for all robocore-cli errors."""
    def __init__(self, message: str, context: dict = None):
        super().__init__(message)
        self.message = message
        self.context = context or {}

    def __str__(self):
        return self.message


class ConfigurationError(RobocoreError):
    """Raised when configuration validation fails."""
    pass


class ValidationError(RobocoreError):
    """Raised when validation checks fail."""
    pass


class WorkspaceError(RobocoreError):
    """Raised when workspace operations fail."""
    pass


class SourceNotFoundError(RobocoreError):
    """Raised when source paths don't exist."""
    pass


class DockerError(RobocoreError):
    """Raised when Docker operations fail."""
    pass


class HostError(RobocoreError):
    """Raised when host-related operations fail."""
    pass


class BuildError(RobocoreError):
    """Raised when build operations fail."""
    pass


class DeployError(RobocoreError):
    """Raised when deployment operations fail."""
    pass
