"""Run ament_flake8 (the ROS 2 Python style checker) over the package."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Fail if any style errors are found."""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, f'Found {len(errors)} code style errors / warnings:\n' + '\n'.join(errors)
