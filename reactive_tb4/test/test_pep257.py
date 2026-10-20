"""Run ament_pep257 (docstring style) over the package."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Fail if any docstring style errors are found."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
