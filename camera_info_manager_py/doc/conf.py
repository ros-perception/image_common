import inspect
import os
import re
import sys

# When rosdoc2 runs sphinx-build, it exec's this file from a generated wrapper.
# The wrapper contains: exec(open("/abs/path/to/doc/conf.py").read())
# Parse that line from the call stack to find the package root and add it to
# sys.path so that autodoc can import camera_info_manager.
try:
    import camera_info_manager  # noqa: F401
except ImportError:
    for _fi in inspect.stack():
        for _line in (_fi.code_context or []):
            _m = re.search(r'exec\(open\("([^"]+)"\)', _line)
            if _m:
                _pkg_root = os.path.dirname(os.path.dirname(_m.group(1)))
                if os.path.isdir(_pkg_root):
                    sys.path.insert(0, _pkg_root)
                break

project = 'camera_info_manager_py'
copyright = "2024, Open Robotics"
author = "Jack O'Quin, Jose Mastrangelo, Mike Hosmar"

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode',
    'myst_parser',
]

autodoc_member_order = 'bysource'

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
exclude_patterns = [
    '_build',
    'overview.rst',
    'api.rst',
]

html_theme = 'sphinx_rtd_theme'
