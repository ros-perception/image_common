# Copyright 2026 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
copyright = '2026, Open Robotics'  # noqa: A001
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
