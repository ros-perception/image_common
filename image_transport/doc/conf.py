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

import os
import sys

# Allow Sphinx extensions to find local modules
sys.path.insert(0, os.path.abspath('.'))

# -- Project information (overridden by rosdoc2 from package.xml) ----------
project = 'image_transport'
copyright = '2026, Open Robotics'  # noqa: A001
author = 'Alejandro Hernandez Cordero, Geoffrey Biggs'

# -- General configuration -------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'breathe',
    'myst_parser',
]

# breathe_projects and breathe_default_project are overridden by rosdoc2 at build time.
breathe_default_project = 'image_transport Doxygen Project'
breathe_default_members = ('members', 'undoc-members')

# myst_parser — allow .md files in toctrees
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
# Exclude root-level copies of user docs (rosdoc2 copies them to user_docs/ too,
# where they are included via the user_docs.rst glob toctree).
exclude_patterns = [
    '_build',
    'camera_api.rst',
    'filter_api.rst',
    'overview.rst',
    'plugin_api.rst',
    'user_api.rst',
]

# -- HTML output options ---------------------------------------------------
html_theme = 'sphinx_rtd_theme'
