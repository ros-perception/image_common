import os
import sys

# Allow Sphinx extensions to find local modules
sys.path.insert(0, os.path.abspath('.'))

# -- Project information (overridden by rosdoc2 from package.xml) ----------
project = 'image_transport'
copyright = '2024, Open Robotics'
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
