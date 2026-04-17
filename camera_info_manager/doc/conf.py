import os
import sys

sys.path.insert(0, os.path.abspath('.'))

project = 'camera_info_manager'
copyright = '2024, Open Robotics'
author = 'Alejandro Hernandez Cordero, Geoffrey Biggs, Jack O\'Quin, Michael Carroll'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'breathe',
    'myst_parser',
]

# breathe_projects and breathe_default_project are overridden by rosdoc2 at build time.
breathe_default_project = 'camera_info_manager Doxygen Project'
breathe_default_members = ('members', 'undoc-members')

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
# Exclude root-level copies of user docs (rosdoc2 also copies them to user_docs/
# where they are covered by the user_docs.rst glob toctree).
exclude_patterns = [
    '_build',
    'overview.rst',
    'api.rst',
]

html_theme = 'sphinx_rtd_theme'
