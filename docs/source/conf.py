import os
import sys

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

sys.path.insert(0, os.path.abspath('../..'))
sys.path.insert(0, os.path.abspath('../'))


project = 'Post-Earthquake Search and Rescue Robot'
copyright = '2025, teamG'
author = 'teamG'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration


# -- General configuration
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'myst_parser',  # if using Markdown
]


# -- Options for HTML output
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_context = {
    "display_github": True, # Integrate GitHub
    "github_user": "waleedelfieky",
    "github_repo": "CogArch-SAR-TIAGo",
    "github_version": "main",  # branch
    "conf_py_path": "/docs/source/",  # path in the repo to the docs root
}


templates_path = ['_templates']
exclude_patterns = []
