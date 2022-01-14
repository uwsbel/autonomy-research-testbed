# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import miniav
from datetime import date
import os
import sys

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, root_path)
sys.setrecursionlimit(1500)

# -- Project information -----------------------------------------------------

project = "MiniAV Project"
copyright = f"{date.today().year}, Simulation Based Engineering Laboratory"
author = "Simulation Based Engineering Laboratory"

# The full version, including alpha/beta/rc tags
from importlib.metadata import version as get_version
release = get_version("miniav")

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    # "rinoh.frontend.sphinx",
    "sphinx.ext.napoleon",
    "sphinx.ext.autodoc",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "sphinx.ext.githubpages",
    "autoapi.extension",
    "myst_parser",
    "sphinxarg.ext",
]

# autoapi config
autoapi_type = "python"
autoapi_dirs = [".."]
autoapi_options = [
    "members",
    "show-inheritance",
    "show-module-summary",
    "special-members",
    "imported-members",
    # "inherited-members"  # Doesn't work with viewcode extension
]
autoapi_ignore = ["*_import*"]
autoapi_keep_files = False
autoapi_generate_api_docs = True
autoapi_add_toctree_entry = True
# autoapi_keep_files = True
# autoapi_template_dir = "_templates"
# autoapi_member_order = "groupwise"

# suppress_warnings = ["autoapi"]

add_module_names = False

# Napoleon
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_use_ivar = True
napoleon_use_param = False
napoleon_use_rtype = False
napoleon_type_aliases = None
napoleon_attr_annotations = True


from sphinx.ext import autodoc

class SimpleDocumenter(autodoc.MethodDocumenter):
    objtype = "simple"

    #do not indent the content
    content_indent = ""

    #do not add a header to the docstring
    def add_directive_header(self, sig):
        pass


def setup(app):
    app.add_autodocumenter(SimpleDocumenter)


viewcode_enable_epub = True

# Display todos by setting to True
todo_include_todos = True

# Add any paths that contain templates here, relative to this directory.
# templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = "friendly"

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "furo"
html_favicon = "_static/cropped-SBEL-32x32.png"
html_theme_options = {
    "announcement": """
        <a style=\"text-decoration: none; color: white;\" 
           href=\"https://sbel.wisc.edu\">
           <img src=\"https://github.com/uwsbel/miniav/blob/master/docs/_static/cropped-SBEL-192x192.png?raw=true\" 
                style=\"vertical-align: middle; display: inline; padding-right: 7.5px; height: 20px;\"/>
           Checkout our website!
        </a>
    """,
    "sidebar_hide_name": True,
    "light_logo": "SBEL-light.png",
    "dark_logo": "SBEL-dark.png",
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

# These paths are either relative to html_static_path
# or fully qualified paths (eg. https://...)
html_css_files = [
    "css/custom.css",
]
