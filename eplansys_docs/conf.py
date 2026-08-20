# Configuration file for the Sphinx documentation builder.
#
# The site is built from this directory and published at
# https://eplansys.github.io/eplansys by .github/workflows/docs.yaml.

project = 'ePlanSys'
copyright = '2026, Haniel Ulises Vasquez Morales'
author = 'Haniel Ulises Vasquez Morales'
release = '0.1.0'
version = '0.1.0'

extensions = [
    'sphinx.ext.graphviz',
    'sphinx.ext.mathjax',
    'sphinx.ext.todo',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'requirements.txt']

# Figures, tables and listings are numbered and cross-referenced by number.
numfig = True

# Unresolved documentation gaps are part of the text, not a hidden note.
todo_include_todos = True

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'collapse_navigation': False,
    'navigation_depth': 3,
    'titles_only': False,
}
html_static_path = []
html_logo = 'eplansys.svg'
html_show_sphinx = False
html_show_sourcelink = False

# "Edit on GitHub" links in the theme's breadcrumb.
html_context = {
    'display_github': True,
    'github_user': 'ePlanSys',
    'github_repo': 'eplansys',
    'github_version': 'rolling',
    'conf_py_path': '/eplansys_docs/',
}

graphviz_output_format = 'svg'

# Doxygen XML is produced by .github/workflows/doxygen-doc.yml, which publishes
# it separately rather than into this tree. Uncomment once that XML is written
# to a path this build can read, and add 'breathe' to requirements.txt.
#
# extensions.append('breathe')
# breathe_projects = {
#     'eplansys': '../docs/xml',
# }
# breathe_default_project = 'eplansys'
# breathe_default_members = ('members', 'undoc-members')
