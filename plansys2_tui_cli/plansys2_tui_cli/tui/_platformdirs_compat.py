# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Backfill the ``platformdirs`` names the vendored textual imports.

Jammy (and so Humble) ships ``platformdirs`` 2.5, which predates
``user_downloads_path``; importing the vendored textual there dies at import
time with ``cannot import name 'user_downloads_path' from 'platformdirs'``.
As with ``_typing_compat``, the missing name is patched onto whatever copy is
installed rather than vendoring a second one. The stand-in only has to answer
where a delivered file goes, which the TUI never asks for.
"""

import os
from pathlib import Path


def _user_downloads_path() -> Path:
    """
    Return the directory a downloaded file belongs in.

    Newer platformdirs reads ``~/.config/user-dirs.dirs``; this reads the
    environment the same file exports and otherwise falls back to the
    conventional location.
    """
    configured = os.environ.get('XDG_DOWNLOAD_DIR')
    return Path(configured).expanduser() if configured else Path.home() / 'Downloads'


def patch() -> None:
    """Add ``user_downloads_path`` to the installed platformdirs if it lacks it."""
    import platformdirs

    if not hasattr(platformdirs, 'user_downloads_path'):
        platformdirs.user_downloads_path = _user_downloads_path
