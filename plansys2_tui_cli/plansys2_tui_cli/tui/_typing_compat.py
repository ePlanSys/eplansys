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
Backfill the ``typing_extensions`` names the vendored textual imports.

Jammy (and so Humble) ships ``typing_extensions`` 3.10, which predates
``Self``; importing the vendored textual there dies at import time with
``cannot import name 'Self' from 'typing_extensions'``. Rather than vendor a
second copy of ``typing_extensions``, patch the missing names onto whatever
copy is installed, taking them from ``typing`` when the running interpreter
has them and falling back to ``Any`` when it does not. Every name below is
only ever used in annotations, so ``Any`` is a faithful stand-in at runtime.
"""

import typing

# The names the vendored textual imports from typing_extensions.
_NAMES = (
    'Coroutine',
    'Final',
    'Literal',
    'ParamSpec',
    'Protocol',
    'Self',
    'TypeAlias',
    'TypedDict',
    'TypeGuard',
    'get_args',
    'runtime_checkable',
)


def patch() -> None:
    """Add any missing name to the installed ``typing_extensions``."""
    import typing_extensions

    for name in _NAMES:
        if hasattr(typing_extensions, name):
            continue
        setattr(typing_extensions, name, getattr(typing, name, typing.Any))
