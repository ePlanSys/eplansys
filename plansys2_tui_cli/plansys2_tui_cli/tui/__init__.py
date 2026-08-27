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
Make the vendored textual importable before any module in here imports it.

Both steps have to happen before the first ``import textual`` anywhere in the
package, and doing them in a module that itself imports textual would force
that import below running code, which reads as an ordering mistake to the
linters. Here they are simply what importing the package does.
"""

import os
import sys

# Prefer the vendored textual shipped inside this package (no system pip
# needed).
_vendor_dir = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'vendor')
)
if os.path.isdir(_vendor_dir) and _vendor_dir not in sys.path:
    sys.path.insert(0, _vendor_dir)

from ._platformdirs_compat import patch as _patch_platformdirs  # noqa: E402
from ._typing_compat import patch as _patch_typing_extensions  # noqa: E402

# Jammy (and so Humble) ships a typing_extensions and a platformdirs older
# than the vendored textual expects; the names it imports from them are
# backfilled before textual is imported at all.
_patch_typing_extensions()
_patch_platformdirs()
