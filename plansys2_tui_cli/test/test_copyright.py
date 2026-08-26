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
Lint this package's own code, and not the Textual copy it vendors.

plansys2_tui_cli/vendor/textual is another project's source, carried here so
the front end runs without a pip install into the ROS environment. It is 247
files against this package's fifteen, and holding it to this repository's
conventions would report thousands of divergences that must not be acted on:
the copy is deliberately unmodified so it can be replaced wholesale. See the
NOTICE file.
"""

import os
import subprocess
import sys

import pytest

# The package's own sources. Named positively rather than as an exclusion,
# because ament_copyright takes paths and not exclude patterns.
OWN_CODE = [
    'plansys2_tui_cli/__init__.py',
    'plansys2_tui_cli/cli',
    'plansys2_tui_cli/controller',
    'plansys2_tui_cli/tui',
    'setup.py',
    'test',
]

VENDORED = 'plansys2_tui_cli/vendor'


def _run(module, *args):
    env = os.environ.copy()
    env['FLAKE8_JOBS'] = '1'
    result = subprocess.run(
        [sys.executable, '-m', module, *args],
        env=env,
        capture_output=True,
        text=True,
    )
    return result


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    result = _run('ament_copyright.main', *OWN_CODE)
    assert result.returncode == 0, (
        'ament_copyright failed.\n'
        f'STDOUT:\n{result.stdout}\n'
        f'STDERR:\n{result.stderr}\n'
    )
