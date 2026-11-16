# Copyright (c) 2009, Willow Garage, Inc.
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

"""End-to-end test for the ``convert`` calibration-format executable."""

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
import pytest
import yaml

THIS_DIR = os.path.dirname(os.path.realpath(__file__))


def _find_convert():
    """Locate the camera_calibration_parsers ``convert`` executable."""
    # Preferred: absolute path injected by CMake at test time.
    env_path = os.environ.get('CONVERT_EXECUTABLE')
    if env_path and os.path.isfile(env_path):
        return env_path

    # Fallback: resolve it from the installed package prefix.
    try:
        candidate = os.path.join(
            get_package_prefix('camera_calibration_parsers'),
            'lib', 'camera_calibration_parsers', 'convert',
        )
        if os.path.isfile(candidate):
            return candidate
    except PackageNotFoundError:
        pass

    # Last resort: search the PATH.
    return shutil.which('convert')


def _run_convert(*args):
    """Run ``convert`` with the given arguments."""
    convert = _find_convert()
    assert convert is not None, 'convert executable not found'

    return subprocess.run(
        [convert, *args],
        capture_output=True,
        text=True,
    )


def _convert(ini_name, tmp_path):
    """Convert a fixture ``.ini`` to ``.yaml`` and return the parsed YAML."""
    ini_path = os.path.join(THIS_DIR, ini_name)
    yaml_path = os.path.join(str(tmp_path), 'out.yaml')
    result = _run_convert(ini_path, yaml_path)
    assert result.returncode == 0, result.stderr
    assert os.path.isfile(yaml_path)

    with open(yaml_path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_convert_calib5_ini(tmp_path):
    """Convert the plumb_bob fixture and check the recovered calibration."""
    calib = _convert('calib5.ini', tmp_path)
    assert calib['camera_name'] == 'mono_left'
    assert calib['image_width'] == 640
    assert calib['image_height'] == 480
    assert calib['distortion_model'] == 'plumb_bob'
    assert calib['projection_matrix']['data'][0] == pytest.approx(262.927429)


def test_convert_calib8_ini(tmp_path):
    """Convert the rational_polynomial fixture and check the distortion model."""
    calib = _convert('calib8.ini', tmp_path)
    assert calib['camera_name'] == 'mono_left'
    assert calib['image_width'] == 640
    assert calib['image_height'] == 480
    assert calib['distortion_model'] == 'rational_polynomial'


def test_convert_yaml_to_ini(tmp_path):
    """Convert a ``.yaml`` back to ``.ini``, keeping the camera name."""
    _convert('calib5.ini', tmp_path)
    ini_path = os.path.join(str(tmp_path), 'out.ini')
    result = _run_convert(os.path.join(str(tmp_path), 'out.yaml'), ini_path)
    assert result.returncode == 0, result.stderr
    with open(ini_path, encoding='utf-8') as f:
        assert '[mono_left]' in f.read()


def test_convert_without_arguments_prints_usage():
    """Print the usage when the file names are missing."""
    result = _run_convert()
    assert result.returncode == 0
    assert 'Usage' in result.stdout


def test_convert_missing_input(tmp_path):
    """Fail when the input file does not exist."""
    yaml_path = os.path.join(str(tmp_path), 'out.yaml')
    result = _run_convert(os.path.join(str(tmp_path), 'missing.ini'), yaml_path)
    assert result.returncode != 0
    assert not os.path.exists(yaml_path)


def test_convert_unknown_output_format(tmp_path):
    """Fail when the extension of the output file is unknown."""
    output_path = os.path.join(str(tmp_path), 'out.txt')
    result = _run_convert(os.path.join(THIS_DIR, 'calib5.ini'), output_path)
    assert result.returncode != 0
    assert not os.path.exists(output_path)


def test_convert_output_format_cannot_store_calibration(tmp_path):
    """Fail when saving rational_polynomial distortion as ``.ini``."""
    output_path = os.path.join(str(tmp_path), 'out.ini')
    result = _run_convert(os.path.join(THIS_DIR, 'calib8.ini'), output_path)
    assert result.returncode != 0
