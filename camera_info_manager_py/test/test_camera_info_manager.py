# Copyright 2026, Open Source Robotics Foundation, Inc.
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
#    * Neither the name of the the copyright holder nor the names of its
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

import errno
from pathlib import Path
from typing import NoReturn

from ament_index_python import PackageNotFoundError
import pytest
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import yaml

from camera_info_manager import camera_info_manager as cim_module
from camera_info_manager import CameraInfoManager, CameraInfoMissingError

# Paths are passed around as_posix(), because file:// URLs and the directory creation in
# saveCalibrationFile() expect forward slashes, on Windows too. Errors that are reported
# differently by each OS are injected, instead of provoked using the file system.


def _make_camera_info(fx: float = 500.0) -> CameraInfo:
    ci = CameraInfo()
    ci.width = 640
    ci.height = 480
    ci.distortion_model = 'plumb_bob'
    ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    ci.k = [fx, 0.0, 320.0, 0.0, fx, 240.0, 0.0, 0.0, 1.0]
    ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, 320.0, 0.0, 0.0, fx, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return ci


def _raises_oserror(code: int) -> object:
    def raiser(*args: object, **kwargs: object) -> NoReturn:
        raise OSError(code, 'injected error')
    return raiser


@pytest.fixture
def package_share(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    # Resolves package://some_package/ to a temporary directory.
    def get_share_directory(package: str) -> str:
        if package == 'some_package':
            return tmp_path.as_posix()
        raise PackageNotFoundError(package)

    monkeypatch.setattr(cim_module, 'get_package_share_directory', get_share_directory)
    return tmp_path


def test_service_name_uses_namespace(node: Node) -> None:
    cim = CameraInfoManager(node, namespace='my_ns')
    assert cim.svc.srv_name.endswith('my_ns/set_camera_info')


def test_is_calibrated_raises_before_load(node: Node) -> None:
    with pytest.raises(CameraInfoMissingError):
        CameraInfoManager(node).isCalibrated()


def test_load_camera_info_from_package(node: Node, package_share: Path) -> None:
    cim_module.saveCalibrationFile(
        _make_camera_info(fx=777.0), (package_share / 'cam0.yaml').as_posix(), 'cam0')

    cim = CameraInfoManager(node, cname='cam0', url='package://some_package/cam0.yaml')
    cim.loadCameraInfo()
    assert cim.isCalibrated()
    assert cim.getCameraInfo().k[0] == 777.0


def test_load_camera_info_unknown_package_raises(node: Node, package_share: Path) -> None:
    cim = CameraInfoManager(node, url='package://no_such_package/cam0.yaml')
    with pytest.raises(CameraInfoMissingError):
        cim.loadCameraInfo()


def test_load_camera_info_invalid_url_is_uncalibrated(node: Node) -> None:
    cim = CameraInfoManager(node, url='html://ros.org/cam0.yaml')
    cim.loadCameraInfo()
    assert not cim.isCalibrated()


def test_load_camera_info_empty_url_uses_default(
    node: Node, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv('ROS_HOME', tmp_path.as_posix())
    cim_module.saveCalibrationFile(
        _make_camera_info(fx=777.0), (tmp_path / 'camera_info' / 'cam0.yaml').as_posix(), 'cam0')

    cim = CameraInfoManager(node, cname='cam0')
    cim.loadCameraInfo()
    assert cim.getCameraInfo().k[0] == 777.0


def test_set_camera_info_reports_save_failure(node: Node, tmp_path: Path) -> None:
    # Opening a directory for writing fails on every OS, with a different error on each.
    directory = tmp_path / 'directory.yaml'
    directory.mkdir()
    cim = CameraInfoManager(node, url=f'file://{directory.as_posix()}')

    request = SetCameraInfo.Request()
    request.camera_info = _make_camera_info(fx=777.0)
    response = cim.setCameraInfo(request, SetCameraInfo.Response())

    assert not response.success
    assert response.status_message == 'Error storing camera calibration.'
    assert cim.getCameraInfo().k[0] == 777.0


def test_set_camera_name_only_resets_camera_info_if_changed(node: Node) -> None:
    cim = CameraInfoManager(node, cname='cam0')
    cim.camera_info = _make_camera_info()

    assert cim.setCameraName('cam0')
    assert cim.camera_info is not None

    assert cim.setCameraName('cam1')
    assert cim.camera_info is None


def test_set_url(node: Node) -> None:
    cim = CameraInfoManager(node, url='file:///first.yaml')
    cim.camera_info = _make_camera_info()

    assert not cim.setURL('html://ros.org/cam0.yaml')
    assert cim.getURL() == 'file:///first.yaml'

    assert cim.setURL('file:///first.yaml')
    assert cim.camera_info is not None

    assert cim.setURL('file:///second.yaml')
    assert cim.getURL() == 'file:///second.yaml'
    assert cim.camera_info is None


def test_gen_camera_name() -> None:
    assert cim_module.genCameraName('') == '_'
    assert cim_module.genCameraName('cam_0') == 'cam_0'
    assert cim_module.genCameraName('my cam-0') == 'my_cam_0'


def test_get_package_file_name(package_share: Path) -> None:
    assert (
        cim_module.getPackageFileName('package://some_package/dir/cam0.yaml')
        == f'{package_share.as_posix()}/dir/cam0.yaml'
    )
    assert cim_module.getPackageFileName('package://no_such_package/cam0.yaml') == ''


def test_load_calibration_file_empty_or_missing_is_uncalibrated(tmp_path: Path) -> None:
    empty_file = tmp_path / 'empty.yaml'
    empty_file.write_text('')
    assert cim_module.loadCalibrationFile(empty_file.as_posix(), 'cam0').k[0] == 0.0
    assert cim_module.loadCalibrationFile((tmp_path / 'no.yaml').as_posix(), 'cam0').k[0] == 0.0


def test_load_calibration_file_with_other_camera_name(tmp_path: Path) -> None:
    yaml_file = (tmp_path / 'cam0.yaml').as_posix()
    cim_module.saveCalibrationFile(_make_camera_info(fx=777.0), yaml_file, 'cam0')
    assert cim_module.loadCalibrationFile(yaml_file, 'other_name').k[0] == 777.0


@pytest.mark.parametrize('url, expected', [
    ('', cim_module.URL_empty),
    ('file:///cam0.yaml', cim_module.URL_file),
    ('FILE://C:/cam0.yaml', cim_module.URL_file),
    ('package://some_package/cam0.yaml', cim_module.URL_package),
    ('PACKAGE://some_package/cam0.yaml', cim_module.URL_package),
    ('package://', cim_module.URL_invalid),
    ('package://some_package', cim_module.URL_invalid),
    ('package://some_package/', cim_module.URL_invalid),
    ('html://ros.org/cam0.yaml', cim_module.URL_invalid),
])
def test_parse_url(url: str, expected: int) -> None:
    assert cim_module.parseURL(url) == expected


def test_resolve_url_keeps_unknown_variables() -> None:
    assert cim_module.resolveURL('file:///$cam0.yaml', 'cam0') == 'file:///$cam0.yaml'
    assert cim_module.resolveURL('file:///${BAD}.yaml', 'cam0') == 'file:///${BAD}.yaml'


def test_resolve_url_ros_home(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv('ROS_HOME', '/my/ros/home')
    monkeypatch.setenv('HOME', '/my/home')
    assert cim_module.resolveURL('${ROS_HOME}/x', 'cam0') == '/my/ros/home/x'

    monkeypatch.delenv('ROS_HOME')
    assert cim_module.resolveURL('${ROS_HOME}/x', 'cam0') == '/my/home/.ros/x'

    monkeypatch.delenv('HOME')
    assert cim_module.resolveURL('${ROS_HOME}/x', 'cam0') == '${ROS_HOME}/x'


@pytest.mark.parametrize('url', [
    '',
    'html://ros.org/cam0.yaml',
    'package://no_such_package/cam0.yaml',
])
def test_save_calibration_falls_back_to_default(
    url: str, package_share: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv('ROS_HOME', package_share.as_posix())
    assert cim_module.saveCalibration(_make_camera_info(), url, 'cam0')
    assert (package_share / 'camera_info' / 'cam0.yaml').exists()


def test_save_calibration_to_package(package_share: Path) -> None:
    assert cim_module.saveCalibration(
        _make_camera_info(), 'package://some_package/cam0.yaml', 'cam0')
    assert (package_share / 'cam0.yaml').exists()


def test_save_calibration_file_creates_directories(tmp_path: Path) -> None:
    yaml_file = tmp_path / 'a' / 'b' / 'cam0.yaml'
    assert cim_module.saveCalibrationFile(_make_camera_info(), yaml_file.as_posix(), 'cam0')
    assert yaml_file.exists()


@pytest.mark.parametrize('code', [errno.EACCES, errno.EPERM, errno.ENOSPC])
def test_save_calibration_file_cannot_open(
    code: int, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(Path, 'open', _raises_oserror(code))
    assert not cim_module.saveCalibrationFile(_make_camera_info(), '/dir/cam0.yaml', 'cam0')


def test_save_calibration_file_name_without_slash(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(Path, 'open', _raises_oserror(errno.ENOENT))
    assert not cim_module.saveCalibrationFile(_make_camera_info(), 'cam0.yaml', 'cam0')


def test_save_calibration_file_cannot_create_directory(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(Path, 'open', _raises_oserror(errno.ENOENT))
    monkeypatch.setattr(Path, 'mkdir', _raises_oserror(errno.EACCES))
    assert not cim_module.saveCalibrationFile(_make_camera_info(), '/dir/cam0.yaml', 'cam0')


def test_save_calibration_file_cannot_write(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(yaml, 'safe_dump', _raises_oserror(errno.ENOSPC))
    # The directory exists
    assert not cim_module.saveCalibrationFile(
        _make_camera_info(), (tmp_path / 'cam0.yaml').as_posix(), 'cam0')
    # The directory has to be created first
    assert not cim_module.saveCalibrationFile(
        _make_camera_info(), (tmp_path / 'new' / 'cam0.yaml').as_posix(), 'cam0')
