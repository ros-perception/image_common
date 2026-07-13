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

from collections.abc import Iterator
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

from camera_info_manager import (
    ApproximateZoomCameraInfoManager,
    CameraInfoError,
    CameraInfoManager,
    CameraInfoMissingError,
    InterpolatingZoomCameraInfoManager,
    loadCalibrationFile,
    saveCalibrationFile,
    ZoomCameraInfoManager,
)


@pytest.fixture(scope='module', autouse=True)
def rclpy_context() -> Iterator[None]:
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node() -> Iterator[Node]:
    n = Node('test_camera_info_manager')
    yield n
    n.destroy_node()


def test_camera_info_manager_constructs(node: Node) -> None:
    cim = CameraInfoManager(node, cname='cam0', url='', namespace='')
    assert cim.getCameraName() == 'cam0'
    assert cim.getURL() == ''


def test_camera_info_manager_str_does_not_reference_undefined_attrs(node: Node) -> None:
    cim = CameraInfoManager(node, cname='cam0', url='file:///tmp/x.yaml')
    assert str(cim) == '[cam0]file:///tmp/x.yaml'


def test_get_camera_info_raises_before_load(node: Node) -> None:
    cim = CameraInfoManager(node, cname='cam0')
    with pytest.raises(CameraInfoMissingError):
        cim.getCameraInfo()


def test_set_camera_name_validation(node: Node) -> None:
    cim = CameraInfoManager(node, cname='cam0')
    assert cim.setCameraName('new_name') is True
    assert cim.getCameraName() == 'new_name'
    assert cim.setCameraName('') is False
    assert cim.setCameraName('bad name') is False


def test_zoom_manager_constructs_with_node(node: Node) -> None:
    z = ZoomCameraInfoManager(node, min_zoom=0, max_zoom=10, cname='zoom0')
    assert z.getCameraName() == 'zoom0'
    assert z._min_zoom == 0
    assert z._max_zoom == 10
    assert z._zoom == 0


def test_zoom_manager_set_zoom_in_range(node: Node) -> None:
    z = ApproximateZoomCameraInfoManager(
        node, min_fov=30.0, max_fov=70.0,
        initial_image_width=640, initial_image_height=480,
        min_zoom=0, max_zoom=100, cname='zoom0',
    )
    z.set_zoom(50)
    assert z._zoom == 50


def test_zoom_manager_set_zoom_out_of_range_raises(node: Node) -> None:
    z = ApproximateZoomCameraInfoManager(
        node, min_fov=30.0, max_fov=70.0,
        initial_image_width=640, initial_image_height=480,
        min_zoom=0, max_zoom=100, cname='zoom0',
    )
    with pytest.raises(CameraInfoError):
        z.set_zoom(101)
    with pytest.raises(CameraInfoError):
        z.set_zoom(-1)


def test_approximate_zoom_updates_k_without_calibration(node: Node) -> None:
    z = ApproximateZoomCameraInfoManager(
        node, min_fov=30.0, max_fov=70.0,
        initial_image_width=640, initial_image_height=480,
        min_zoom=0, max_zoom=100, cname='zoom0',
    )
    z.loadCameraInfo()
    z.set_zoom(50)
    info = z.getCameraInfo()
    assert info.width == 640
    assert info.height == 480
    assert info.k[0] > 0.0
    assert info.k[4] > 0.0
    assert info.k[2] == 320.0
    assert info.k[5] == 240.0


def test_approximate_zoom_set_resolution(node: Node) -> None:
    z = ApproximateZoomCameraInfoManager(
        node, min_fov=30.0, max_fov=70.0,
        initial_image_width=640, initial_image_height=480,
        min_zoom=0, max_zoom=100, cname='zoom0',
    )
    z.loadCameraInfo()
    z.set_resolution(1280, 720)
    info = z.getCameraInfo()
    assert info.width == 1280
    assert info.height == 720


def test_interpolating_zoom_constructs(node: Node) -> None:
    iz = InterpolatingZoomCameraInfoManager(
        node, calibration_url_template='file:///tmp/cal_%d.yaml',
        zoom_levels=[0, 50, 100], cname='zoom0',
    )
    assert iz._min_zoom == 0
    assert iz._max_zoom == 100
    assert iz._calibration_url_template == 'file:///tmp/cal_%d.yaml'


def test_zoom_manager_context_exit_does_not_raise(node: Node) -> None:
    z = ZoomCameraInfoManager(node, min_zoom=0, max_zoom=10, cname='zoom0')
    with z:
        pass


def _make_camera_info(
    width: int = 640,
    height: int = 480,
    fx: float = 500.0,
    fy: float = 500.0,
    cx: float = 320.0,
    cy: float = 240.0,
) -> CameraInfo:
    ci = CameraInfo()
    ci.width = width
    ci.height = height
    ci.distortion_model = 'plumb_bob'
    ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return ci


def test_yaml_round_trip(tmp_path: Path) -> None:
    cname = 'cam0'
    src = _make_camera_info(width=1024, height=768, fx=900.0, fy=901.0)
    yaml_file = tmp_path / 'cam0.yaml'
    assert saveCalibrationFile(src, str(yaml_file), cname) is True

    loaded = loadCalibrationFile(str(yaml_file), cname)
    assert loaded.width == src.width
    assert loaded.height == src.height
    assert loaded.distortion_model == src.distortion_model
    assert list(loaded.d) == list(src.d)
    assert list(loaded.k) == list(src.k)
    assert list(loaded.r) == list(src.r)
    assert list(loaded.p) == list(src.p)


def test_camera_info_manager_loads_from_file(node: Node, tmp_path: Path) -> None:
    cname = 'cam0'
    src = _make_camera_info(fx=777.0)
    yaml_file = tmp_path / 'cam0.yaml'
    saveCalibrationFile(src, str(yaml_file), cname)

    cim = CameraInfoManager(node, cname=cname, url=f'file://{yaml_file}')
    cim.loadCameraInfo()
    assert cim.isCalibrated()
    info = cim.getCameraInfo()
    assert info.k[0] == 777.0


def test_set_camera_info_service_round_trip(node: Node, tmp_path: Path) -> None:
    cname = 'set_svc_cam'
    yaml_file = tmp_path / f'{cname}.yaml'
    cim = CameraInfoManager(node, cname=cname, url=f'file://{yaml_file}')

    client = node.create_client(SetCameraInfo, 'set_camera_info')
    assert client.wait_for_service(timeout_sec=5.0)

    req = SetCameraInfo.Request()
    req.camera_info = _make_camera_info(fx=1234.0, fy=1234.0)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    assert future.done()
    rsp = future.result()
    assert rsp is not None
    assert rsp.success
    assert yaml_file.exists()

    # Service callback updates the in-memory CameraInfo too
    info = cim.getCameraInfo()
    assert info.k[0] == 1234.0


def test_interpolating_zoom_interpolates_k(node: Node, tmp_path: Path) -> None:
    cname = 'izoom'
    # Calibrate at zoom 0 (fx=400) and zoom 100 (fx=800)
    low = _make_camera_info(fx=400.0, fy=400.0)
    high = _make_camera_info(fx=800.0, fy=800.0)
    saveCalibrationFile(low, str(tmp_path / 'cal_0.yaml'), cname)
    saveCalibrationFile(high, str(tmp_path / 'cal_100.yaml'), cname)

    template = f'file://{tmp_path}/cal_%d.yaml'
    iz = InterpolatingZoomCameraInfoManager(
        node, calibration_url_template=template, zoom_levels=[0, 100], cname=cname,
    )
    iz.loadCameraInfo()

    # Exact-zoom hit
    iz.set_zoom(0)
    assert iz.getCameraInfo().k[0] == pytest.approx(400.0)
    iz.set_zoom(100)
    assert iz.getCameraInfo().k[0] == pytest.approx(800.0)

    # Midpoint interpolation: ratio = (50 - 0) / (100 - 0) = 0.5
    # camera_info.k = ratio * low + (1 - ratio) * high = 0.5*400 + 0.5*800 = 600
    iz.set_zoom(50)
    assert iz.getCameraInfo().k[0] == pytest.approx(600.0)
    assert iz.getCameraInfo().k[4] == pytest.approx(600.0)
