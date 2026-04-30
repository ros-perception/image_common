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

import pytest
import rclpy
from rclpy.node import Node

from camera_info_manager import (
    ApproximateZoomCameraInfoManager,
    CameraInfoError,
    CameraInfoManager,
    CameraInfoMissingError,
    InterpolatingZoomCameraInfoManager,
    ZoomCameraInfoManager,
)


@pytest.fixture(scope='module', autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Node('test_camera_info_manager')
    yield n
    n.destroy_node()


def test_camera_info_manager_constructs(node):
    cim = CameraInfoManager(node, cname='cam0', url='', namespace='')
    assert cim.getCameraName() == 'cam0'
    assert cim.getURL() == ''


def test_camera_info_manager_str_does_not_reference_undefined_attrs(node):
    cim = CameraInfoManager(node, cname='cam0', url='file:///tmp/x.yaml')
    assert str(cim) == '[cam0]file:///tmp/x.yaml'


def test_get_camera_info_raises_before_load(node):
    cim = CameraInfoManager(node, cname='cam0')
    with pytest.raises(CameraInfoMissingError):
        cim.getCameraInfo()


def test_set_camera_name_validation(node):
    cim = CameraInfoManager(node, cname='cam0')
    assert cim.setCameraName('new_name') is True
    assert cim.getCameraName() == 'new_name'
    assert cim.setCameraName('') is False
    assert cim.setCameraName('bad name') is False


def test_zoom_manager_constructs_with_node(node):
    z = ZoomCameraInfoManager(node, min_zoom=0, max_zoom=10, cname='zoom0')
    assert z.getCameraName() == 'zoom0'
    assert z._min_zoom == 0
    assert z._max_zoom == 10
    assert z._zoom == 0


def test_zoom_manager_set_zoom_in_range(node):
    z = ApproximateZoomCameraInfoManager(
        node, min_fov=30.0, max_fov=70.0,
        initial_image_width=640, initial_image_height=480,
        min_zoom=0, max_zoom=100, cname='zoom0',
    )
    z.set_zoom(50)
    assert z._zoom == 50


def test_zoom_manager_set_zoom_out_of_range_raises(node):
    z = ApproximateZoomCameraInfoManager(
        node, min_fov=30.0, max_fov=70.0,
        initial_image_width=640, initial_image_height=480,
        min_zoom=0, max_zoom=100, cname='zoom0',
    )
    with pytest.raises(CameraInfoError):
        z.set_zoom(101)
    with pytest.raises(CameraInfoError):
        z.set_zoom(-1)


def test_approximate_zoom_updates_k_without_calibration(node):
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


def test_approximate_zoom_set_resolution(node):
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


def test_interpolating_zoom_constructs(node):
    iz = InterpolatingZoomCameraInfoManager(
        node, calibration_url_template='file:///tmp/cal_%d.yaml',
        zoom_levels=[0, 50, 100], cname='zoom0',
    )
    assert iz._min_zoom == 0
    assert iz._max_zoom == 100
    assert iz._calibration_url_template == 'file:///tmp/cal_%d.yaml'


def test_zoom_manager_context_exit_does_not_raise(node):
    z = ZoomCameraInfoManager(node, min_zoom=0, max_zoom=10, cname='zoom0')
    with z:
        pass
