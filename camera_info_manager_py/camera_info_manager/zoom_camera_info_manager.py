#!/usr/bin/env python
# Copyright 2016, Martin Pecka
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Martin Pecka nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
`CameraInfo` support for drivers of zoom cameras written in Python.

A similar C++ API does not exist yet.
"""
# enable some python3 compatibility options:

from copy import deepcopy
from math import radians, tan

from sensor_msgs.msg import CameraInfo

from camera_info_manager import (
    CameraInfoError,
    CameraInfoManager,
    CameraInfoMissingError,
    getPackageFileName,
    loadCalibrationFile,
    parseURL,
    resolveURL,
    URL_empty,
    URL_file,
    URL_package,
)


class ZoomCameraInfoManager(CameraInfoManager):
    """
    :class:`ZoomCameraInfoManager` provides ROS CameraInfo support for Python zoom camera drivers.

    This is a base class for all zoom cameras (since there are several
    ways to implement the changing camera infos).

    No standardized calibration package/procedure has been released yet,
    but at each specific class' docstring contains some hints on how to
    calibrate a zoom camera.
    """

    def __init__(self, min_zoom, max_zoom, cname='camera', url='', namespace=''):
        """
        Construct the manager.

        :param int min_zoom: The minimum zoom level.
                             The zoom values should linearly affect the focal distance.
        :param int max_zoom: The maximum zoom level.
                             The zoom values should linearly affect the focal distance.
        :param string cname: camera name.
        :param string url: Uniform Resource Locator for camera calibration data.
                           This should point to the minimum zoom calibration file.
        :param string namespace: Optional ROS namespace prefix for the service name.
                                 If a namespace is specified, the '/' separator required between it
                                 and ``set_camera_info`` will be supplied automatically.
        """
        CameraInfoManager.__init__(self, cname, url, namespace)

        self._min_zoom = min_zoom
        self._max_zoom = max_zoom
        self._zoom = min_zoom

    def _update_camera_info(self):
        """Update the camera info after zoom or calibration has changed."""
        raise NotImplementedError()

    def set_zoom(self, zoom):
        """
        Set zoom to the given level and update the camera info.

        :param int zoom: The zoom level.

        :raises: :exc:`CameraInfoError` if
                      the zoom level is outside the bounds given in constructor.
        """
        if zoom < self._min_zoom or zoom > self._max_zoom:
            raise CameraInfoError('Zoom got outside the allowed range')

        self._zoom = zoom

        self._update_camera_info()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.svc.shutdown()


class ApproximateZoomCameraInfoManager(ZoomCameraInfoManager):
    """
    A zoom camera info manager.

    A zoom camera info manager implementation that only approximates the K matrix
    by computing it from specified minimum and maximum field of view (FOV). This
    will probably never be very precise, but may work well enough for some cameras.

    Only the K matrix is going to be computed. All other parts of the :class:`CameraInfo`
    message are either empty or copied from the loaded calibration file (if provided).

    Especially, the distortion coefficients should also decrease with increasing zoom
    level, but the relation is unclear.

    Hints for calibration:
    You should find the minimum and maximum FOV (Field Of View) of your camera in its
    documentation (or the documentation of the lens).
    The FOV passed to constructor is the horizontal FOV; vertical FOV is derived from
    the horizontal FOV and image aspect ratio. You can also measure the FOV physically
    by shooting a tape measure and using a bit of trigonometry.
    Ideally, you should calibrate the camera for the lowest and highest zoom levels,
    and then compare, if the K matrix returned by this manager is similar. If it is not,
    just play with the minimum/maximum FOV, until you are close enough at both ends.
    """

    def __init__(
        self,
        min_fov,
        max_fov,
        initial_image_width,
        initial_image_height,
        min_zoom,
        max_zoom,
        cname='camera',
        url='',
        namespace='',
    ):
        """
        Construct the manager.

        :param float min_fov: The minimum horizontal field of view. Corresponds to
            maximum zoom level.
        :param float max_fov: The maximum horizontal field of view. Corresponds to
            minimum zoom level.
        :param int initial_image_width: Horizontal resolution of the image.
        :param int initial_image_height: Vertical resolution of the image.
        :param int min_zoom: The minimum zoom level. The zoom values should linearly
            affect the focal distance.
        :param int max_zoom: The maximum zoom level. The zoom values should linearly
            affect the focal distance.
        :param string cname: camera name.
        :param string url: Uniform Resource Locator for camera calibration data. This
            should point to the minimum zoom calibration file.
        :param string namespace: Optional ROS namespace prefix for the service name.
            If a namespace is specified, the '/' separator required between it and
            ``set_camera_info`` will be supplied automatically.
        """
        ZoomCameraInfoManager.__init__(self, min_zoom, max_zoom, cname, url, namespace)

        self._min_fov = min_fov
        self._max_fov = max_fov

        self._image_width = initial_image_width
        self._image_height = initial_image_height

        self._loaded_camera_info = None
        """Camera info loaded from the calibration URL."""

    def loadCameraInfo(self):
        CameraInfoManager.loadCameraInfo(self)

        self._loaded_camera_info = deepcopy(self.camera_info)

    def set_resolution(self, width, height):
        """
        Set resolution of the image plane and updates the camera info.

        :param int width: Width of the image in px.
        :param int height: Height of the image in px.
        """
        self._image_width = width
        self._image_height = height

        self._update_camera_info()

    def _update_camera_info(self):
        if self._loaded_camera_info is None:
            return

        self.camera_info = deepcopy(self._loaded_camera_info)
        self.camera_info.k[8] = 1.0

        aspect_ratio = float(self._image_height) / self._image_width
        zoom_percentage = float(self._zoom - self._min_zoom) / float(
            self._max_zoom - self._min_zoom
        )

        horizontal_fov = radians(
            (self._max_fov - self._min_fov) * (1 - zoom_percentage) + self._min_fov
        )
        horizontal_focal_length_in_px = self._image_width / (2 * tan(horizontal_fov / 2))
        self.camera_info.k[0] = horizontal_focal_length_in_px

        vertical_fov = horizontal_fov * aspect_ratio
        vertical_focal_length_in_px = self._image_height / (2 * tan(vertical_fov / 2))
        self.camera_info.k[4] = vertical_focal_length_in_px

        self.camera_info.width = self._image_width
        self.camera_info.height = self._image_height

        if self._loaded_camera_info.k[2] != 0.0:
            # if a standard calibration is available, just scale the principal point
            # offset with the resolution
            self.camera_info.k[2] = (
                float(self._image_width)
                / self._loaded_camera_info.width
                * self._loaded_camera_info.k[2]
            )
            self.camera_info.k[5] = (
                float(self._image_height)
                / self._loaded_camera_info.height
                * self._loaded_camera_info.k[5]
            )
        else:
            # if no calibration is available, just set the principal point to lie in
            # the middle of the image
            self.camera_info.k[2] = self._image_width / 2.0
            self.camera_info.k[5] = self._image_height / 2.0


class InterpolatingZoomCameraInfoManager(ZoomCameraInfoManager):
    """
    An Interpoated zoom camera info manager.

    Interpolates between several calibrations taken at different zoom levels.

    The calibration results can be as accurate as you need (you increase accuracy
    by adding more calibration files). E.g. if the camera has only a few discrete
    zoom steps, you can just calibrate all of them and reach the lowest calibration
    error possible.

    Hints for calibration:
    Use the standard camera calibration package, set a fixed zoom level and perform
    the calibration. Copy the calibration YAML file to some other location, change
    the zoom level, and calibrate again. Rename all the calibration YAML files, so
    that their names correspond to the calibration_url_template you pass to the
    constructor.
    It is advised to always perform calibration on (at least) the highest and lowest
    zoom step.
    The last calibration should be on the most-used zoom level, in case some other
    code doesn't know this is a zoom camera
    """

    def __init__(
        self, calibration_url_template, zoom_levels, cname='camera', url='', namespace=''
    ):
        """
        Construct the manager.

        :param string calibration_url_template: Template of the URL that contains the
            calibration files. The template string should contain a single '%d'
            substitution, which will be substituted with zoom level.
        :param list zoom_levels: The zoom levels at which the calibration files exist.
            The smallest value is automatically taken as the minimum allowed zoom, and
            respectively for the largest one.
        :param string cname: camera name.
        :param string url: Uniform Resource Locator for camera calibration data. This
            should point to the minimum zoom calibration file.
        :param string namespace: Optional ROS namespace prefix for the service name.
            If a namespace is specified, the '/' separator required between it and
            ``set_camera_info`` will be supplied automatically.
        """
        ZoomCameraInfoManager.__init__(
            self, min(zoom_levels), max(zoom_levels), cname, url, namespace
        )

        self._calibration_url_template = calibration_url_template
        self._zoom_levels = zoom_levels

        self._camera_infos = None

    def _update_camera_info(self):
        if not self.isCalibrated() or self._camera_infos is None:
            return

        if self._zoom in list(self._camera_infos.keys()):
            self.camera_info = deepcopy(self._camera_infos[self._zoom])
            return

        # find the closest zoom levels at which we have calibration available
        closest_higher_zoom = min(z for z in self._zoom_levels if z >= self._zoom)
        closest_lower_zoom = max(z for z in self._zoom_levels if z <= self._zoom)

        # compute the weight of the lower and higher calibration files
        ratio = float(self._zoom - closest_lower_zoom) / float(
            closest_higher_zoom - closest_lower_zoom
        )

        # linearly interpolate all the matrices
        self.camera_info = deepcopy(self._camera_infos[closest_lower_zoom])
        self.camera_info.k = [
            (ratio * low + (1 - ratio) * high)
            for (low, high) in zip(
                self._camera_infos[closest_lower_zoom].k, self._camera_infos[closest_higher_zoom].k
            )
        ]
        self.camera_info.p = [
            (ratio * low + (1 - ratio) * high)
            for (low, high) in zip(
                self._camera_infos[closest_lower_zoom].p, self._camera_infos[closest_higher_zoom].p
            )
        ]
        self.camera_info.r = [
            (ratio * low + (1 - ratio) * high)
            for (low, high) in zip(
                self._camera_infos[closest_lower_zoom].r, self._camera_infos[closest_higher_zoom].r
            )
        ]
        self.camera_info.d = [
            (ratio * low + (1 - ratio) * high)
            for (low, high) in zip(
                self._camera_infos[closest_lower_zoom].d, self._camera_infos[closest_higher_zoom].d
            )
        ]

    def loadCameraInfo(self):
        CameraInfoManager.loadCameraInfo(self)

        # load all calibration files for all zoom levels
        self._camera_infos = {}
        for zoom_level in self._zoom_levels:
            resolved_url = resolveURL(self._calibration_url_template % zoom_level, self.cname)
            url_type = parseURL(resolved_url)

            if url_type == URL_empty:
                raise CameraInfoError('Zoom camera cannot use default calibration URLs.')

            self.node.get_logger().info(
                'camera calibration URL for zoom level %d: %s' % (zoom_level, resolved_url)
            )

            if url_type == URL_file:
                self._camera_infos[zoom_level] = loadCalibrationFile(resolved_url[7:], self.cname)

            elif url_type == URL_package:
                filename = getPackageFileName(resolved_url)
                if not filename:  # package not resolved
                    raise CameraInfoMissingError('Calibration package missing.')
                self._camera_infos[zoom_level] = loadCalibrationFile(filename, self.cname)

            else:
                self.node.get_logger().error('Invalid camera calibration URL: ' + resolved_url)
                self._camera_infos[zoom_level] = CameraInfo()

        if len(list(self._camera_infos.keys())) < 2:
            raise CameraInfoError(
                'Interpolating zoom camera info manager needs at least two calibrations to exist.'
            )
