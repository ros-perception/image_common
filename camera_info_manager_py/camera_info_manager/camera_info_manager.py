#!/usr/bin/env python
# Copyright 2012, Jack O'Quin
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

"""Python camera_info_manager interface.

Providing `CameraInfo` support for drivers written in Python.
This is very similar to the

`C++ camera_info_manager`_ package, but not identical.

.. _`C++ camera_info_manager`: http://ros.org/wiki/camera_info_manager
.. _`sensor_msgs/CameraInfo`: http://ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html
.. _`sensor_msgs/SetCameraInfo`: http://ros.org/doc/api/sensor_msgs/html/srv/SetCameraInfo.html

"""

import errno
import locale
import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import yaml

default_camera_info_url = 'file://${ROS_HOME}/camera_info/${NAME}.yaml'
# parseURL() type codes:
URL_empty = 0  # empty string
URL_file = 1  # file:
URL_package = 2  # package:
URL_invalid = 3  # anything >= is invalid


class CameraInfoError(Exception):
    """Base class for exceptions in this module.

    ..exception: CameraInfoError
    """


class CameraInfoMissingError(CameraInfoError):
    """Exception raised when CameraInfo has not been loaded.

    ..exception: CameraInfoMissingError
    """


class CameraInfoManager:
    """CameraInfoManager class.

    :class:`CameraInfoManager` provides ROS CameraInfo support for
    Python camera drivers. It handles the `sensor_msgs/SetCameraInfo`_
    service requests, saving and restoring `sensor_msgs/CameraInfo`_
    data.

    :param cname: camera name.
    :param url: Uniform Resource Locator for camera calibration data.
    :param namespace: Optional ROS namespace prefix for the service
           name.  If a namespace is specified, the '/' separator
           required between it and ``set_camera_info`` will be
           supplied automatically.

    .. describe:: str(camera_info_manager_obj)

       :returns: String representation of :class:`CameraInfoManager` object.

    **ROS Service**

    - set_camera_info (`sensor_msgs/SetCameraInfo`_) stores
                      calibration information

    Typically, these service requests are made by a calibration
    package, such as:

    - http://www.ros.org/wiki/camera_calibration

    The calling node *must* invoke `rospy.spin()` in some thread, so
    :class:`CameraInfoManager` gets called to handle arriving service
    requests.

    If a driver handles multiple cameras, it should use the
    ``namespace`` parameter to declare separate
    :class:`CameraInfoManager` instances for each one, as in this
    stereo example::

      left_ci = CameraInfoManager(cname='left_camera', namespace='left')
      right_ci = CameraInfoManager(cname='right_camera', namespace='right')

    **Camera Name**

    The device driver sets a camera name via the
    :class:`CameraInfoManager` constructor or the
    :py:meth:`setCameraName` method.  This name is written when
    CameraInfo is saved, and checked when data are loaded, with a
    warning logged if the name read does not match.

    Syntax: a camera name contains any combination of alphabetic,
            numeric and '_' characters.  Case is significant.

    Camera drivers may use any syntactically valid name they please.
    Where possible, it is best for the name to be unique to the
    device, such as a GUID, or the make, model and serial number.  Any
    parameters that affect calibration, such as resolution, focus,
    zoom, etc., may also be included in the name, uniquely identifying
    each CameraInfo file.

    The camera name can be resolved as part of the URL, allowing
    direct access to device-specific calibration information.

    **Uniform Resource Locator**

    The location for getting and saving calibration data is expressed
    by Uniform Resource Locator.  The driver defines a URL via the
    :class:`CameraInfoManager` constructor or the :py:meth:`setURL`
    method.  Many drivers provide a `~camera_info_url` parameter so
    users may customize this URL, but that is handled outside this
    class.

    Camera calibration information is stored in YAML format.

    Example URL syntax:

    - `file:///full/path/to/local/file.yaml`
    - `package://camera_info_manager_py/tests/test_calibration.yaml`
    - `package://ros_package_name/calibrations/camera3.yaml`

    The `file:` URL specifies a full path name in the local system.
    The `package:` URL is handled the same as `file:`, except the path
    name is resolved relative to the location of the named ROS
    package, which must be reachable via `$ROS_PACKAGE_PATH`.

    The URL may contain substitution variables delimited by `${...}`,
    including:

    - ${NAME} resolved to the current camera name defined by the
              device driver.
    - ${ROS_HOME} resolved to the `$ROS_HOME` environment variable if
                  defined, `~/.ros` if not.

    Resolution is done in a single pass through the URL string.
    Variable values containing substitutable strings are not resolved
    recursively.  Unrecognized variable names are treated literally
    with no substitution, but an error is logged.

    Examples with variable substitution:

    - `package://my_cameras/calibrations/${NAME}.yaml`
    - `file://${ROS_HOME}/camera_info/left_front_camera.yaml`

    The default URL is:

    - `file://${ROS_HOME}/camera_info/${NAME}.yaml`

    If that file exists, its contents are used. Any new calibration
    will be stored there, missing parent directories being created if
    necessary and possible.

    **Loading Calibration Data**

    Unlike the `C++ camera_info_manager`_, this Python implementation
    loads nothing until the :py:meth:`loadCameraInfo` method is
    called.  It is an error to call :py:meth:`getCameraInfo`, or
    :py:meth:`isCalibrated` before that is done.

    If the URL or camera name changes, :py:meth:`loadCameraInfo` must
    be called again before the data are accessible.

    """

    def __init__(self, node: Node, cname='camera', url='', namespace=''):
        """Call the Constructor."""
        self.node = node
        self.cname = cname
        self.url = url
        self.camera_info = None

        # advertise set_camera_info service
        service_name = 'set_camera_info'
        if namespace:
            service_name = namespace + '/' + service_name
        self.node.get_logger().debug(service_name + ' service declared')
        self.svc = self.node.create_service(SetCameraInfo, service_name, self.setCameraInfo)

    def __str__(self):
        """Return string representation of CameraInfoManager.

        :returns: Return string representation of :class:CameraInfoManager.
        """
        return '[' + self.cname + ']' + str(self.utm)

    def getCameraInfo(self):
        """Get the current camera calibration.

        The :py:meth:`loadCameraInfo` must have been called since the
        last time the camera name or URL changed.

        :returns: `sensor_msgs/CameraInfo`_ message.

        :raises: :exc:`CameraInfoMissingError` if camera info not up
                 to date.

        """
        if self.camera_info is None:
            raise CameraInfoMissingError('Calibration missing, loadCameraInfo() needed.')
        return self.camera_info

    def getCameraName(self):
        """Get the current camera name.

        :returns: camera name string
        """
        return self.cname

    def getURL(self):
        """Get the current calibration URL.

        :returns: URL string without variable expansion.
        """
        return self.url

    def isCalibrated(self):
        """Is the current CameraInfo calibrated?.

        The :py:meth:`loadCameraInfo` must have been called since the
        last time the camera name or URL changed.

        :returns: True if camera calibration exists;
                  False for null calibration.

        :raises: :exc:`CameraInfoMissingError` if camera info not up
                 to date.

        """
        if self.camera_info is None:
            raise CameraInfoMissingError('Calibration missing, ' + 'loadCameraInfo() needed.')
        return self.camera_info.k[0] != 0.0

    def _loadCalibration(self, url, cname):
        """Load calibration data (if any available).

        This method updates self.camera_info, if possible, based on
        the url and cname parameters.  An empty or non-existent
        calibration is *not* considered an error, a null
        `sensor_msgs/CameraInfo`_ being provided in that case.

        :param url: Uniform Resource Locator for calibration data.
        :param cname: Camera name.

        :raises: :exc:`IOError` if an existing calibration is unreadable.
        :raises: :exc:`CameraInfoMissingError` if a `package:` URL is
                 inaccessible.
        """
        resolved_url = resolveURL(url, cname)
        url_type = parseURL(resolved_url)

        if url_type == URL_empty:
            self._loadCalibration(default_camera_info_url, cname)
            return

        self.node.get_logger().info('camera calibration URL: ' + resolved_url)

        if url_type == URL_file:
            self.camera_info = loadCalibrationFile(resolved_url[7:], cname)

        elif url_type == URL_package:
            filename = getPackageFileName(resolved_url)
            if not filename:  # package not resolved
                raise CameraInfoMissingError('Calibration package missing.')
            self.camera_info = loadCalibrationFile(filename, cname)

        else:
            self.node.get_logger().error('Invalid camera calibration URL: ' + resolved_url)
            self.camera_info = CameraInfo()

    def loadCameraInfo(self):
        """Load currently configured calibration data (if any).

        This method updates camera_info, if possible, based on the
        currently-configured URL and camera name.  An empty or
        non-existent calibration is *not* considered an error; a null
        `sensor_msgs/CameraInfo`_ being provided in that case.

        :raises: :exc:`IOError` if an existing calibration is unreadable.

        """
        self._loadCalibration(self.url, self.cname)

    def setCameraInfo(self, req):
        """Set camera info request callback.

        :param req: SetCameraInfo request message.
        :returns: SetCameraInfo response message, success is True if
                  message handled.

        :post: camera_info updated, can be used immediately without
               reloading.
        """
        self.node.get_logger().debug('SetCameraInfo received for ' + self.cname)
        self.camera_info = req.camera_info
        rsp = SetCameraInfo.Response()
        rsp.success = saveCalibration(req.camera_info, self.url, self.cname)
        if not rsp.success:
            rsp.status_message = 'Error storing camera calibration.'
        return rsp

    def setCameraName(self, cname):
        """Set a new camera name.

        :param cname: camera name to use for saving calibration data

        :returns: True if new name has valid syntax; valid names
                  contain only alphabetic, numeric, or '_' characters.

        :post: camera name updated, if valid. A new name may affect
               the URL, so camera_info will have to be reloaded before
               being used again.

        """
        # validate name
        if not cname:
            return False  # name may not be empty
        for ch in cname:
            if not ch.isalnum() and ch != '_':
                return False  # invalid character

        # name is valid, use it
        if self.cname != cname:
            self.cname = cname
            self.camera_info = None  # missing if name changed
        return True

    def setURL(self, url):
        """Set the calibration URL.

        :param cname: camera name to use for saving calibration data

        :returns: True if new name has valid syntax.

        :post: URL updated, if valid. A new value may change the
               camera_info, so it will have to be reloaded before
               being used again.

        """
        if parseURL(resolveURL(url, self.cname)) >= URL_invalid:
            return False  # syntax error

        # URL looks valid, so use it
        if self.url != url:
            self.url = url
            self.camera_info = None  # missing if URL changed
        return True


# related utility functions


def genCameraName(from_string):
    """Generate a valid camera name.

    Valid names contain only alphabetic, numeric, or '_'
    characters. All invalid characters in from_string are replaced
    by an '_'.

    :param from_string: string from which to base camera name.

    :returns: a valid camera name based on from_string.

    """
    if not from_string:
        return '_'  # name may not be empty

    retval = ''
    for i in range(len(from_string)):
        if not from_string[i].isalnum() and from_string[i] != '_':
            retval += '_'
        else:
            retval += from_string[i]
    return retval


def getPackageFileName(url):
    """Get file name corresponding to a `package:` URL.

    `param url` fully-resolved Uniform Resource Locator
    `returns` file name if package found, "" otherwise

    """
    # Scan URL from after "package://" until next '/' and extract
    # package name.  The parseURL() already checked that it's present.
    prefix_len = len('package://')
    rest = url.find('/', prefix_len)
    package = url[prefix_len:rest]

    # Look up the ROS package path name.
    pkgPath = ''
    try:
        pkgPath = get_package_share_directory(package)
        pkgPath += url[rest:]

    except PackageNotFoundError:
        rclpy.logging.get_logger('camera_info_manager').warning(
            'unknown package: ' + package + ' (ignored)'
        )

    return pkgPath


def loadCalibrationFile(filename, cname):
    """Load calibration data from a file.

    This function returns a `sensor_msgs/CameraInfo`_ message, based
    on the filename parameter.  An empty or non-existent file is *not*
    considered an error; a null CameraInfo being provided in that
    case.

    :param filename: location of CameraInfo to read
    :param cname: Camera name.
    :returns: `sensor_msgs/CameraInfo`_ message containing calibration,
              if file readable; null calibration message otherwise.
    :raises: :exc:`IOError` if an existing calibration file is unreadable.

    """
    ci = CameraInfo()
    try:
        with Path(filename).open(encoding=locale.getpreferredencoding(False)) as f:
            calib = yaml.safe_load(f)
            if calib is not None:
                if calib['camera_name'] != cname:
                    rclpy.logging.get_logger('camera_info_manager').warn(
                        '['
                        + cname
                        + '] does not match name '
                        + calib['camera_name']
                        + ' in file '
                        + filename
                    )

                # fill in CameraInfo fields
                ci.width = calib['image_width']
                ci.height = calib['image_height']
                ci.distortion_model = calib['distortion_model']
                ci.d = calib['distortion_coefficients']['data']
                ci.k = calib['camera_matrix']['data']
                ci.r = calib['rectification_matrix']['data']
                ci.p = calib['projection_matrix']['data']

    except OSError:  # OK if file did not exist
        pass

    return ci


def parseURL(url):
    """Parse calibration Uniform Resource Locator.

    `param url`: string to parse
    `returns` URL type code

    `note`: Unsupported URL types have codes >= URL_invalid.

    """
    if not url:
        return URL_empty

    if url[0:8].upper() == 'FILE:///':
        return URL_file

    if url[0:10].upper() == 'PACKAGE://':
        # look for a '/' following the package name, make sure it is
        # there, the name is not empty, and something follows it

        rest = url.find('/', 10)
        if rest < len(url) - 1 and rest >= 0:
            return URL_package

    return URL_invalid


def resolveURL(url, cname):
    """Resolve substitution strings in Uniform Resource Locator.

    :param url: URL to resolve, which may include `${...}`
                substitution variables.
    :param cname: camera name for resolving `${NAME}` variables.

    :returns: a copy of the URL with any variable information resolved.

    """
    resolved = ''  # resolved URL to return
    rest = 0  # index of remaining string to parse

    while True:
        # find the next '$' in the URL string
        dollar = url.find('$', rest)

        if dollar == -1:  # no more '$'s there?
            resolved += url[rest:]
            return resolved

        # copy characters up to the next '$'
        resolved += url[rest:dollar]

        if url[dollar + 1 : dollar + 2] != '{':
            #  no '{' follows, so keep the '$'
            resolved += '$'

        elif url[dollar + 1 : dollar + 7] == '{NAME}':
            # substitute camera name
            resolved += cname
            dollar += 6

        elif url[dollar + 1 : dollar + 11] == '{ROS_HOME}':
            # substitute $ROS_HOME
            ros_home = os.environ.get('ROS_HOME')
            if ros_home is None:
                ros_home = os.environ.get('HOME')
                if ros_home is None:
                    rclpy.logging.get_logger('camera_info_manager').warn(
                        '[CameraInfoManager]' + ' unable to resolve ${ROS_HOME}'
                    )
                    ros_home = '${ROS_HOME}'  # retain it unresolved
                else:
                    ros_home += '/.ros'
            resolved += ros_home
            dollar += 10

        else:
            # not a valid substitution variable
            rclpy.logging.get_logger('camera_info_manager').warn(
                '[CameraInfoManager] invalid URL substitution (not resolved): ' + url
            )
            resolved += '$'  # keep the bogus '$'

        # look for next '$'
        rest = dollar + 1


def saveCalibration(new_info, url, cname):
    """Save calibration data.

    This function writes new calibration information to the
    location defined by the url and cname parameters, if possible.

    :param new_info: `sensor_msgs/CameraInfo`_ to save.
    :param url: Uniform Resource Locator for calibration data (if
                empty use file://${ROS_HOME}/camera_info/${NAME}.yaml).
    :param cname: Camera name.
    :returns: True if able to save the data.
    """
    success = False
    resolved_url = resolveURL(url, cname)
    url_type = parseURL(resolved_url)

    if url_type == URL_empty:
        return saveCalibration(new_info, default_camera_info_url, cname)

    rclpy.logging.get_logger('camera_info_manager').info(
        'writing calibration data to URL: ' + resolved_url
    )

    if url_type == URL_file:
        success = saveCalibrationFile(new_info, resolved_url[7:], cname)

    elif url_type == URL_package:
        filename = getPackageFileName(resolved_url)
        if not filename:  # package not resolved
            rclpy.logging.get_logger('camera_info_manager').error(
                'Calibration package missing: ' + resolved_url + ' (ignored)'
            )
            # treat it like an empty URL
            success = saveCalibration(new_info, default_camera_info_url, cname)
        else:
            success = saveCalibrationFile(new_info, filename, cname)

    else:
        rclpy.logging.get_logger('camera_info_manager').error(
            'Invalid camera calibration URL: ' + resolved_url
        )
        # treat it like an empty URL
        success = saveCalibration(new_info, default_camera_info_url, cname)
    return success


def saveCalibrationFile(ci, filename, cname):
    """Save calibration data to a YAML file.

    This function writes the new calibration information to a YAML
    file, if possible.

    :param ci: `sensor_msgs/CameraInfo`_ to save.
    :param filename: local file to store data.
    :param cname: Camera name.
    :returns: True if able to save the data.
    """
    # make calibration dictionary from CameraInfo fields and camera name
    calib = {
        'image_width': ci.width,
        'image_height': ci.height,
        'camera_name': cname,
        'distortion_model': ci.distortion_model,
        'distortion_coefficients': {'data': ci.d, 'rows': 1, 'cols': len(ci.d)},
        'camera_matrix': {'data': ci.k, 'rows': 3, 'cols': 3},
        'rectification_matrix': {'data': ci.r, 'rows': 3, 'cols': 3},
        'projection_matrix': {'data': ci.p, 'rows': 3, 'cols': 4},
    }

    # make sure the directory exists and the file is writable
    try:
        with Path(filename).open('w', encoding=locale.getpreferredencoding(False)) as f:
            try:
                yaml.safe_dump(calib, f)
                return True
            except OSError:
                return False  # fail if unable to write file
    except OSError as e:
        if e.errno in {errno.EACCES, errno.EPERM}:
            rclpy.logging.get_logger('camera_info_manager').error(
                'file [' + filename + '] not accessible'
            )
            return False  # unable to write this file
        if e.errno in {errno.ENOENT}:
            # Find last slash in the name.  The URL parser ensures
            # there is at least one '/', at the beginning.
            last_slash = filename.rfind('/')
            if last_slash < 0:
                rclpy.logging.get_logger('camera_info_manager').error(
                    'filename [' + filename + "] has no '/'"
                )
                return False  # not a valid URL

            # try to create the directory and all its parents
            dirname = filename[0 : last_slash + 1]
            try:
                Path(dirname).mkdir(parents=True)
            except OSError:
                rclpy.logging.get_logger('camera_info_manager').error(
                    'unable to create path to directory [' + dirname + ']'
                )
                return False

            # try again to create the file
            with Path(filename).open('w', encoding=locale.getpreferredencoding(False)) as f:
                try:
                    yaml.safe_dump(calib, f)
                    return True

                except OSError:
                    rclpy.logging.get_logger('camera_info_manager').error(
                        'file [' + filename + '] not accessible'
                    )
                    return False  # fail if unable to write file
