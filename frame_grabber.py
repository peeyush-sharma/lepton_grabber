#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This code builds upon the python code sample from groupgets github page.
# https://github.com/groupgets/purethermal1-uvc-capture/blob/master/python/uvc-radiometry.py
import argparse
import atexit
import binascii
import os
import platform
import queue
import threading
import time
from ctypes import *

import numpy as np
import png

# Following structures are used as data exchange/storage implements when dealing with the precompiled libuvc lib by
#  leveraging ctypes functionality of python.
class uvc_context(Structure):
    _fields_ = [("usb_ctx", c_void_p),
                ("own_usb_ctx", c_uint8),
                ("open_devices", c_void_p),
                ("handler_thread", c_ulong),
                ("kill_handler_thread", c_int)]


class uvc_device(Structure):
    _fields_ = [("ctx", POINTER(uvc_context)),
                ("ref", c_int),
                ("usb_dev", c_void_p)]


class uvc_stream_ctrl(Structure):
    _fields_ = [("bmHint", c_uint16),
                ("bFormatIndex", c_uint8),
                ("bFrameIndex", c_uint8),
                ("dwFrameInterval", c_uint32),
                ("wKeyFrameRate", c_uint16),
                ("wPFrameRate", c_uint16),
                ("wCompQuality", c_uint16),
                ("wCompWindowSize", c_uint16),
                ("wDelay", c_uint16),
                ("dwMaxVideoFrameSize", c_uint32),
                ("dwMaxPayloadTransferSize", c_uint32),
                ("dwClockFrequency", c_uint32),
                ("bmFramingInfo", c_uint8),
                ("bPreferredVersion", c_uint8),
                ("bMinVersion", c_uint8),
                ("bMaxVersion", c_uint8),
                ("bInterfaceNumber", c_uint8)]


class uvc_format_desc(Structure):
    pass


class timeval(Structure):
    _fields_ = [("tv_sec", c_long), ("tv_usec", c_long)]


class uvc_frame(Structure):
    _fields_ = [("data", POINTER(c_uint8)),  # /** Image data for this frame */
                ("data_bytes", c_size_t),  # /** Size of image data buffer */
                ("width", c_uint32),  # /** Width of image in pixels */
                ("height", c_uint32),  # /** Height of image in pixels */
                # /** Pixel data format */
                ("frame_format", c_uint),  # enum uvc_frame_format frame_format
                ("step", c_size_t),  # /** Number of bytes per horizontal line (undefined for compressed format) */
                ("sequence", c_uint32),  # /** Frame number (may skip, but is strictly monotonically increasing) */
                ("capture_time", timeval),  # /** Estimate of system time when the device started capturing the image */
                # /** Handle on the device that produced the image.
                #  * @warning You must not call any uvc_* functions during a callback. */
                ("source", POINTER(uvc_device)),
                # /** Is the data buffer owned by the library?
                #  * If 1, the data buffer can be arbitrarily reallocated by frame conversion
                #  * functions.
                #  * If 0, the data buffer will not be reallocated or freed by the library.
                #  * Set this field to zero if you are supplying the buffer.
                #  */
                ("library_owns_data", c_uint8)]


class uvc_device_handle(Structure):
    _fields_ = [("dev", POINTER(uvc_device)),
                ("prev", c_void_p),
                ("next", c_void_p),
                ("usb_devh", c_void_p),
                ("info", c_void_p),
                ("status_xfer", c_void_p),
                ("status_buf", c_ubyte * 32),
                ("status_cb", c_void_p),
                ("status_user_ptr", c_void_p),
                ("button_cb", c_void_p),
                ("button_user_ptr", c_void_p),
                ("streams", c_void_p),
                ("is_isight", c_ubyte)]


class lep_oem_sw_version(Structure):
    _fields_ = [("gpp_major", c_ubyte),
                ("gpp_minor", c_ubyte),
                ("gpp_build", c_ubyte),
                ("dsp_major", c_ubyte),
                ("dsp_minor", c_ubyte),
                ("dsp_build", c_ubyte),
                ("reserved", c_ushort)]

# Some static values
AGC_UNIT_ID = 3
OEM_UNIT_ID = 4
RAD_UNIT_ID = 5
SYS_UNIT_ID = 6
VID_UNIT_ID = 7

UVC_FRAME_FORMAT_UYVY = 4
UVC_FRAME_FORMAT_I420 = 5
UVC_FRAME_FORMAT_RGB = 7
UVC_FRAME_FORMAT_BGR = 8
UVC_FRAME_FORMAT_Y16 = 13

BUF_SIZE = 2  # Arbitrary buffer size for the queue storing image data


class Statics(object):
    """
    Used for defining static values
    """
    prefix_discovery = "video"
    path_discovery = "/dev"


class GrabThread(threading.Thread):
    def __init__(self, capture_device, trigger_event, barrier):
        """
        Init method
        :param capture_device: device to capture from
        :param trigger_event: trigger event object for threads to wait on
        :param barrier: thread barrier object
        """
        threading.Thread.__init__(self)
        self.device = capture_device
        self.trigger_event = trigger_event
        self.barrier = barrier

        self.img_frame = None

    def run(self):
        """
        run method
        :return: None
        """
        while True:
            self.barrier.wait()  # Wait at barrier until all threads reach here
            self.trigger_event.wait()  # Wait for the trigger event
            self.img_frame = self.device.capture()
            global event_kill_threads
            if event_kill_threads.isSet():
                self.barrier.wait()
                break

PFCFUNC = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)


class Camera(object):
    """
    Camera wrapper object
    """
    def __init__(self, uvc_lib=None, uvc_device=None, device_id=None):
        """
        Init method
        :param uvc_lib: Reference to the loaded libuvc lib
        :param uvc_device: device reference
        :param device_id: unique device id
        """
        self.libuvc = uvc_lib
        self.device = uvc_device
        self.id = device_id

        self.isLepton = False
        self.queue = queue.Queue(BUF_SIZE)  # Queue object for storing streaming frame data

        self.handle = None
        self.serial = None
        self.init_methods()

        def py_frame_callback(frame, userptr=None):
            """
            Callback function to process data from ctypes hook
            :param frame: frame data
            :param userptr: None
            :return: None
            """
            array_pointer = cast(frame.contents.data,
                                 POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
            data = np.frombuffer(array_pointer.contents, dtype=np.dtype(np.uint16)).reshape(frame.contents.height,
                                                                                            frame.contents.width)  # no copy
            if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
                return
            if not self.queue.full():
                self.queue.put(data)

        self.func_cb = PFCFUNC(py_frame_callback)

    def init_methods(self):
        """
        Methods to run on init
        :return: None
        """
        self.handle = POINTER(uvc_device_handle)()
        retcode = self.libuvc.uvc_open(self.device, byref(self.handle))
        assert (retcode == 0), "ERROR: uvc_open failed for device {}".format(self.id)
        atexit.register(self.libuvc.uvc_unref_device, self.device)  # Unref on program exit
        self.serial = self.get_serial(self.handle)  # Store serial

    def call_extension_unit(self, devh, unit, control, data, size):
        return self.libuvc.uvc_get_ctrl(devh, unit, control, data, size, 0x81)

    def get_serial(self, dev_handle=None):
        """
        Obtain serial of device
        :param dev_handle: device handle
        :return: string containing hex representation of serial
        """
        assert dev_handle, "Invalid device handle"
        serial_num = create_string_buffer(8)
        self.call_extension_unit(dev_handle, SYS_UNIT_ID, 3, serial_num, 8)
        return binascii.hexlify(serial_num.value)

    def start(self):
        """
        Wrapper for starting camera interface
        :return: None
        """
        ctrl = uvc_stream_ctrl()
        self.libuvc.uvc_get_stream_ctrl_format_size(self.handle, byref(ctrl), UVC_FRAME_FORMAT_Y16, 80, 60, 9)
        retcode = self.libuvc.uvc_start_streaming(self.handle, byref(ctrl), self.func_cb, None, 0)
        assert (retcode == 0), "Failed to start streaming device: {}".format(self.id)

    def stop(self):
        """
        Wrapper for stopping camera interface
        :return: None
        """
        self.libuvc.uvc_stop_streaming(self.handle)

    def capture(self, prefix=0):
        """
        Frame capture
        :param prefix: int to prefix on label
        :return: Captured frame
        """
        data = self.queue.get(True, 500)
        # Save stuff in various formats, TODO: Make this optional on debug switches
        with open("{}_img_{}.csv".format(prefix, self.id), mode='wb') as txt_file:
            np.savetxt(txt_file, data, fmt='%o', delimiter=',', newline='\n')
        rows, columns = data.shape
        with open("{}_img_{}.png".format(prefix, self.id), mode='wb') as bmp_file:
            png_writer = png.Writer(columns, rows, greyscale=True, alpha=False, bitdepth=16)
            png_writer.write(bmp_file, np.asarray(data))
        return data


class Grabber(object):
    """
    Trigger based image grabber from a video stream source
    """
    def __init__(self):
        self.libuvc = self.load_library()
        self.uvc_ctx = self.uvc_setup(self.libuvc)  # Get UVC context

        self.list_devices, self.list_size = self.discovery(self.libuvc, self.uvc_ctx)  # Run device discovery on init.

        self.trigger_event = threading.Event()  # Trigger event as a threading event
        self.barrier = threading.Barrier(self.list_size)

        # self.trigger_count = 0
        self.thread_list = []

    @staticmethod
    def load_library():
        """
        Loads up various precompiled libraries
        :return: reference to loaded lib
        """
        if platform.system() == 'Darwin':
            lib_uvc = cdll.LoadLibrary("libuvc.dylib")
        elif platform.system() == 'Linux':
            lib_uvc = cdll.LoadLibrary(os.path.join(os.getcwd(), "libs", "libuvc.so"))
        else:
            lib_uvc = cdll.LoadLibrary("libuvc")
        assert lib_uvc, "libuvc couldn't be loaded"
        return lib_uvc

    @staticmethod
    def discovery(lib_uvc=None, uvc_ctx=None):
        """
        Discover video devices
        :param lib_uvc: Reference to libuvc lib
        :param uvc_ctx: libuvc context
        :return: list of devices formatted as strings
        """
        assert lib_uvc, "Invalid UVC lib"
        assert uvc_ctx, "Invalid UVC context"
        P_uvc_device = POINTER(uvc_device)
        PP_uvc_device = POINTER(P_uvc_device)
        list_devices = PP_uvc_device()
        retcode = lib_uvc.uvc_get_device_list(uvc_ctx, byref(list_devices))
        assert (retcode == 0), "uvc_get_device_list error"

        # Find all /dev/videoX devices
        count = 0
        for item in os.listdir(Statics.path_discovery):
            if item.startswith(Statics.prefix_discovery):
                count += 1

        if count <= 0:
            raise ValueError("No camera devices found")
        return list_devices, count

    @staticmethod
    def uvc_teardown(lib_uvc, uvc_ctx):
        """
        Teardown UVC stuff
        :param lib_uvc: libuvc reference
        :param uvc_ctx: libuvc context
        :return:
        """
        lib_uvc.uvc_exit(uvc_ctx)

    @staticmethod
    def uvc_setup(lib_uvc):
        """
        Setup UVC stuff
        :param lib_uvc: libuvc reference
        :return: libuvc context
        """
        # Create a UVC context
        context = POINTER(uvc_context)()
        retcode = lib_uvc.uvc_init(byref(context), 0)  # Initialize the UVC context
        assert (retcode == 0), "uvc_init error: {}".format(retcode)
        # atexit.register(Grabber.uvc_teardown, lib_uvc, context)  # Register teardown at exit
        return context

    def start(self, count=1):
        """
        Starts the serial capture mode
        :param count: Count of captures to be performed
        :return: None
        """
        list_camera = []
        try:
            for i in range(self.list_size):  # Init all camera devices
                camera = Camera(self.libuvc, self.list_devices[i], i)
                list_camera.append(camera)

            for i in range(count):
                for camera in list_camera:
                    try:
                        camera.start()
                        camera.capture(prefix=i)
                    finally:
                        camera.stop()
                time.sleep(SLEEP_INTERVAL)
        finally:
            atexit.register(self.uvc_teardown, self.libuvc, self.uvc_ctx)  # Exit the UVC context)

    def start_threaded(self):
        """
        Start the threaded capture mode
        :return: None
        """
        list_camera = []
        try:
            for i in range(self.list_size):  # Step 1: Initialize all camera objects
                camera = Camera(self.libuvc, self.list_devices[i], i)
                list_camera.append(camera)

            for camera in list_camera:  # Step 2: Start all camera objects
                try:
                    camera.start()
                    camera.capture()

                    # Step 2: Now that all camera have been initialized and started, create threads to capture frames
                    dev_thread = GrabThread(camera, self.trigger_event, self.barrier)
                    self.thread_list.append(dev_thread)
                    dev_thread.start()
                finally:
                    camera.stop()
        finally:
            atexit.register(self.uvc_teardown, self.libuvc, self.uvc_ctx)  # Exit the UVC context)

if __name__ == '__main__':
    event_kill_threads = threading.Event()
    atexit.register(event_kill_threads.set)

    parser = argparse.ArgumentParser()
    parser.add_argument("--dbg_interval", help="Specify the sleep interval between captures", default='10', type=int)
    parser.add_argument("--dbg_capture_count", help="Specify count of captures to be done", default='10', type=int)
    args = parser.parse_args()
    SLEEP_INTERVAL = args.dbg_interval
    CAPTURE_COUNT = args.dbg_capture_count

    grab = Grabber()
    grab.start(count=CAPTURE_COUNT)
