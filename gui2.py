from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, \
    QDoubleSpinBox, QComboBox, QGridLayout, QWidget, QVBoxLayout, \
    QGraphicsPixmapItem, QLabel, QSlider, QMessageBox
from PyQt5 import QtCore, QtGui, uic
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPainter, QPixmap, QImage, QTransform
import pyqtgraph as pg

import sys
import time
from datetime import datetime
import numpy as np

from PIL import Image as Image_pil

from scipy.spatial import Delaunay

from toolbox import *

import threading
import zmq
import json
import os

## Camera stuff
from pyueye import ueye

def start_server(app_worker, raster_controls):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:55535")

    def handle_request(request):
        try:
            data = json.loads(request)
            action = data['action']
            connection = data['connection']
            if action == 'PROGRAM_VALUE':
                setpoint_value = data['value']
                return SET_VALUE(connection, setpoint_value)
            elif action == 'CHECK_VALUE':
                current_value = GET_VALUE(connection)
                return json.dumps({"status": "SUCCESS", "value": current_value})
            else:
                return json.dumps({"status": "ERROR", "message": "Invalid action"})
        except Exception as e:
            print(str(e))
            return json.dumps({"status": "ERROR", "message": str(e)})

    def SET_VALUE(connection, setpoint_value):
        worker = app_worker
        ui = raster_controls
        timeout_sec = 60
        if connection == "laser_raster_x_coord":
            start= time.time()
            worker.raster_manager.moveX(float(setpoint_value))
            while time.time() - start < timeout_sec:
                if worker.raster_manager.device_x.taskComplete == True:
                    return json.dumps({"status": "SUCCESS"})
                time.sleep(0.1)
            print("Timeout error when moving motor")
            return json.dumps({"status": "ERROR"})
        
        elif connection == "laser_raster_y_coord":
            start= time.time()
            worker.raster_manager.moveY(float(setpoint_value))
            while time.time() - start < timeout_sec:
                if worker.raster_manager.device_y.taskComplete == True:
                    return json.dumps({"status": "SUCCESS"})
                time.sleep(0.1)
            print("Timeout error when moving motor")
            return json.dumps({"status": "ERROR"})
        
        elif connection == "arm_raster":
            print("Received command to arm")
            ui.worker.running = True
            ui.thread = QThread(parent=ui)
            ui.worker.moveToThread(ui.thread)
            ui.thread.finished.connect(ui.worker.stop)
            ui.thread.start()

        elif connection == "move_to_next":    
            ui.worker.manual_work()
            start= time.time()
            while time.time() - start < timeout_sec:
                    if worker.raster_manager.device_x.taskComplete and worker.raster_manager.device_y.taskComplete:
                        return json.dumps({"status": "SUCCESS"})
                    time.sleep(0.1)
            print("Timeout error when moving motor")
            return json.dumps({"status": "ERROR"})
        
        elif connection == "disarm_raster":
            print("Received command to disarm")
            ui.worker.running = False
            ui.worker.mpl_instance.needs_update = True
            ui.thread.stop()

    def GET_VALUE(connection):
        worker = app_worker
        if connection == "laser_raster_x_coord_monitor" or connection == "laser_raster_x_coord":
            val = worker.raster_manager.get_current_x()
        elif connection == "laser_raster_y_coord_monitor" or connection == "laser_raster_y_coord":
            val = worker.raster_manager.get_current_y()
        return val

    while True:
        request = socket.recv()
        response = handle_request(request)
        socket.send(response.encode())

class CameraThread(QThread):
    new_frame = pyqtSignal(np.ndarray)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

        self.counter = 0
        self.time_emitted = 0

        ## Camera initialization
        self.hcam = ueye.HIDS(0)
        self.ret = ueye.is_InitCamera(self.hcam, None)
   
        # set the color mode.
        self.ret = ueye.is_SetColorMode(self.hcam, ueye.IS_CM_BGR8_PACKED)

        # set the region of interest (Camera dependent).
        self.width = 1280
        self.height = 1024
        rect_aoi = ueye.IS_RECT()
        rect_aoi.s32X = ueye.int(0)
        rect_aoi.s32Y = ueye.int(0)
        rect_aoi.s32Width = ueye.int(self.width)
        rect_aoi.s32Height = ueye.int(self.height)
        ueye.is_AOI(self.hcam, ueye.IS_AOI_IMAGE_SET_AOI, rect_aoi, ueye.sizeof(rect_aoi))

        # allocate memory for live view.
        self.mem_ptr = ueye.c_mem_p()
        self.mem_id = ueye.int()
        self.bitspixel = 24 # for colormode = IS_CM_BGR8_PACKED
        self.ret = ueye.is_AllocImageMem(self.hcam, self.width, self.height, self.bitspixel,
                                    self.mem_ptr, self.mem_id)
                
        # set active memory region.
        self.ret = ueye.is_SetImageMem(self.hcam, self.mem_ptr, self.mem_id)

        # continuous capture to memory.
        self.ret = ueye.is_CaptureVideo(self.hcam, ueye.IS_DONT_WAIT)
        self.lineinc = self.width * int((self.bitspixel + 7) / 8)

        # set initial exposure
        time_exposure_ = 3
        time_exposure = ueye.double(time_exposure_)
        self.ret = ueye.is_Exposure(self.hcam, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, time_exposure, ueye.sizeof(time_exposure))

    def run(self):
        while self.running:
                # Capture a frame from the camera
            img = ueye.get_data(self.mem_ptr, self.width, self.height, self.bitspixel, self.lineinc, copy=True)

            # Turn it into something readable
            img = np.reshape(img, (self.height,self.width,3))
            
            self.msleep(100)

            self.counter += 1

            self.new_frame.emit(img)

    def stop(self):
        self.running = False

class MplCanvas(QWidget):
    clicked = pyqtSignal(float, float)
    newScale = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        # Load the pixel positions and don't scale
        self.pixel_positions = []
        self.to_scale = False

        self.plotWidget = pg.PlotWidget(background='w')
        self.marker = [1, 1]
        self.have_bounds = False
        self.styles = {"color": "b", "font-size": "28px"}
        self.plotWidget.setLabel("left", "y (mm)", **self.styles)
        self.plotWidget.setLabel("bottom", "x (mm)", **self.styles)
        self.plotWidget.setAutoVisible(y=True)
        self.plotWidget.showGrid(x=True, y=True, alpha=0.3)
        lay = QVBoxLayout(self)
        lay.addWidget(self.plotWidget)
        self.scatter = pg.ScatterPlotItem(size=10)
        self.plotWidget.addItem(self.scatter)

        self.crosshair_v = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#632222d1"))
        self.crosshair_h = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen("#632222d1"))
        self.plotWidget.addItem(self.crosshair_v, ignoreBounds=True)
        self.plotWidget.addItem(self.crosshair_h, ignoreBounds=True)
        self.proxy = pg.SignalProxy(self.plotWidget.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)

        self.count = 0

        # set up initial scaling factor
        self.scale = 0.002

        # set up the image item
        self.img = None

        self.hull = []
        self.xmin = 100
        self.xmax = -100
        self.ymin = 100
        self.ymax = -100
        self.hull_scatter = pg.ScatterPlotItem(size=10)
        self.plotWidget.addItem(self.hull_scatter)
        self.plotWidget.scene().sigMouseClicked.connect(self.on_click)       
    
        self.calibrated = False
        self.calibration_scale = (1.0, 1.0)
        self.calibration_offset = (0.0, 0.0)

        # Set up imaging thread
        self.cam_thread = CameraThread() 
        self.cam_thread.new_frame.connect(self.update_frame) 
        self.cam_thread.start()

        self.time_plotted = time.time()

        self.oldframe = 0


    def mouseMoved(self, e):
        pos = e[0]
        if self.plotWidget.sceneBoundingRect().contains(pos):
            mousePoint = self.plotWidget.getPlotItem().vb.mapSceneToView(pos)
            mousecoord = "({:.4f}, {:.4f})".format(mousePoint.x(), mousePoint.y())
            self.plotWidget.setLabel("top", mousecoord, **self.styles)
            self.crosshair_v.setPos(mousePoint.x())
            self.crosshair_h.setPos(mousePoint.y())

    def update_plot(self):
        self.plotWidget.getPlotItem().listDataItems()[0].setBrush("#7894f2")
        self.plotWidget.getPlotItem().listDataItems()[0].setPen("#540808")
        self.scatter.addPoints([self.marker[0]], [self.marker[1]], brush=pg.mkBrush("#ff0000"))

    def update_frame(self, image_array):
        """
        Removes old image and adds new image to canvas
        """
        if image_array.ndim == 3 and image_array.shape[2] == 3:  # RGB
            h, w, c = image_array.shape
            q_image = QImage(image_array.data, w, h, 3 * w, QImage.Format_RGB888)
        elif image_array.ndim == 2:  # Grayscale image
            h, w = image_array.shape
            q_image = QImage(image_array.data, w, h, w, QImage.Format_Grayscale8)
        elif image_array.ndim == 3 and image_array.shape[2] == 4:  # RGBA
            h, w, c = image_array.shape
            q_image = QImage(image_array.data, w, h, 4 * w, QImage.Format_RGBA8888)

        # Convert the QImage to a QPixmap and set it as the label's pixmap
        pixmap = QPixmap.fromImage(q_image)

        oldimage = self.img

        if self.img is not None:
            # Remove the old image (if exists) from the plot
            self.plotWidget.getPlotItem().removeItem(self.img)

        # Add in the new frame
        self.img = QGraphicsPixmapItem(pixmap)
        self.img.setScale(self.scale)
        self.img.setRotation(0*180)
        # self.img.setOpacity(0.6)

        if oldimage == self.img:
            print("Same frame again")

        self.plotWidget.addItem(self.img)
        # print("Time to update: ", time.time() - self.time_plotted)  
        # self.time_plotted = time.time()

    def record_scale_point(self, pixel_x, pixel_y):
        """
        Records a scale point where the user clicks on the canvas.
        """
        # Store the pixel positions as pairs
        self.pixel_positions.append((pixel_x, pixel_y))
        print(f"Recorded Pixel: ({pixel_x}, {pixel_y})")

        # If we have two scale points, calculate the transformation matrix
        if len(self.pixel_positions) == 2:
            self.calculate_scale()

    def calculate_scale(self):
        """
        Calculate scaling for the image, based on user's input
        """
        (x1_pixel, y1_pixel), (x2_pixel, y2_pixel) = self.pixel_positions
        self.scale = self.scale/np.sqrt((x1_pixel - x2_pixel)**2 + (y1_pixel - y2_pixel)**2)
        self.newScale.emit(self.scale)

    def scaling(self):
        self.pixel_positions.clear()
        self.to_scale = True
        print("Click two points to scale...")
        
    def setscale(self, value):
        self.scale = value

    def setexposure(self, value):
        time_exposure = value
        time_exposure_ = ueye.double(time_exposure)
        ueye.is_Exposure(self.cam_thread.hcam, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, time_exposure_, ueye.sizeof(time_exposure_))

    def closeEvent(self, event):
        # Make sure to release the camera when the widget is closed
        ueye.is_StopLiveVideo(self.hcam, ueye.IS_FORCE_VIDEO_STOP)
        ueye.is_ExitCamera(self.hcam)
        event.accept()

    def display_bounds(self, x1, y1, x2, y2):
        if self.have_bounds:
            self.plotWidget.removeItem(self.rect_item)
        self.rect_item = RectItem(QtCore.QRectF(x1, y1, x2-x1, y2-y1))
        self.plotWidget.addItem(self.rect_item)
        self.have_bounds = True

    def on_click(self, e):
        if not self.to_scale:
            pos = e.scenePos()
            mousePoint = self.plotWidget.getPlotItem().vb.mapSceneToView(pos)
            x = mousePoint.x()
            y = mousePoint.y()
            self.hull_scatter.addPoints([x], [y], brush=pg.mkBrush("#c402cf"))
            self.hull.append([x, y])
            if x > self.xmax:
                self.xmax = x
            if y > self.ymax:
                self.ymax = y
            if x < self.xmin:
                self.xmin = x
            if y < self.ymin:
                self.ymin = y
            if x is None or y is None:
                return  # Ignore clicks outside axes
            self.clicked.emit(x, y)
            return
        else:
            pos = e.scenePos()
            mousePoint = self.plotWidget.getPlotItem().vb.mapSceneToView(pos)
            x = mousePoint.x()
            y = mousePoint.y()
            self.record_scale_point(x, y)
            if len(self.pixel_positions) >= 2:
                self.to_scale = False

    def plot_motor_bounds(self):
        pass


class RectItem(pg.GraphicsObject):
    def __init__(self, rect, parent=None):
        super().__init__(parent)
        self._rect = rect
        self.picture = QtGui.QPicture()
        self._generate_picture()

    @property
    def rect(self):
        return self._rect

    def _generate_picture(self):
        painter = QPainter(self.picture)
        painter.setPen(pg.mkPen("#cc6600"))
        painter.setBrush(pg.mkBrush("#ebce191a"))
        painter.drawRect(self.rect)
        painter.end()

    def paint(self, painter, option, widget=None):
        painter.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())

class Worker(QObject):
    finished = pyqtSignal()

    def __init__(self, mpl_instance, parent=None):
        QObject.__init__(self, parent=parent)

        self.mpl_instance = mpl_instance
        self.running = False

        self.log_data = []
        self.log_path = None

        global boundaries, xstep, ystep, saving_dir

        device_x = KCube("27268551", name="X")
        device_y = KCube("27268560", name="Y")
             
        self.raster_manager = ArrayPatternRasterX(device_x, device_y, boundaries=boundaries, xstep=xstep, ystep=ystep)
        
        self.sleep_timer = 2

    def auto_work(self):
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(os.getcwd(), f"raster_log_{timestamp_str}.json")
        self.log_data = []

        print(f"[LOGGING] Started logging to: {self.log_path}")

        while self.running:

            time.sleep(self.sleep_timer)

            self.raster_manager.update_motors()
            last_x = self.raster_manager.get_current_x()
            last_y = self.raster_manager.get_current_y()

            self.log_data.append({
            "timestamp": time.time(),
            "x": last_x,
            "y": last_y
            })

            self.mpl_instance.marker[0] = last_x
            self.mpl_instance.marker[1] = last_y
            self.mpl_instance.needs_update = False
            self.mpl_instance.update_plot()

        # Save JSON to disk
        try:
            with open(self.log_path, 'w') as f:
                json.dump(self.log_data, f, indent=2)
            print(f"[LOGGING] Saved to {self.log_path}")
        except Exception as e:
            print(f"[LOGGING ERROR] Could not write to {self.log_path}: {e}")
                    
        self.finished.emit()

    def manual_work(self):
        if self.running:
            self.raster_manager.update_motors()
            last_x = self.raster_manager.get_current_x()
            last_y = self.raster_manager.get_current_y()
            self.mpl_instance.marker[0] = last_x
            self.mpl_instance.marker[1] = last_y
            self.mpl_instance.needs_update = False
            self.mpl_instance.update_plot()

    def change_raster_algorithm(self, ind):
        try:
            algo = {0: "Square Raster X", 1: "Square Raster Y", 2: "Spiral Raster", 3: "Convex Hull Raster"}
            print(f"Changed algorithm to {algo[ind]}.")

            device_x = self.raster_manager.device_x
            device_y = self.raster_manager.device_y
            boundaries = self.raster_manager.boundaries
            xstep = self.raster_manager.xstep_size
            ystep = self.raster_manager.ystep_size
            x_direction = self.raster_manager.x_direction
            y_direction = self.raster_manager.y_direction
            
            scale_x = self.raster_manager.scale_x
            scale_y = self.raster_manager.scale_y
            offset_x = self.raster_manager.offset_x
            offset_y = self.raster_manager.offset_y
        
            if algo[ind] == "Square Raster X":
                self.raster_manager = ArrayPatternRasterX(device_x, device_y, boundaries, xstep, ystep)
            elif algo[ind] == "Square Raster Y":
                self.raster_manager = ArrayPatternRasterY(device_x, device_y, boundaries, xstep, ystep)
            elif algo[ind] == "Spiral Raster":
                self.raster_manager = SpiralRaster(device_x, device_y, boundaries, radius, step, alpha, del_alpha)
            elif algo[ind] == "Convex Hull Raster":
                self.raster_manager = ConvexHullRaster(device_x, device_y, boundaries, xstep, ystep)
            else:
                raise RuntimeWarning
            
            self.raster_manager.scale_x = scale_x
            self.raster_manager.scale_y = scale_y
            self.raster_manager.offset_x = offset_x
            self.raster_manager.offset_y = offset_y
        
            self.raster_manager.x_direction = x_direction
            self.raster_manager.y_direction = y_direction
        
        except Exception as e:
            print("Error:", e)

    def stop(self):
        self.running = False
        
    def update_marker(self):
        last_x = self.raster_manager.get_current_x()
        last_y = self.raster_manager.get_current_y()
        self.mpl_instance.marker[0] = last_x
        self.mpl_instance.marker[1] = last_y

    def setsleep(self, value):
        self.sleep_timer = value

class CalibrationManager(QObject):
    calibration_updated = pyqtSignal(object)

    def __init__(self, canvas, raster_manager, UI):
        super().__init__()

        self.canvas = canvas
        self.raster_manager = raster_manager
        self.ui = UI

        # Initialize the pixel and motor positions
        self.pixel_positions = []
        self.motor_positions = []

        self.to_calibrate = False

        self.save_path = "calibration_data.json"

        self.canvas.clicked.connect(self.handle_click)

        self.calibration_updated.connect(self.raster_manager.set_calibration)
        self.calibration_updated.connect(self.canvas.plot_motor_bounds)
        self.calibration_updated.connect(self.ui.show_calibration)
        self.calibration_updated.connect(self.ui.calibration_done_popup)


    def record_calibration_point(self, pixel_x, pixel_y):
        """
        Records a calibration point where the user clicks on the canvas.
        It also retrieves the corresponding motor position.
        """
        motor_x = self.raster_manager.get_current_x()  # Get current motor position in x
        motor_y = self.raster_manager.get_current_y()  # Get current motor position in y

        # Store the pixel and motor positions as pairs
        self.pixel_positions.append((pixel_x, pixel_y))
        self.motor_positions.append((motor_x, motor_y))

        # Debug: Print the values
        print(f"Recorded Pixel: ({pixel_x}, {pixel_y}), Motor: ({motor_x}, {motor_y})")

        # If we have two calibration points, calculate the transformation matrix
        if len(self.pixel_positions) == 2:
            self.calculate_calibration()

    def save_calibration(self):
        calibration_data = {
            "scale_x": self.scale_x,
            "scale_y": self.scale_y,
            "offset_x": self.offset_x,
            "offset_y": self.offset_y
        }
        with open(self.save_path, "w") as file:
            json.dump(calibration_data, file)
        print(f"Calibration saved to {self.save_path}.")

    def load_calibration(self):
        if os.path.exists(self.save_path):
            with open(self.save_path, "r") as file:
                calibration_data = json.load(file)
                self.scale_x = calibration_data["scale_x"]
                self.scale_y = calibration_data["scale_y"]
                self.offset_x = calibration_data["offset_x"]
                self.offset_y = calibration_data["offset_y"]

                # Apply these to the canvas
                self.canvas.calibration_scale = (self.scale_x, self.scale_y)
                self.canvas.calibration_offset = (self.offset_x, self.offset_y)
                self.canvas.calibrated = True
                
                # Also update the raster manager
                self.calibration_updated.emit(self)

            print(f"Calibration loaded from {self.save_path}.")
        else:
            print("No previous calibration found.")

    def calculate_calibration(self):
        """
        Calculates the transformation matrix based on the two calibration points.
        Assumes that self.pixel_positions and self.motor_positions each contain two points.
        """
        # Get the two points
        (x1_pixel, y1_pixel), (x2_pixel, y2_pixel) = self.pixel_positions
        (x1_motor, y1_motor), (x2_motor, y2_motor) = self.motor_positions

        # Calculate the scaling factors (pixel-to-motor conversion)
        scale_x = (x2_motor - x1_motor) / (x2_pixel - x1_pixel)
        scale_y = (y2_motor - y1_motor) / (y2_pixel - y1_pixel)

        # Calculate the offsets (starting motor position for given pixel position)
        offset_x = x1_motor - scale_x * x1_pixel
        offset_y = y1_motor - scale_y * y1_pixel

        # Save the scaling and offset parameters for future use
        self.scale_x = scale_x
        self.scale_y = scale_y
        self.offset_x = offset_x
        self.offset_y = offset_y

        # Debug: Print the calibration results
        print(f"Calibration Complete:")
        print(f"Scale X: {self.scale_x}, Scale Y: {self.scale_y}")
        print(f"Offset X: {self.offset_x}, Offset Y: {self.offset_y}")

        # self.canvas.calibration_scale = (scale_x, scale_y)
        # self.canvas.calibration_offset = (offset_x, offset_y)
        # self.canvas.calibrated = True

        # ## Find the minimum and maximum motor positions
        # self.xposmax = self.raster_manager.device_x.GetMaxPosition()
        # self.xposmin = self.raster_manager.device_x.GetMinPosition()
        # self.yposmax = self.raster_manager.device_y.GetMaxPosition()
        # self.yposmin = self.raster_manager.device_y.GetMinPosition()
        
        # self.xpixmax = (self.xposmax - self.offset_x) / self.scale_x
        # self.xpixmin = (self.xpixmin - self.offset_x) / self.scale_x
        # self.ypixmax = (self.yposmax - self.offset_y) / self.scale_y
        # self.ypixmin = (self.ypixmin - self.offset_y) / self.scale_y

        self.calibration_updated.emit(self)
        self.save_calibration()

    def calibration(self):
        self.pixel_positions.clear()
        self.motor_positions.clear()
        self.to_calibrate = True
        print("Click two points to calibrate...")

    def reset(self):
        self.scale_x = 1
        self.offset_x = 0
        self.scale_y = 1
        self.offset_y = 0

        # Debug: Print the calibration results
        print(f"Calibration Complete:")
        print(f"Scale X: {self.scale_x}, Scale Y: {self.scale_y}")
        print(f"Offset X: {self.offset_x}, Offset Y: {self.offset_y}")

        self.calibration_updated.emit(self)

    def setyoffset(self, value):
        self.offset_y = value

    def setyscale(self, value):
        self.scale_y = value

    def setxoffset(self, value):
        self.offset_x = value

    def setxscale(self, value):
        self.scale_x = value
    
    
    def setcalibration(self, ui):
        self.scale_x = ui.xscalevalue.value
        self.scale_y = ui.yscalevalue.value
        self.offset_x = ui.xoffsetvalue.value
        self.offset_y = ui.yoffsetvalue.value

    def handle_click(self, x, y):
        if not self.to_calibrate:
            return
        self.record_calibration_point(x, y)
        if len(self.pixel_positions) >= 2:
            self.to_calibrate = False

class UI(QMainWindow):
    stop_signal = pyqtSignal()
    exposureChanged = pyqtSignal(float)
    sleepSignal = pyqtSignal(float)
    scaleChanged = pyqtSignal(float)
    xscaleChanged = pyqtSignal(float)
    yscaleChanged = pyqtSignal(float)
    xoffsetChanged = pyqtSignal(float)
    yoffsetChanged = pyqtSignal(float)
    calibrateSignal = pyqtSignal()
    resetSignal = pyqtSignal()
    clearSignal = pyqtSignal()
    useoldSignal = pyqtSignal()
    scaleSignal = pyqtSignal()
    changeRasterAlgorithm = pyqtSignal(object)
    rasterPathSignal = pyqtSignal(list)

    def __init__(self):

        super().__init__()

        self.canvas = MplCanvas()
        self.worker = Worker(self.canvas)
        self.calibration_manager = CalibrationManager(self.canvas, self.worker.raster_manager, self)
        
        self.canvas.newScale.connect(self.show_scale)
        self.canvas.clicked.connect(self.handle_click)

        self.log_file = None
        self.log_writer = None

        self.have_paths = False
        self.have_moves = False
        self.have_hull = False
        self.conv_hull = []
        
        self.ui = uic.loadUi("raster_gui2.ui", self)
        self.canv_layout = self.findChild(QGridLayout, "gridLayout_2")
        self.canv_layout.addWidget(self.canvas, 430, 40, 721, 431)
        self.resize(2000, 2000)

        # Calibration Button
        self.calibrate = self.findChild(QPushButton, "calibrateButton")
        self.calibrate.clicked.connect(self.calibrateSignal.emit)
        self.calibrateSignal.connect(self.calibration_manager.calibration)
        self.calibrateSignal.connect(self.calibration_popup)

        # Reset Calibration Button
        self.reset = self.findChild(QPushButton, "resetButton")
        self.reset.clicked.connect(self.resetSignal.emit)
        self.resetSignal.connect(self.calibration_manager.reset)

        # Use Previous Calibration Button
        self.useold = self.findChild(QPushButton, "useold")
        self.useold.clicked.connect(self.useoldSignal.emit)
        self.useoldSignal.connect(self.calibration_manager.load_calibration)

        # Reset Plot Button
        self.clear = self.findChild(QPushButton, "clearAll")
        self.clear.clicked.connect(self.clearall)
        self.clear.clicked.connect(self.reset_hull)

        # Calibration Values
        self.yoffsetvalue = self.findChild(QDoubleSpinBox, "yoffset")
        self.yoffsetvalue.setValue(0)
        self.yoffsetvalue.valueChanged.connect(self.yoffsetChanged.emit)
        # self.yoffsetChanged.connect(self.calibration_manager.setyoffset)

        self.yscalevalue = self.findChild(QDoubleSpinBox, "yscale")
        self.yscalevalue.setValue(1)
        self.yscalevalue.valueChanged.connect(self.yscaleChanged.emit)
        # self.yoffsetChanged.connect(self.calibration_manager.setyscale)

        self.xoffsetvalue = self.findChild(QDoubleSpinBox, "xoffset")
        self.xoffsetvalue.setValue(0)
        self.xoffsetvalue.valueChanged.connect(self.xoffsetChanged.emit)
        # self.yoffsetChanged.connect(self.calibration_manager.setxoffset)

        self.xscalevalue = self.findChild(QDoubleSpinBox, "xscale")
        self.xscalevalue.setValue(1)
        self.xscalevalue.valueChanged.connect(self.xscaleChanged.emit)
        # self.yoffsetChanged.connect(self.calibration_manager.setxscale)
    
        # Image Scaler
        self.scaler = self.findChild(QDoubleSpinBox, "scaleImage")
        self.scaler.setValue(0.002)
        self.scaler.valueChanged.connect(self.scaleChanged.emit)
        self.scaleChanged.connect(self.canvas.setscale)

        # Exposure slider
        self.exposure = self.findChild(QDoubleSpinBox, "exposurevalue") 
        self.exposure.setValue(3)
        self.exposure.valueChanged.connect(self.exposureChanged.emit)
        self.exposureChanged.connect(self.canvas.setexposure)

        # Set the sleep timer
        self.sleep_value = self.findChild(QDoubleSpinBox, "sleepTimer")
        self.sleep_value.setValue(2)
        self.sleep_value.valueChanged.connect(self.sleepSignal.emit)
        self.sleepSignal.connect(self.worker.setsleep)


        # Set the scale
        self.scaleButton = self.findChild(QPushButton, "scaleButton")
        self.scaleButton.clicked.connect(self.scaleSignal.emit)
        self.scaleSignal.connect(self.canvas.scaling)

        # Bounds for square raster
        self.x_low_spinbox = self.findChild(QDoubleSpinBox, "xlow")
        self.y_low_spinbox = self.findChild(QDoubleSpinBox, "ylow")
        self.x_high_spinbox = self.findChild(QDoubleSpinBox, "xhigh")
        self.y_high_spinbox = self.findChild(QDoubleSpinBox, "yhigh")
        self.x_low_spinbox.valueChanged.connect(self.update_x_min)
        self.y_low_spinbox.valueChanged.connect(self.update_y_min)
        self.x_high_spinbox.valueChanged.connect(self.update_x_max)
        self.y_high_spinbox.valueChanged.connect(self.update_y_max)
        self.show_limits = self.findChild(QPushButton, "bound_button")
        self.show_limits.clicked.connect(self.display_limit)

        # Parameters for spiral raster
        self.radius = self.findChild(QDoubleSpinBox, "radius_spiral")
        self.step = self.findChild(QDoubleSpinBox, "step_spiral")
        self.delalpha = self.findChild(QDoubleSpinBox, "angle_spiral")
        self.delalpha_step = self.findChild(QDoubleSpinBox, "ang_change")
        self.radius.valueChanged.connect(self.update_r)
        self.step.valueChanged.connect(self.update_st)
        self.delalpha.valueChanged.connect(self.update_delalph)
        self.delalpha_step.valueChanged.connect(self.update_delalph_st)

        # Change raster algorithm
        self.dropbox = self.findChild(QComboBox, "alg_choice")
        self.dropbox.currentIndexChanged.connect(self.changeRasterAlgorithm.emit)
        self.changeRasterAlgorithm.connect(self.worker.change_raster_algorithm)

        # Raster control and parameters
        self.preview_button = self.findChild(QPushButton, "path_button")
        self.start_button = self.findChild(QPushButton, "start_button")
        self.stop_button = self.findChild(QPushButton, "stop_button")
        self.save_button = self.findChild(QPushButton, "save_button")
        self.preview_button.clicked.connect(self.preview_raster)
        self.start_button.clicked.connect(self.start_raster)
        self.stop_button.clicked.connect(self.stop_raster)
        self.save_button.clicked.connect(self.save_raster)
        self.xstep = self.findChild(QDoubleSpinBox, "xstep")
        self.ystep = self.findChild(QDoubleSpinBox, "ystep")
        self.xstep.valueChanged.connect(self.update_raster_step_x)
        self.ystep.valueChanged.connect(self.update_raster_step_y)

        # Homing motors
        self.homeX_button = self.findChild(QPushButton, "homeX_3")
        self.homeY_button = self.findChild(QPushButton, "homeY_3")
        self.homeX_button.clicked.connect(self.home_motorX)
        self.homeY_button.clicked.connect(self.home_motorY)

        # Jogging motors
        self.dx = self.findChild(QDoubleSpinBox, "dx_button")
        self.dy = self.findChild(QDoubleSpinBox, "dy_button")
        self.up_button = self.findChild(QPushButton, "jog_up_button_3")
        self.down_button = self.findChild(QPushButton, "jog_down_button_3")
        self.left_button = self.findChild(QPushButton, "jog_left_button_3")
        self.right_button = self.findChild(QPushButton, "jog_right_button_3")
        self.up_button.clicked.connect(self.jog_up)
        self.down_button.clicked.connect(self.jog_down)
        self.left_button.clicked.connect(self.jog_left)
        self.right_button.clicked.connect(self.jog_right)

        # Moving to set position
        self.x = self.findChild(QDoubleSpinBox, "x")
        self.y = self.findChild(QDoubleSpinBox, "y")
        self.move_to_button = self.findChild(QPushButton, "move_to_pos")
        self.preview_move_button = self.findChild(QPushButton, "preview_pos")
        self.move_to_button.clicked.connect(self.manual_move)
        self.preview_move_button.clicked.connect(self.preview_move)

        # Backlash correction
        self.backlash_x = self.findChild(QDoubleSpinBox, "x_backlash")
        self.backlash_y = self.findChild(QDoubleSpinBox, "y_backlash")
        self.backlash_x.valueChanged.connect(self.update_backlash_x)
        self.backlash_y.valueChanged.connect(self.update_backlash_y)
        
        self.update_backlash_x()
        self.update_backlash_y()

        self.show()
        time.sleep(0.5)
        self.startup_popup()

    def clearall(self):
        self.canvas.scatter.setData([])
        self.canvas.hull.clear()
        self.canvas.hull_scatter.setData([])
        if self.have_paths and hasattr(self.worker.mpl_instance, 'scatter_path'):
            self.worker.mpl_instance.scatter_path.setData([])
            self.have_paths = False

    def startup_popup(self):
        msg = QMessageBox(self)
        msg.setWindowTitle("Welcome!")
        msg.setText("Warning: motors are not calibrated")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def calibration_popup(self):
        msg = QMessageBox()
        msg.setWindowTitle("Calibration Message")
        msg.setText("Your next two on screen clicks will be recorded for calibration.\nPlease move laser to two arbitrary points, and click on laser location each time.")
        msg.setStandardButtons(QMessageBox.Ok)
        close = msg.exec_()

    def calibration_done_popup(self):
        msg = QMessageBox()
        msg.setWindowTitle("Calibration Complete")
        msg.setText("To reset calibration, hit Reset")
        msg.setStandardButtons(QMessageBox.Ok)
        close = msg.exec_()

    def show_calibration(self, calibration_manager):
        self.yoffsetvalue.setValue(calibration_manager.offset_y)
        self.yscalevalue.setValue(calibration_manager.scale_y)
        self.xoffsetvalue.setValue(calibration_manager.offset_x)
        self.xscalevalue.setValue(calibration_manager.scale_x)

    def show_scale(self, val): 
        self.scaler.setValue(val)

    def get_worker(self):
        return self.worker
    
    def handle_click(self, x, y):
        self.x.setValue(x)
        self.y.setValue(y)

    def reset_hull(self):
        if self.have_hull:
            self.canvas.hull = []
            self.canvas.hull_scatter.setData([])
            self.canvas.convexpath.clear()
            self.canvas.xmin = 100
            self.canvas.xmax = -100
            self.canvas.ymin = 100
            self.canvas.ymax = -100
            self.worker.raster_manager.rasterpath = [[],[]]
            self.have_hull = False

    def display_limit(self):
        global boundaries
        x1 = boundaries[0]
        x2 = boundaries[1]
        y1 = boundaries[2]
        y2 = boundaries[3]
        print("Current bounds: X {:.4f} - {:.4f} mm, Y {:.4f} - {:.4f} mm".format(x1, x2, y1, y2))
        self.canvas.display_bounds(x1, y1, x2, y2)
    
    def preview_raster(self):
        print("Previewing the path")
        if self.have_paths:
            self.worker.mpl_instance.scatter_path.setData([])
        path = self.worker.raster_manager.preview_path(self)

        print("Path previewed, x scale = ", self.worker.raster_manager.scale_x)
        print("Path previewed, y scale = ", self.worker.raster_manager.scale_y)
        print("Path previewed, x offset = ", self.worker.raster_manager.offset_x)
        print("Path previewed, y offset = ", self.worker.raster_manager.offset_y)

        self.worker.mpl_instance.scatter_path = pg.ScatterPlotItem(size=10)
        self.worker.mpl_instance.plotWidget.addItem(self.worker.mpl_instance.scatter_path)
        self.worker.mpl_instance.scatter_path.addPoints(path[0], path[1])
        self.worker.mpl_instance.scatter_path.setBrush("#9e93935c")
        self.worker.mpl_instance.scatter_path.setPen("#000000d1")
        self.worker.mpl_instance.scatter_path.setOpacity(0.5)
        self.have_paths = True
        print("Finished")
          
    def start_raster(self):
        print("Received command to start")
        self.worker.running = True
        self.make_threaded_worker()
        self.start_button.setEnabled(False)
        
    def stop_raster(self):
        self.stop_signal.emit()
        self.worker.mpl_instance.needs_update = True
        print("Received command to stop")
        self.start_button.setEnabled(True)
        self.worker.stop()
        
    def save_raster(self):
        if self.worker.running:
            print("The rastering is still running")
        else:
          self.canvas.scatter.clear()
    
    def home_motorX(self):
        try:
            print("Received command to home X")
            self.worker.raster_manager.homeX()
            last_x = self.worker.raster_manager.get_current_x()
            last_y = self.worker.raster_manager.get_current_y()
            # self.worker.mpl_instance.marker[0] = last_x
            # self.worker.mpl_instance.marker[1] = last_y
            # self.worker.mpl_instance.update_plot()
        except AttributeError:
            return False
  
    def home_motorY(self):
        try:
            print("Received command to home Y")
            self.worker.raster_manager.homeY()
            last_x = self.worker.raster_manager.get_current_x()
            last_y = self.worker.raster_manager.get_current_y()
            # self.worker.mpl_instance.marker[0] = last_x
            # self.worker.mpl_instance.marker[1] = last_y
            # self.worker.mpl_instance.update_plot()
        except AttributeError:
            return False
    
    def jog_up(self):
        try:
            print("Received command to jog up.")
            y = self.worker.raster_manager.get_current_y()
            dy = abs(self.dy.value())
            print("Jogging Y from {:.4f} to {:.4f}".format(y, y + dy))
            self.worker.raster_manager.moveY(y + dy)
            last_x = self.worker.raster_manager.get_current_x()
            last_y = self.worker.raster_manager.get_current_y()
            self.worker.mpl_instance.marker[0] = last_x
            self.worker.mpl_instance.marker[1] = last_y
            self.worker.mpl_instance.update_plot()
        except AttributeError:
            return False

    def jog_down(self):
        try:
            print("Received command to jog down.")
            y = self.worker.raster_manager.get_current_y()
            dy = abs(self.dy.value())
            print("Jogging Y from {:.4f} to {:.4f}".format(y, y - dy))
            self.worker.raster_manager.moveY(y - dy)
            last_x = self.worker.raster_manager.get_current_x()
            last_y = self.worker.raster_manager.get_current_y()
            self.worker.mpl_instance.marker[0] = last_x
            self.worker.mpl_instance.marker[1] = last_y
            self.worker.mpl_instance.update_plot()
        except AttributeError:
            return False

    def jog_left(self):
        try:
            print("Received command to jog left.")
            x = self.worker.raster_manager.get_current_x()
            dx = abs(self.dx.value())
            print("Jogging X from {:.4f} to {:.4f}".format(x, x - dx))
            self.worker.raster_manager.moveX(x - dx)
            last_x = self.worker.raster_manager.get_current_x()
            last_y = self.worker.raster_manager.get_current_y()
            self.worker.mpl_instance.marker[0] = last_x
            self.worker.mpl_instance.marker[1] = last_y
            self.worker.mpl_instance.update_plot()
        except AttributeError:
            return False

    def jog_right(self):
        try:
            print("Received command to jog right.")
            x = self.worker.raster_manager.get_current_x()
            dx = abs(self.dx.value())
            print("Jogging X from {:.4f} to {:.4f}".format(x, x + dx))
            self.worker.raster_manager.moveX(x + dx)
            last_x = self.worker.raster_manager.get_current_x()
            last_y = self.worker.raster_manager.get_current_y()
            self.worker.mpl_instance.marker[0] = last_x
            self.worker.mpl_instance.marker[1] = last_y
            self.worker.mpl_instance.update_plot()
        except AttributeError:
            return False

    def manual_move(self):
        try:
            print("Received command to move to ({:.4f}, {:.4f})".format(self.x.value(),  self.y.value()))
            self.worker.raster_manager.moveTo(self.x.value(), self.y.value())
            last_x = self.worker.raster_manager.get_current_x()
            last_y = self.worker.raster_manager.get_current_y()
            self.worker.mpl_instance.marker[0] = last_x
            self.worker.mpl_instance.marker[1] = last_y
            self.worker.mpl_instance.update_plot()
        except AttributeError:
            pass
        
    def preview_move(self):
        print("Previewing move to {:.4f}, {:.4f}".format(self.x.value(),  self.y.value()))
        if self.have_moves:
            self.worker.mpl_instance.moving_path.clear()
        move = self.worker.raster_manager.preview_move(self.x.value(), self.y.value())
        self.worker.mpl_instance.moving_path = pg.ScatterPlotItem(size=10)
        self.worker.mpl_instance.plotWidget.addItem(self.worker.mpl_instance.moving_path)
        self.worker.mpl_instance.moving_path.addPoints(move[0], move[1])
        self.worker.mpl_instance.moving_path.setBrush("#9e93935c")
        self.worker.mpl_instance.moving_path.setPen("#000000d1")
        self.worker.mpl_instance.moving_path.setOpacity(0.5)
    
    def update_x_min(self):
        global boundaries
        try:
            self.worker.do_raster = False
            v_old = self.worker.raster_manager.xlim_lo
            self.worker.raster_manager.update_x_low(self.x_low_spinbox.value())
            print("Changed x_min from {:.4f} to {:.4f}".format(v_old, self.x_low_spinbox.value()))
            x1 = self.x_low_spinbox.value()
            x2 = self.x_high_spinbox.value()
            y1 = self.y_low_spinbox.value()
            y2 = self.y_high_spinbox.value()
            boundaries = (x1, x2, y1, y2)
        except AttributeError:
            return False

    def update_x_max(self):
        global boundaries
        try:
            self.worker.do_raster = False
            v_old = self.worker.raster_manager.xlim_hi
            self.worker.raster_manager.update_x_high(self.x_high_spinbox.value())
            print("Changed x_max from {:.4f} to {:.4f}".format(v_old, self.x_high_spinbox.value()))
            x1 = self.x_low_spinbox.value()
            x2 = self.x_high_spinbox.value()
            y1 = self.y_low_spinbox.value()
            y2 = self.y_high_spinbox.value()
            boundaries = (x1, x2, y1, y2)
        except AttributeError:
            return False

    def update_y_min(self):
        global boundaries
        try:
            self.worker.do_raster = False
            v_old = self.worker.raster_manager.ylim_lo
            self.worker.raster_manager.update_y_low(self.y_low_spinbox.value())
            print("Changed y_min from {:.4f} to {:.4f}".format(v_old, self.y_low_spinbox.value()))
            x1 = self.x_low_spinbox.value()
            x2 = self.x_high_spinbox.value()
            y1 = self.y_low_spinbox.value()
            y2 = self.y_high_spinbox.value()
            boundaries = (x1, x2, y1, y2)
        except AttributeError:
            return False

    def update_y_max(self):
        global boundaries
        try:
            self.worker.do_raster = False
            v_old = self.worker.raster_manager.ylim_hi
            self.worker.raster_manager.update_y_high(self.y_high_spinbox.value())
            print("Changed y_max from {:.4f} to {:.4f}".format(v_old, self.y_high_spinbox.value()))
            x1 = self.x_low_spinbox.value()
            x2 = self.x_high_spinbox.value()
            y1 = self.y_low_spinbox.value()
            y2 = self.y_high_spinbox.value()
            boundaries = (x1, x2, y1, y2)
        except AttributeError:
            return False

    def update_raster_step_x(self):
        step_size_old_x = self.worker.raster_manager.xstep_size
        self.worker.raster_manager.update_step_size_x(self.xstep.value())
        step_size_new_x = self.worker.raster_manager.xstep_size
        print("Updated raster step size x from {:.4f} to {:.4f}".format(step_size_old_x, step_size_new_x))\
            
    def update_raster_step_y(self):
        step_size_old_y = self.worker.raster_manager.ystep_size
        self.worker.raster_manager.update_step_size_y(self.ystep.value())
        step_size_new_y = self.worker.raster_manager.ystep_size
        print("Updated raster step size y from {:.4f} to {:.4f}".format(step_size_old_y, step_size_new_y))
        
    def update_r(self):
        radius_old = self.worker.raster_manager.spiral_rad
        self.worker.raster_manager.update_radius(self.radius.value())
        radius_new = self.worker.raster_manager.spiral_rad
        print("Updated radius from {:.4f} to {:.4f}".format(radius_old, radius_new))
        
    def update_st(self):
        step_old = self.worker.raster_manager.spiral_step
        self.worker.raster_manager.update_spiral_step(self.step.value())
        step_new = self.worker.raster_manager.spiral_step
        print("Updated spiral step from {:.4f} to {:.4f}".format(step_old, step_new))
        
    def update_delalph(self):
        dela_old = self.worker.raster_manager.angle_step
        self.worker.raster_manager.update_angle_step(self.delalpha.value())
        dela_new = self.worker.raster_manager.angle_step
        print("Updated angle step from {:.4f} to {:.4f}".format(dela_old, dela_new))
        
    def update_delalph_st(self):
        dela_step_old = self.worker.raster_manager.angle_step_change
        self.worker.raster_manager.update_angle_step_change(self.delalpha_step.value())
        dela_step_new = self.worker.raster_manager.angle_step_change
        print("Updated angle step change from {:.4f} to {:.4f}".format(dela_step_old, dela_step_new))
        
    def update_backlash_x(self):
        xback = Decimal(float(self.backlash_x.value()))
        self.worker.raster_manager.update_backlash_on_x(xback)
        new_backlash_x = self.backlash_x.value()
        print("Updated backlash on x to {:.3f} mm".format(new_backlash_x))
        
    def update_backlash_y(self):
        yback = Decimal(float(self.backlash_y.value()))
        self.worker.raster_manager.update_backlash_on_y(yback)
        new_backlash_y = self.backlash_y.value()
        print("Updated backlash on y to {:.3f} mm".format(new_backlash_y))
    
    def make_threaded_worker(self):
        self.thread = QThread(parent=self)
        self.stop_signal.connect(self.worker.stop)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.auto_work)
        self.thread.finished.connect(self.worker.stop)

        print("Starting Auto Raster, x scale = ", self.worker.raster_manager.scale_x)
        print("Starting Auto Raster, y scale = ", self.worker.raster_manager.scale_y)
        print("Starting Auto Raster, x offset = ", self.worker.raster_manager.offset_x)
        print("Starting Auto Raster, y offset = ", self.worker.raster_manager.offset_y)
        
        self.thread.start()

if __name__ == '__main__':
    global boundaries, xstep, ystep, saving_dir
    
    boundaries = (0.0, 12.0, 0.0, 12.0)
    xstep = 0.05
    ystep = 0.05
    radius = 0.05 
    step = 0.008
    alpha = 0.1
    del_alpha = 0.05
    
    app = QApplication(sys.argv)    
    UIWindow = UI()

    server_thread = threading.Thread(target=start_server, args=(UIWindow.get_worker(),UIWindow,))
    server_thread.daemon = True
    server_thread.start()

    sys.exit(app.exec_())
    