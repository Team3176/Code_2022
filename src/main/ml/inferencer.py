import argparse

import cv2
import numpy as np
from time import time
import tflite_runtime.interpreter as tflite
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTables, NetworkTablesInstance
import cv2
import collections
import json
import sys


class ConfigParser:
    def __init__(self, config_path):
        self.team = -1

        # parse file
        try:
            with open(config_path, "rt", encoding="utf-8") as f:
                j = json.load(f)
        except OSError as err:
            print("could not open '{}': {}".format(config_path, err), file=sys.stderr)

        # top level must be an object
        if not isinstance(j, dict):
            self.parseError("must be JSON object", config_path)

        # team number
        try:
            self.team = j["team"]
        except KeyError:
            self.parseError("could not read team number", config_path)

        # cameras
        try:
            self.cameras = j["cameras"]
        except KeyError:
            self.parseError("could not read cameras", config_path)

    def parseError(self, str, config_file):
        """Report parse error."""
        print("config error in '" + config_file + "': " + str, file=sys.stderr)


class PBTXTParser:
    def __init__(self, path):
        self.path = path
        self.file = None

    def parse(self):
        with open(self.path, 'r') as f:
            self.file = ''.join([i.replace('item', '') for i in f.readlines()])
            blocks = []
            obj = ""
            for i in self.file:
                if i == '}':
                    obj += i
                    blocks.append(obj)
                    obj = ""
                else:
                    obj += i
            self.file = blocks
            label_map = []
            for obj in self.file:
                obj = [i for i in obj.split('\n') if i]
                name = obj[2].split()[1][1:-1]
                label_map.append(name)
            self.file = label_map

    def get_labels(self):
        return self.file


class BBox(collections.namedtuple('BBox', ['xmin', 'ymin', 'xmax', 'ymax'])):
    """Bounding box.
    Represents a rectangle which sides are either vertical or horizontal, parallel
    to the x or y axis.
    """
    __slots__ = ()

    def scale(self, sx, sy):
        """Returns scaled bounding box."""
        return BBox(xmin=sx * self.xmin,
                    ymin=sy * self.ymin,
                    xmax=sx * self.xmax,
                    ymax=sy * self.ymax)


class Tester:
    def __init__(self, config_parser):
        print("Initializing TFLite runtime interpreter")
        try:
            model_path = "model.tflite"
            self.interpreter = tflite.Interpreter(model_path, experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
            self.hardware_type = "Coral Edge TPU"
        except:
            print("Failed to create Interpreter with Coral, switching to unoptimized")
            model_path = "unoptimized.tflite"
            self.interpreter = tflite.Interpreter(model_path)
            self.hardware_type = "Unoptimized"

        self.interpreter.allocate_tensors()

        print("Getting labels")
        parser = PBTXTParser("map.pbtxt")
        parser.parse()
        #self.labels = parser.get_labels()
        self.labels = ['red', 'blue', 'invalid']

        print("Connecting to Network Tables")
        ntinst = NetworkTablesInstance.getDefault()
        ntinst.startClientTeam(config_parser.team)
        #ntinst.startDSClient()
        self.entry = ntinst.getTable("ML").getEntry("detections")

        self.coral_entry = ntinst.getTable("ML").getEntry("coral")
        self.fps_entry = ntinst.getTable("ML").getEntry("fps")
        self.color_entry = ntinst.getTable("ML").getEntry("color")
        self.xmin_entry = ntinst.getTable("ML").getEntry("xmin")
        self.xmax_entry = ntinst.getTable("ML").getEntry("xmax")
        self.width_entry = ntinst.getTable("ML").getEntry("width")
        self.height_entry = ntinst.getTable("ML").getEntry("height")
        self.resolution_entry = ntinst.getTable("ML").getEntry("resolution")
        self.temp_entry = []

        print("Starting camera server")
        #cs = CameraServer.getInstance()
        #camera = cs.startAutomaticCapture()
        camera_config = config_parser.cameras[0]
        WIDTH, HEIGHT = camera_config["width"], camera_config["height"]
        #camera.setResolution(WIDTH, HEIGHT)
        #self.cvSink = cs.getVideo()
        self.cvSink = cv2.VideoCapture(0)
        self.cvSink.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.cvSink.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        WIDTH = 680
        self.width_entry.setNumber(WIDTH)
        HEIGHT = 420
        self.height_entry.setNumber(HEIGHT)
        self.img = np.zeros(shape=(HEIGHT, WIDTH, 3), dtype=np.uint8)
        #self.output = cs.putVideo("Axon", WIDTH, HEIGHT)
        self.frames = 0

        self.coral_entry.setString(self.hardware_type)
        self.resolution_entry.setString(str(WIDTH) + ", " + str(HEIGHT))

    def getAverages(self, yMin, xMin, yMax, xMax, image): #HOW IS THIS A SYNTAX ERROR AAAAAAAAAAA
        box = image
        avg = [0,0,0]
        numPixels = 0
        ySize = yMax - yMin
        xSize = xMax - xMin
        i = 0
        #print("box:"+box)
        while i < (xSize-2):
            c = 0
            while c < (ySize-3) :
                if (c < ySize-2 and i < xSize-2):
                    #print("ymmin:"+str(yMin)+" ymax:"+str(yMax)+" xmin:"+str(xMin)+" xmax:"+str(xMax)+" i:"+str(i)+" c:"+str(c))
                    #print("box len i: "+str(len(box))+"    box len c"+str(len(box[i])))
                    if (i < len(box) and c < len(box[i])):
                        color = box[i, c] #BGR
                        avg[0] += color[2] #Red
                        avg[1] += color[1] #Green
                        avg[2] += color[0] #Blue
                        #print(str(i)+" "+str(c)+" "+str(avg[0])+ " "+str(avg[1])+" "+str(avg[2]))
                        numPixels += 1

                c += 1
            i += 1
        if (len(avg) > 0 and numPixels > 0):
            avg[0] = avg[0] / numPixels
            avg[1] = avg[1] / numPixels
            avg[2] = avg[2] / numPixels

        print(avg)
        return avg


    def isWithinTolerance(self, arr1, arr2, tolerance):
        for i in range(len(arr1)):
            if abs(arr1[i] - arr2[i]) > tolerance[i]:
                return False
        return True


    def run(self):
        print("Starting mainloop")
        while True:
            start = time()
            # Acquire frame and resize to expected shape [1xHxWx3]
            #ret, frame_cv2 = self.cvSink.grabFrame(self.img)
            ret, frame_cv2 = self.cvSink.read()
            if not ret:
                print("Image failed")
                continue

            # input
            scale = self.set_input(frame_cv2)

            # run inference
            self.interpreter.invoke()

            # output
            boxes, class_ids, scores, x_scale, y_scale = self.get_output(scale)
            #print(len(boxes))
            for i in range(len(boxes)):
                if scores[i] > 0.5:
                    ymin, xmin, ymax, xmax = boxes[i]

                    bbox = BBox(xmin=xmin,
                                ymin=ymin,
                                xmax=xmax,
                                ymax=ymax).scale(x_scale, y_scale)
                    ymin, xmin, ymax, xmax = int(bbox.ymin), int(bbox.xmin), int(bbox.ymax), int(bbox.xmax)

                    zoomAmount = 0.5
                    width = xmax - xmin
                    height = ymax - ymin
                    xmin = int(xmin + width * (zoomAmount * 0.5))
                    ymin = int(ymin + height * (zoomAmount * 0.5))
                    xmax = int(xmax - width * (zoomAmount * 0.5))
                    ymax = int(ymax - height * (zoomAmount * 0.5))

                    height, width, channels = frame_cv2.shape

                    if not 0 <= ymin < ymax <= height or not 0 <= xmin < xmax <= width:
                        print('invalid inner box size')
                        print(xmin, xmax, ymin, ymax)
                        continue

                    red = [225,  55, 105]
                    #red = [0, 0, 0]
                    redtolerance = [40, 40, 40]
                    blue = [62, 155,  220]
                    bluetolerance = [40, 40, 40]

                    cropped = frame_cv2[ymin:ymax, xmin: xmax]
                    averages = self.getAverages(ymin, xmin, ymax, xmax, cropped)
                    #averages = np.average(cropped, axis=(0, 1))

                    if self.isWithinTolerance(red, averages, redtolerance):
                        class_ids[i] = 0
                        cv2.rectangle(frame_cv2, (xmin, ymin), (xmax, ymax), red, 2)
                        self.color_entry.setString("red")
                        self.xmin_entry.setNumber(xmin)
                        self.xmax_entry.setNumber(xmax)
                    elif self.isWithinTolerance(blue, averages, bluetolerance):
                        class_ids[i] = 1
                        cv2.rectangle(frame_cv2, (xmin, ymin), (xmax, ymax), blue, 2)
                        self.color_entry.setString("blue")
                        self.xmin_entry.setNumber(xmin)
                        self.xmax_entry.setNumber(xmax)
                    else:
                        class_ids[i] = 2
                        cv2.rectangle(frame_cv2, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                        self.color_entry.setString("invalid")
                        self.xmin_entry.setNumber(xmin)
                        self.xmax_entry.setNumber(xmax)

            for i in range(len(boxes)):
                if scores[i] > .5:

                    class_id = class_ids[i]
                    if np.isnan(class_id):
                        continue

                    class_id = int(class_id)
                    if class_id not in range(len(self.labels)):
                        continue

                    frame_cv2 = self.label_frame(frame_cv2, self.labels[class_id], boxes[i], scores[i], x_scale, y_scale)
                #if scores[i] < .5:
                #  self.color_entry.setString("null")

            #self.output.putFrame(frame_cv2)
            if self.frames % 2 == 0:
              self.entry.setString(json.dumps(self.temp_entry))
              self.temp_entry = []
              self.color_entry.setString("null")
              self.xmin_entry.setNumber(0)
              self.xmax_entry.setNumber(0)
            if self.frames % 100 == 0:
                print("Completed", self.frames, "frames. FPS:", (1 / (time() - start)))
            if self.frames % 10 == 0:
                self.fps_entry.setNumber((1 / (time() - start)))
            self.frames += 1

    def label_frame(self, frame, object_name, box, score, x_scale, y_scale):
        ymin, xmin, ymax, xmax = box
        score = float(score)
        bbox = BBox(xmin=xmin,
                    ymin=ymin,
                    xmax=xmax,
                    ymax=ymax).scale(x_scale, y_scale)

        height, width, channels = frame.shape
        # check bbox validity
        if not 0 <= ymin < ymax <= height or not 0 <= xmin < xmax <= width:
            return frame

        ymin, xmin, ymax, xmax = int(bbox.ymin), int(bbox.xmin), int(bbox.ymax), int(bbox.xmax)
        self.temp_entry.append({"label": object_name, "box": {"ymin": ymin, "xmin": xmin, "ymax": ymax, "xmax": xmax},
                                "confidence": score})

        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 4)

        # Draw label
        # Look up object name from "labels" array using class index
        label = '%s: %d%%' % (object_name, score * 100)  # Example: 'person: 72%'
        label_size, base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
        label_ymin = max(ymin, label_size[1] + 10)  # Make sure not to draw label too close to top of window
        cv2.rectangle(frame, (xmin, label_ymin - label_size[1] - 10), (xmin + label_size[0], label_ymin + base - 10),
                      (255, 255, 255), cv2.FILLED)
        # Draw label text
        cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        return frame

    def input_size(self):
        """Returns input image size as (width, height) tuple."""
        _, height, width, _ = self.interpreter.get_input_details()[0]['shape']
        return width, height

    def set_input(self, frame):
        """Copies a resized and properly zero-padded image to the input tensor.
        Args:
          frame: image
        Returns:
          Actual resize ratio, which should be passed to `get_output` function.
        """
        width, height = self.input_size()
        h, w, _ = frame.shape
        new_img = np.reshape(cv2.resize(frame, (300, 300)), (1, 300, 300, 3))
        self.interpreter.set_tensor(self.interpreter.get_input_details()[0]['index'], np.copy(new_img))
        return width / w, height / h

    def output_tensor(self, i):
        """Returns output tensor view."""
        tensor = self.interpreter.get_tensor(self.interpreter.get_output_details()[i]['index'])
        return np.squeeze(tensor)

    def get_output(self, scale):
        boxes = self.output_tensor(0)
        class_ids = self.output_tensor(1)
        scores = self.output_tensor(2)

        width, height = self.input_size()
        image_scale_x, image_scale_y = scale
        x_scale, y_scale = width / image_scale_x, height / image_scale_y
        return boxes, class_ids, scores, x_scale, y_scale


if __name__ == '__main__':
    config_file = "/boot/frc.json"
    config_parser = ConfigParser(config_file)
    tester = Tester(config_parser)
    tester.run()
