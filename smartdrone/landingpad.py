from PIL import Image
import cv2
import math
import numpy as np

CATEGORY_NUM = 2

class PreprocessYOLO(object):
    """A simple class for loading images with PIL and reshaping them to the specified
    input resolution for YOLOv3-608.
    """

    def __init__(self, yolo_input_resolution):
        """Initialize with the input resolution for YOLOv3, which will stay fixed in this sample.

        Keyword arguments:
        yolo_input_resolution -- two-dimensional tuple with the target network's (spatial)
        input resolution in HW order
        """
        self.yolo_input_resolution = yolo_input_resolution

    def process(self, image_raw):
        """Load an image PIL.Image,
        and return it together with a pre-processed version required for feeding it into a
        YOLOv3 network.

        Keyword arguments:
        input_image_path -- string path of the image to be loaded
        """
        image_raw, image_resized = self._load_and_resize_img(image_raw)
        image_preprocessed = self._shuffle_and_normalize(image_resized)
        return image_raw, image_preprocessed

    def _load_and_resize(self, input_image_path):
        """Load an image from the specified path and resize it to the input resolution.
        Return the input image before resizing as a PIL Image (required for visualization),
        and the resized image as a NumPy float array.

        Keyword arguments:
        input_image_path -- string path of the image to be loaded
        """

        image_raw = Image.open(input_image_path)
        # Expecting yolo_input_resolution in (height, width) format, adjusting to PIL
        # convention (width, height) in PIL:
        new_resolution = (
            self.yolo_input_resolution[1],
            self.yolo_input_resolution[0])
        image_resized = image_raw.resize(
            new_resolution, resample=Image.BICUBIC)
        image_resized = np.array(image_resized, dtype=np.float32, order='C')
        return image_raw, image_resized
    
    def _load_and_resize_img(self, image_raw):
        """Load an image from the specified path and resize it to the input resolution.
        Return the input image before resizing as a PIL Image (required for visualization),
        and the resized image as a NumPy float array.

        Keyword arguments:
        input_image_path -- string path of the image to be loaded
        """

#         image_raw = Image.open(input_image_path)
        # Expecting yolo_input_resolution in (height, width) format, adjusting to PIL
        # convention (width, height) in PIL:
        new_resolution = (
            self.yolo_input_resolution[1],
            self.yolo_input_resolution[0])
        image_resized = image_raw.resize(
            new_resolution, resample=Image.BICUBIC)
        image_resized = np.array(image_resized, dtype=np.float32, order='C')
        return image_raw, image_resized

    def _shuffle_and_normalize(self, image):
        """Normalize a NumPy array representing an image to the range [0, 1], and
        convert it from HWC format ("channels last") to NCHW format ("channels first"
        with leading batch dimension).

        Keyword arguments:
        image -- image as three-dimensional NumPy float array, in HWC format
        """
        image /= 255.0
        # HWC to CHW format:
        image = np.transpose(image, [2, 0, 1])
        # CHW to NCHW format
        image = np.expand_dims(image, axis=0)
        # Convert the image to row-major order, also known as "C order":
        image = np.array(image, dtype=np.float32, order='C')
        return image
    
class PostprocessYOLO(object):
    """Class for post-processing the three outputs tensors from YOLOv3-608."""

    def __init__(self,
                 yolo_masks,
                 yolo_anchors,
                 obj_threshold,
                 nms_threshold,
                 yolo_input_resolution):
        """Initialize with all values that will be kept when processing several frames.
        Assuming 3 outputs of the network in the case of (large) YOLOv3.

        Keyword arguments:
        yolo_masks -- a list of 3 three-dimensional tuples for the YOLO masks
        yolo_anchors -- a list of 9 two-dimensional tuples for the YOLO anchors
        object_threshold -- threshold for object coverage, float value between 0 and 1
        nms_threshold -- threshold for non-max suppression algorithm,
        float value between 0 and 1
        input_resolution_yolo -- two-dimensional tuple with the target network's (spatial)
        input resolution in HW order
        """
        self.masks = yolo_masks
        self.anchors = yolo_anchors
        self.object_threshold = obj_threshold
        self.nms_threshold = nms_threshold
        self.input_resolution_yolo = yolo_input_resolution

    def process(self, outputs, resolution_raw):
        """Take the YOLOv3 outputs generated from a TensorRT forward pass, post-process them
        and return a list of bounding boxes for detected object together with their category
        and their confidences in separate lists.

        Keyword arguments:
        outputs -- outputs from a TensorRT engine in NCHW format
        resolution_raw -- the original spatial resolution from the input PIL image in WH order
        """
        outputs_reshaped = list()
        for output in outputs:
            outputs_reshaped.append(self._reshape_output(output))

        boxes, categories, confidences = self._process_yolo_output(
            outputs_reshaped, resolution_raw)

        return boxes, categories, confidences

    def _reshape_output(self, output):
        """Reshape a TensorRT output from NCHW to NHWC format (with expected C=255),
        and then return it in (height,width,3,85) dimensionality after further reshaping.

        Keyword argument:
        output -- an output from a TensorRT engine after inference
        """
        output = np.transpose(output, [0, 2, 3, 1])
        _, height, width, _ = output.shape
        dim1, dim2 = height, width
        dim3 = 3
        # There are CATEGORY_NUM=80 object categories:
        dim4 = (4 + 1 + CATEGORY_NUM)
        return np.reshape(output, (dim1, dim2, dim3, dim4))

    def _process_yolo_output(self, outputs_reshaped, resolution_raw):
        """Take in a list of three reshaped YOLO outputs in (height,width,3,85) shape and return
        return a list of bounding boxes for detected object together with their category and their
        confidences in separate lists.

        Keyword arguments:
        outputs_reshaped -- list of three reshaped YOLO outputs as NumPy arrays
        with shape (height,width,3,85)
        resolution_raw -- the original spatial resolution from the input PIL image in WH order
        """

        # E.g. in YOLOv3-608, there are three output tensors, which we associate with their
        # respective masks. Then we iterate through all output-mask pairs and generate candidates
        # for bounding boxes, their corresponding category predictions and their confidences:
        boxes, categories, confidences = list(), list(), list()
        for output, mask in zip(outputs_reshaped, self.masks):
            box, category, confidence = self._process_feats(output, mask)
            box, category, confidence = self._filter_boxes(box, category, confidence)
            boxes.append(box)
            categories.append(category)
            confidences.append(confidence)

        boxes = np.concatenate(boxes)
        categories = np.concatenate(categories)
        confidences = np.concatenate(confidences)

        # Scale boxes back to original image shape:
        width, height = resolution_raw
        image_dims = [width, height, width, height]
        boxes = boxes * image_dims

        # Using the candidates from the previous (loop) step, we apply the non-max suppression
        # algorithm that clusters adjacent bounding boxes to a single bounding box:
        nms_boxes, nms_categories, nscores = list(), list(), list()
        for category in set(categories):
            idxs = np.where(categories == category)
            box = boxes[idxs]
            category = categories[idxs]
            confidence = confidences[idxs]

            keep = self._nms_boxes(box, confidence)

            nms_boxes.append(box[keep])
            nms_categories.append(category[keep])
            nscores.append(confidence[keep])

        if not nms_categories and not nscores:
            return None, None, None

        boxes = np.concatenate(nms_boxes)
        categories = np.concatenate(nms_categories)
        confidences = np.concatenate(nscores)

        return boxes, categories, confidences

    def _process_feats(self, output_reshaped, mask):
        """Take in a reshaped YOLO output in height,width,3,85 format together with its
        corresponding YOLO mask and return the detected bounding boxes, the confidence,
        and the class probability in each cell/pixel.

        Keyword arguments:
        output_reshaped -- reshaped YOLO output as NumPy arrays with shape (height,width,3,85)
        mask -- 2-dimensional tuple with mask specification for this output
        """

        # Two in-line functions required for calculating the bounding box
        # descriptors:
        def sigmoid(value):
            """Return the sigmoid of the input."""
            return 1.0 / (1.0 + math.exp(-value))

        def exponential(value):
            """Return the exponential of the input."""
            return math.exp(value)
        
        def np_sigmoid(arr):
            return 1.0 / (1.0 + np.exp(-arr))
        
        def np_exponential(arr):
            return np.exp(arr)

        # Vectorized calculation of above two functions:
        sigmoid_v = np.vectorize(sigmoid)
        exponential_v = np.vectorize(exponential)

        grid_h, grid_w, _, _ = output_reshaped.shape
        #print(grid_h, grid_w)

        anchors = [self.anchors[i] for i in mask]

        # Reshape to N, height, width, num_anchors, box_params:
        anchors_tensor = np.reshape(anchors, [1, 1, len(anchors), 2])
#         box_xy = sigmoid_v(output_reshaped[..., :2])
#         box_wh = exponential_v(output_reshaped[..., 2:4]) * anchors_tensor
#         box_confidence = sigmoid_v(output_reshaped[..., 4])

#         box_confidence = np.expand_dims(box_confidence, axis=-1)
#         box_class_probs = sigmoid_v(output_reshaped[..., 5:])
        
        box_xy = np_sigmoid(output_reshaped[..., :2])
        box_wh = np_exponential(output_reshaped[..., 2:4]) * anchors_tensor
        box_confidence = np_sigmoid(output_reshaped[..., 4])
        box_confidence = np.expand_dims(box_confidence, axis=-1)
        box_class_probs = np_sigmoid(output_reshaped[..., 5:])

#         col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
#         row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
        col = np.tile(np.arange(0, grid_w), grid_h).reshape(-1, grid_w)
        row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_w)

        col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        grid = np.concatenate((col, row), axis=-1)

        box_xy += grid
        box_xy /= (grid_w, grid_h)
#         box_wh /= self.input_resolution_yolo
        box_wh /= (self.input_resolution_yolo[1], self.input_resolution_yolo[0])
        box_xy -= (box_wh / 2.)
        boxes = np.concatenate((box_xy, box_wh), axis=-1)

        # boxes: centroids, box_confidence: confidence level, box_class_probs:
        # class confidence
        return boxes, box_confidence, box_class_probs

    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        """Take in the unfiltered bounding box descriptors and discard each cell
        whose score is lower than the object threshold set during class initialization.

        Keyword arguments:
        boxes -- bounding box coordinates with shape (height,width,3,4); 4 for
        x,y,height,width coordinates of the boxes
        box_confidences -- bounding box confidences with shape (height,width,3,1); 1 for as
        confidence scalar per element
        box_class_probs -- class probabilities with shape (height,width,3,CATEGORY_NUM)

        """
        box_scores = box_confidences * box_class_probs
        box_classes = np.argmax(box_scores, axis=-1)
        box_class_scores = np.max(box_scores, axis=-1)
        pos = np.where(box_class_scores >= self.object_threshold)

        boxes = boxes[pos]
        classes = box_classes[pos]
        scores = box_class_scores[pos]

        return boxes, classes, scores

    def _nms_boxes(self, boxes, box_confidences):
        """Apply the Non-Maximum Suppression (NMS) algorithm on the bounding boxes with their
        confidence scores and return an array with the indexes of the bounding boxes we want to
        keep (and display later).

        Keyword arguments:
        boxes -- a NumPy array containing N bounding-box coordinates that survived filtering,
        with shape (N,4); 4 for x,y,height,width coordinates of the boxes
        box_confidences -- a Numpy array containing the corresponding confidences with shape N
        """
        x_coord = boxes[:, 0]
        y_coord = boxes[:, 1]
        width = boxes[:, 2]
        height = boxes[:, 3]

        areas = width * height
        ordered = box_confidences.argsort()[::-1]

        keep = list()
        while ordered.size > 0:
            # Index of the current element:
            i = ordered[0]
            keep.append(i)
            xx1 = np.maximum(x_coord[i], x_coord[ordered[1:]])
            yy1 = np.maximum(y_coord[i], y_coord[ordered[1:]])
            xx2 = np.minimum(x_coord[i] + width[i], x_coord[ordered[1:]] + width[ordered[1:]])
            yy2 = np.minimum(y_coord[i] + height[i], y_coord[ordered[1:]] + height[ordered[1:]])

            width1 = np.maximum(0.0, xx2 - xx1 + 1)
            height1 = np.maximum(0.0, yy2 - yy1 + 1)
            intersection = width1 * height1
            union = (areas[i] + areas[ordered[1:]] - intersection)

            # Compute the Intersection over Union (IoU) score:
            iou = intersection / union

            # The goal of the NMS algorithm is to reduce the number of adjacent bounding-box
            # candidates to a minimum. In this step, we keep only those elements whose overlap
            # with the current bounding box is lower than the threshold:
            indexes = np.where(iou <= self.nms_threshold)[0]
            ordered = ordered[indexes + 1]

        keep = np.array(keep)
        return keep


# Output shapes expected by the post-processor
output_shapes = [(1, 21, 19, 33), (1, 21, 38, 66), (1, 21, 76, 132)] # CORRECT REAL 608x800 numpy image array. 
# Do inference with TensorRT
trt_outputs = []


# Create a pre-processor object by specifying the required input resolution for YOLOv3
input_resolution_yolov3_HW = (608, 1056)
preprocessor = PreprocessYOLO(input_resolution_yolov3_HW)


postprocessor_args = {"yolo_masks": [(6, 7, 8), (3, 4, 5), (0, 1, 2)],                    # A list of 3 three-dimensional tuples for the YOLO masks
                      "yolo_anchors": [(8, 8), (12, 12), (15, 15), (20, 20), (27, 27),  # A list of 9 two-dimensional tuples for the YOLO anchors
                                       (42, 43), (71, 71), (124, 124), (256, 251)],
                      "obj_threshold": 0.1,                                               # Threshold for object coverage, float value between 0 and 1
                      "nms_threshold": 0.3,                                               # Threshold for non-max suppression algorithm, float value between 0 and 1
                      "yolo_input_resolution": input_resolution_yolov3_HW}
postprocessor = PostprocessYOLO(**postprocessor_args)



import tensorrt as trt
from smartdrone import common
import os
TRT_LOGGER = trt.Logger()

def get_engine_2(engine_file_path=""):
    """Attempts to load a serialized engine if available, otherwise builds a new TensorRT engine and saves it."""
    if os.path.exists(engine_file_path):
        # If a serialized engine exists, use it instead of building an engine.
        print("Reading engine from file {}".format(engine_file_path))
        with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            return runtime.deserialize_cuda_engine(f.read())
    else:
        return None

# Load engine
engine_file_path = "/home/bootai/workspace/sm/weights/yolov3_fp16_608x1056.trt"
engine = get_engine_2(engine_file_path)
context = engine.create_execution_context()
inputs, outputs, bindings, stream = common.allocate_buffers(engine)


def landing_pad_detect(RGB_img):
    image_raw = Image.fromarray(RGB_img)
    # Load an image from the specified input path, and return it together with  a pre-processed version
    image_raw, image = preprocessor.process(image_raw)
    # Store the shape of the original input image in WH format, we will need it for later
    shape_orig_WH = image_raw.size
    
    inputs[0].host = image
    trt_outputs = common.do_inference_v2(context, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)
    
    # Before doing post-processing, we need to reshape the outputs as the common.do_inference will give us flat arrays.
    trt_outputs = [output.reshape(shape) for output, shape in zip(trt_outputs, output_shapes)]
    # Run the post-processing algorithms on the TensorRT outputs and get the bounding box details of detected objects
    boxes, classes, scores = postprocessor.process(trt_outputs, (shape_orig_WH))
    return boxes, scores

def filter_score_max(bboxes, scores):
    if len(scores) < 1:
        return None, None
    if len(scores) == 1:
        return bboxes[0], scores[0]
    else:
        max_score_idx = scores.argmax()
        return bboxes[max_score_idx], scores[max_score_idx]

def H_2_bbox_size(H, ratio=1.3):
    # (1105+1148+1162)/3 ~ 1138
    size = int(1138 / H)
    size_min = int(size/ratio)
    size_max = int(size*ratio)
    return size, size_min, size_max

def validate_bbox_size(H, bboxes, scores, ratio=1.3):
    bs = []
    ss = []
    _, size_min, size_max = H_2_bbox_size(H, ratio)
    for i, bbox in enumerate(bboxes):
        bbox_size_min = min(bbox[2],bbox[3])
        bbox_size_max = max(bbox[2],bbox[3])
        is_valid = (bbox_size_min > size_min) and (bbox_size_max < size_max)
        if is_valid:
            bs.append(bbox)
            ss.append(scores[i])
    return bs, ss


def compute_target_NE(H, ux, vy, alpha_heading, delta_pan, delta_tilt, delta_roll):
    """
    delta_pan, delta_tilt, delta_roll, alpha_headingv in degree
    ux, vy in pixels
    H in meters
    """
    [fx, fy, cx, cy] = [995.8, 1010.6, 951.0, 543.0]
    alpha_x = math.atan((ux - cx)/fx) * 180 / math.pi
    alpha_y = math.atan((vy - cy)/fy) * 180 / math.pi
    
    delta_x = H * math.tan((alpha_x - delta_roll) * math.pi / 180)
    delta_y = -1 * H * math.tan((alpha_y - delta_tilt) * math.pi / 180)
    
    phi = (360 - alpha_heading - delta_pan) * math.pi / 180
    N = delta_x * math.sin(phi) + delta_y * math.cos(phi)
    E = delta_x * math.cos(phi) - delta_y * math.sin(phi)
    
    return N,E

def compute_target_NE_gimbal_1642(H, ux, vy, alpha_heading):
    delta_pan = 5 #-6 #5 # Magic number (calibration)
    delta_tilt=0
    delta_roll=0
    return compute_target_NE(H, ux, vy, alpha_heading, delta_pan, delta_tilt, delta_roll)

def compute_target_NE_gimbal_1340(H, ux, vy, alpha_heading):
    delta_pan  = 5 - 90 #-6 - 90 #5 - 90 # 90 is rotation of camera
    delta_tilt = 0
    delta_roll = -5 # gimbal error. Negative # Magic number (calibration)
    return compute_target_NE(H, ux, vy, alpha_heading, delta_pan, delta_tilt, delta_roll)

def compute_target_from_frame(RGB_img, H, heading, is_gimbal_rotated, ratio=2.0):
    bboxes, scores = landing_pad_detect(RGB_img)
    print(bboxes, scores, ratio)
    bbox, score = filter_score_max(*validate_bbox_size(H, bboxes, scores, ratio))
    # print(bbox, score)
    if bbox is None:
        return None, None, None, None, None, None
    else:
        l,t,w,h = bbox.astype(np.int32)
        u_x = l + w/2
        v_y = t + h/2
        # print(u_x, v_y)
        if not is_gimbal_rotated:
            to_North, to_East = compute_target_NE_gimbal_1642(H, u_x, v_y, heading)
            # print("AAAAAAAAAA")
            # print(to_North, to_East)
        else:
            to_North, to_East = compute_target_NE_gimbal_1340(H, u_x, v_y, heading)
            # print("BBBBBBBBBB")
            # print(to_North, to_East)
        return to_North, to_East, l,t,w,h