"""YOLOv8 exported ONNX: RGB letterbox, three class probabilities, class-wise NMS."""
import ast
import cv2
import numpy as np
import onnxruntime as ort

NAMES = ['fire', 'chemical', 'human']

class Detector:
    def __init__(self, model, threshold=0.4):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        options.add_session_config_entry('session.inter_op.allow_spinning', '0')
        self.session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]
        assert self.input.shape == [1, 3, 640, 640], self.input.shape
        metadata = self.session.get_modelmeta().custom_metadata_map
        names = ast.literal_eval(metadata['names'])
        assert [names[i] for i in range(3)] == NAMES, names
        self.threshold = threshold

    def infer(self, bgr):
        h, w = bgr.shape[:2]
        scale = min(640 / w, 640 / h)
        nw, nh = round(w * scale), round(h * scale)
        left, top = round((640 - nw) / 2 - .1), round((640 - nh) / 2 - .1)
        canvas = np.full((640, 640, 3), 114, np.uint8)
        canvas[top:top+nh, left:left+nw] = cv2.resize(bgr, (nw, nh))
        tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255
        raw = self.session.run(None, {self.input.name: tensor})[0]
        assert raw.shape == (1, 7, 8400), raw.shape
        rows = raw[0].T
        classes = rows[:, 4:].argmax(axis=1)
        scores = rows[np.arange(len(rows)), classes+4]
        keep = scores >= self.threshold
        rows, classes, scores = rows[keep], classes[keep], scores[keep]
        boxes = rows[:, :4].copy()
        boxes[:, :2] -= boxes[:, 2:] / 2
        indices = []
        for cls in range(3):
            group = np.flatnonzero(classes == cls)
            if not len(group):
                continue
            selected = cv2.dnn.NMSBoxes(boxes[group].tolist(), scores[group].tolist(), self.threshold, .45)
            indices.extend(group[np.asarray(selected, dtype=int).reshape(-1)].tolist())
        objects = []
        for i in sorted(indices, key=lambda i: float(scores[i]), reverse=True)[:100]:
            x, y, bw, bh = boxes[i]
            xyxy = [max(0, min(w, (x-left)/scale)), max(0, min(h, (y-top)/scale)),
                    max(0, min(w, (x+bw-left)/scale)), max(0, min(h, (y+bh-top)/scale))]
            if xyxy[2] <= xyxy[0] or xyxy[3] <= xyxy[1]:
                continue
            objects.append(dict(class_id=int(classes[i]), label=NAMES[classes[i]], score=float(scores[i]), bbox_xyxy=list(map(float, xyxy))))
        return objects
