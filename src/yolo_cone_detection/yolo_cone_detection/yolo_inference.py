"""TCone-YOLO inference wrapper.

A thin wrapper around the YOLOv5 / TCone-YOLO inference pipeline that keeps
all PyTorch / model-loading logic isolated from the ROS2 node.

The caller is responsible for adding the TCone-YOLO repository root to
``sys.path`` before importing this module (so that ``models`` and ``utils``
are resolvable).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np


def ensure_tcone_path(tcone_root: str) -> None:
    """Add the TCone-YOLO repository root to ``sys.path`` if not already present."""
    root = str(Path(tcone_root).resolve())
    if root not in sys.path:
        sys.path.insert(0, root)


class YoloInference:
    """Lazy-loads a TCone-YOLO model and runs single-frame detection.

    The model is loaded on first call to :meth:`warmup` so that construction
    of this object never touches the GPU / heavy imports.
    """

    def __init__(
        self,
        weights: str,
        data_yaml: str,
        tcone_root: str,
        imgsz: int = 640,
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
        max_det: int = 1000,
        device: str = "",
        half: bool = False,
    ) -> None:
        self.weights = weights
        self.data_yaml = data_yaml
        self.tcone_root = tcone_root
        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = device
        self._detect_count = 0
        self.half = half

        self._model = None
        self._names: Optional[Sequence[str]] = None
        self._stride = 32
        self._pt = True

    # ------------------------------------------------------------------ #
    # Model loading
    # ------------------------------------------------------------------ #
    def load(self) -> None:
        """Load the model onto the selected device. Idempotent."""
        if self._model is not None:
            return

        ensure_tcone_path(self.tcone_root)

        import torch  # noqa: WPS433  (intentional lazy import)
        from models.common import DetectMultiBackend
        from utils.general import check_img_size
        from utils.torch_utils import select_device

        device = select_device(self.device)
        self._device = device

        model = DetectMultiBackend(
            self.weights,
            device=device,
            dnn=False,
            data=self.data_yaml,
            fp16=self.half,
        )
        self._model = model
        self._names = model.names
        self._stride = int(model.stride)
        self._pt = bool(model.pt)
        self._imgsz = check_img_size(self.imgsz, s=self._stride)

        # FP16 only meaningful on CUDA
        if self.half and device.type != "cuda":
            self.half = False

    def warmup(self) -> None:
        self.load()
        self._model.warmup(imgsz=(1, 3, self._imgsz, self._imgsz))

    # ------------------------------------------------------------------ #
    # Inference
    # ------------------------------------------------------------------ #
    def detect(self, bgr_image: np.ndarray) -> List[Tuple[float, float, float, float, float, int]]:
        """Run detection on a single BGR image.

        Args:
            bgr_image: uint8 HxWx3 image in BGR order (OpenCV convention).

        Returns:
            List of detections ``[x1, y1, x2, y2, conf, cls]`` in the original
            image's pixel coordinates. ``cls`` is the integer class index.
        """
        self.load()
        if self._model is None:
            return []

        import torch  # noqa: WPS433
        from utils.augmentations import letterbox
        from utils.general import non_max_suppression, scale_boxes

        im0 = np.ascontiguousarray(bgr_image)

        # Letterbox resize to model input size
        im = letterbox(im0, self._imgsz, stride=self._stride, auto=self._pt)[0]
        im = im.transpose((2, 0, 1))[::-1]  # HWC -> CHW, BGR -> RGB
        im = np.ascontiguousarray(im)

        _t = time.perf_counter()
        im_tensor = torch.from_numpy(im).to(self._device)
        im_tensor = im_tensor.half() if self.half else im_tensor.float()
        im_tensor /= 255.0
        if im_tensor.ndim == 3:
            im_tensor = im_tensor[None]
        _t_h2d = time.perf_counter()

        # Inference must not build an autograd graph: without this the node
        # pays a large per-frame overhead (the benchmark in diagnostics/ uses
        # no_grad and runs an order of magnitude faster). no_grad is used
        # instead of inference_mode because the boxes are rounded in-place
        # below, which inference tensors forbid outside the context.
        with torch.no_grad():
            pred = self._model(im_tensor, augment=False, visualize=False)
            if self._device.type == "cuda":
                torch.cuda.synchronize()
            _t_infer = time.perf_counter()

            pred = non_max_suppression(
                pred,
                self.conf_thres,
                self.iou_thres,
                classes=None,
                agnostic=False,
                max_det=self.max_det,
            )
            _t_nms = time.perf_counter()

        self._detect_count += 1
        if self._detect_count % 10 == 0:
            print(
                f"[yolo-timing] h2d={(_t_h2d - _t) * 1000:.1f}ms "
                f"infer={(_t_infer - _t_h2d) * 1000:.1f}ms nms={(_t_nms - _t_infer) * 1000:.1f}ms "
                f"device={self._device.type}",
                flush=True,
            )

        detections: List[Tuple[float, float, float, float, float, int]] = []
        for det in pred:
            if det is None or len(det) == 0:
                continue
            det[:, :4] = scale_boxes(im_tensor.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in det.cpu().numpy():
                detections.append(
                    (float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3]), float(conf), int(cls))
                )
        return detections

    @property
    def names(self) -> Optional[Sequence[str]]:
        return self._names
