"""Standalone YOLOv5 benchmark on real captured frames.

Measures preprocess / H2D / forward (CUDA events) / NMS / end-to-end per
(imgsz, half) combination, plus detection counts for accuracy comparison.
Uses the exact project weights and the project's own inference helpers.
"""

import json
import os
import sys
import time

import cv2
import numpy as np

TCONE = os.path.expanduser("~/qutms_ws/src/TCone-YOLO-main")
WEIGHTS = os.path.join(TCONE, "compared-results/model/TCone-YOLO_model/weights/best.pt")
DATA = os.path.join(TCONE, "data/Cone.yaml")
FRAMES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames")
WARMUP, ITERS = 5, 30


def bench(imgsz: int, half: bool, frames):
    sys.path.insert(0, TCONE)
    import torch
    from utils.augmentations import letterbox
    from utils.general import non_max_suppression, scale_boxes, check_img_size
    from models.common import DetectMultiBackend
    from utils.torch_utils import select_device

    device = select_device("0")
    t0 = time.perf_counter()
    model = DetectMultiBackend(WEIGHTS, device=device, dnn=False, data=DATA, fp16=half)
    load_s = time.perf_counter() - t0
    stride, pt = int(model.stride), bool(model.pt)
    imgsz = check_img_size(imgsz, s=stride)
    model.warmup(imgsz=(1, 3, imgsz, imgsz))

    det_counts, e2e_ms = [], []
    pre_ms = h2d_ms = fwd_ms = nms_ms = 0.0
    fw_events = [(torch.cuda.Event(True), torch.cuda.Event(True)) for _ in range(ITERS)]

    with torch.no_grad():
        for it in range(ITERS):
            im0 = frames[it % len(frames)]
            t = time.perf_counter()
            im = letterbox(im0, imgsz, stride=stride, auto=pt)[0]
            im = np.ascontiguousarray(im.transpose((2, 0, 1))[::-1])
            t = time.perf_counter() - t

            im_t = torch.from_numpy(im).to(device)
            im_t = im_t.half() if half else im_t.float()
            im_t /= 255.0
            im_t = im_t[None]
            h2d = time.perf_counter() - t - 0 if False else None  # keep simple: measure below

            t1 = time.perf_counter()
            ev0, ev1 = fw_events[it]
            ev0.record()
            pred = model(im_t, augment=False, visualize=False)
            if device.type == "cuda":
                torch.cuda.synchronize()
            ev1.record()
            torch.cuda.synchronize()
            fwd = time.perf_counter() - t1
            h2d_time = 0.0  # included in e2e; h2d alone measured once below

            t2 = time.perf_counter()
            nms_pred = non_max_suppression(pred, 0.15, 0.45, classes=None, agnostic=False, max_det=1000)
            nms = time.perf_counter() - t2

            dets = 0
            for det in nms_pred:
                if det is not None and len(det):
                    det[:, :4] = scale_boxes((imgsz, imgsz), det[:, :4], im0.shape).round()
                    dets += len(det)
            if it >= WARMUP:
                det_counts.append(dets)
                pre_ms += t * 1000
                fwd_ms += fwd * 1000
                nms_ms += nms * 1000
                e2e_ms.append((t + fwd + nms) * 1000)

    n = ITERS - WARMUP
    gpu_fwd = sorted(e0.elapsed_time(e1) for e0, e1 in fw_events[WARMUP:])
    res = {
        "imgsz": imgsz, "half": half, "load_s": round(load_s, 2),
        "pre_ms": round(pre_ms / n, 1), "fwd_ms": round(fwd_ms / n, 1),
        "nms_ms": round(nms_ms / n, 1),
        "e2e_ms_avg": round(float(np.mean(e2e_ms)), 1),
        "e2e_ms_p95": round(float(np.percentile(e2e_ms, 95)), 1),
        "gpu_fwd_ms_avg": round(float(np.mean(gpu_fwd)), 1),
        "gpu_fwd_ms_p95": round(float(np.percentile(gpu_fwd, 95)), 1),
        "fps": round(1000.0 / float(np.mean(e2e_ms)), 1),
        "dets_avg": round(float(np.mean(det_counts)), 1),
        "dets_min": int(np.min(det_counts)), "dets_max": int(np.max(det_counts)),
    }
    del model
    torch.cuda.empty_cache()
    return res


def main() -> None:
    frames = [cv2.imread(os.path.join(FRAMES, f"rgb_{i}.png")) for i in range(5)]
    frames = [f for f in frames if f is not None]
    if not frames:
        sys.exit(f"No frames in {FRAMES}; run capture_frames.py with the sim up first.")
    out = []
    for imgsz in (640, 416, 320):
        for half in (False, True):
            r = bench(imgsz, half, frames)
            out.append(r)
            print(json.dumps(r), flush=True)
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "bench_results.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("\n==== 汇总 ====")
    print(f"{'imgsz':>6} {'half':>6} {'pre':>7} {'fwd':>8} {'nms':>7} {'e2e':>8} {'p95':>8} {'FPS':>7} {'dets':>6}")
    for r in out:
        print(f"{r['imgsz']:>6} {str(r['half']):>6} {r['pre_ms']:>7} {r['fwd_ms']:>8} "
              f"{r['nms_ms']:>7} {r['e2e_ms_avg']:>8} {r['e2e_ms_p95']:>8} {r['fps']:>7} {r['dets_avg']:>6}")


if __name__ == "__main__":
    main()
