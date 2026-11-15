"""Detection scoring against simulator ground truth. Pure python, no ROS."""
from collections import defaultdict

DIST_BINS = [0.0, 0.5, 1.0, 1.5, 2.5, float("inf")]
BACKGROUND = "<background>"
MISSED = "<missed>"


def iou(a, b):
    """IoU of two (x1, y1, x2, y2) boxes."""
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def det_to_xyxy(d):
    """Detection dict (center xywh like Roboflow, or x1..y2) -> (x1, y1, x2, y2)."""
    if "x1" in d:
        return (d["x1"], d["y1"], d["x2"], d["y2"])
    return (d["x"] - d["width"] / 2.0, d["y"] - d["height"] / 2.0,
            d["x"] + d["width"] / 2.0, d["y"] + d["height"] / 2.0)


def _dist_bin(depth):
    for i in range(len(DIST_BINS) - 1):
        if DIST_BINS[i] <= depth < DIST_BINS[i + 1]:
            return i
    return len(DIST_BINS) - 2


def match_frame(gt, dets, iou_thr=0.5, ignore_thr=0.3, class_agnostic=False):
    """Match detections to ground truth for one frame.

    gt:   [{id, class_name, box, depth, status}]   (status 'expected' | 'ignore')
    dets: [{class_name, confidence, ...box...}]
    Returns dict with per-detection labels ('tp' | 'fp' | 'ignored'), per-gt labels
    ('tp' | 'fn' | 'ignore') and confusions [(gt_class, det_class)].
    """
    expected = [g for g in gt if g["status"] == "expected" and g["box"] is not None]
    ignored = [g for g in gt if g["status"] != "expected" and g["box"] is not None]
    order = sorted(range(len(dets)), key=lambda i: -dets[i].get("confidence", 0.0))
    boxes = [det_to_xyxy(d) for d in dets]

    det_label = {}
    gt_matched = {}
    # Pass 1: same-class (or any-class) matches, highest confidence first.
    for i in order:
        best, best_iou = None, iou_thr
        for g in expected:
            if g["id"] in gt_matched:
                continue
            if not class_agnostic and g["class_name"] != dets[i]["class_name"]:
                continue
            v = iou(boxes[i], g["box"])
            if v >= best_iou:
                best, best_iou = g, v
        if best is not None:
            gt_matched[best["id"]] = i
            det_label[i] = "tp"

    confusions = []
    # Pass 2: leftover detections: wrong class on an object, ignorable region, or plain FP.
    for i in order:
        if i in det_label:
            continue
        best, best_iou = None, iou_thr
        for g in expected:
            if g["id"] in gt_matched:
                continue
            v = iou(boxes[i], g["box"])
            if v >= best_iou:
                best, best_iou = g, v
        if best is not None:
            confusions.append((best["class_name"], dets[i]["class_name"]))
            det_label[i] = "fp"
            continue
        if any(iou(boxes[i], g["box"]) >= ignore_thr for g in ignored):
            det_label[i] = "ignored"
        else:
            det_label[i] = "fp"
            confusions.append((BACKGROUND, dets[i]["class_name"]))

    gt_label = {}
    for g in gt:
        if g["status"] != "expected" or g["box"] is None:
            gt_label[g["id"]] = "ignore"
        elif g["id"] in gt_matched:
            gt_label[g["id"]] = "tp"
        else:
            gt_label[g["id"]] = "fn"
    return {"det_label": det_label, "gt_label": gt_label, "confusions": confusions}


class Accumulator:
    """Running totals over many frames."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.frames = 0
        self.tp = self.fp = self.fn = 0
        self.per_class = defaultdict(lambda: {"tp": 0, "fp": 0, "fn": 0})
        self.confusion = defaultdict(lambda: defaultdict(int))
        self.dist = [{"gt": 0, "tp": 0} for _ in range(len(DIST_BINS) - 1)]
        self.scored = defaultdict(list)   # class -> [(confidence, is_tp)]
        self.n_gt = defaultdict(int)      # class -> expected GT count
        self.latency_ms = []

    def add(self, gt, dets, result, latency_ms=None):
        self.frames += 1
        if latency_ms is not None:
            self.latency_ms.append(latency_ms)
        for g in gt:
            lab = result["gt_label"][g["id"]]
            if lab == "ignore":
                continue
            c = g["class_name"]
            self.n_gt[c] += 1
            b = self.dist[_dist_bin(g["depth"])]
            b["gt"] += 1
            if lab == "tp":
                self.tp += 1
                self.per_class[c]["tp"] += 1
                b["tp"] += 1
            else:
                self.fn += 1
                self.per_class[c]["fn"] += 1
                self.confusion[c][MISSED] += 1
        for i, lab in result["det_label"].items():
            if lab == "ignored":
                continue
            c = dets[i]["class_name"]
            self.scored[c].append((dets[i].get("confidence", 0.0), lab == "tp"))
            if lab == "fp":
                self.fp += 1
                self.per_class[c]["fp"] += 1
        for gt_c, det_c in result["confusions"]:
            self.confusion[gt_c][det_c] += 1

    def average_precision(self, cls):
        """All-point interpolated AP for one class at the matching IoU threshold."""
        n = self.n_gt.get(cls, 0)
        if n == 0:
            return None
        pts = sorted(self.scored.get(cls, []), key=lambda t: -t[0])
        tp = fp = 0
        prec, rec = [], []
        for _, is_tp in pts:
            tp += is_tp
            fp += (not is_tp)
            prec.append(tp / float(tp + fp))
            rec.append(tp / float(n))
        mrec = [0.0] + rec + [1.0]
        mpre = [0.0] + prec + [0.0]
        for i in range(len(mpre) - 2, -1, -1):
            mpre[i] = max(mpre[i], mpre[i + 1])
        return sum((mrec[i + 1] - mrec[i]) * mpre[i + 1] for i in range(len(mrec) - 1))

    def summary(self):
        def pr(tp, fp, fn):
            p = tp / float(tp + fp) if tp + fp else None
            r = tp / float(tp + fn) if tp + fn else None
            f1 = 2 * p * r / (p + r) if p and r else (0.0 if p is not None and r is not None else None)
            return p, r, f1
        p, r, f1 = pr(self.tp, self.fp, self.fn)
        classes = sorted(set(self.per_class) | set(self.n_gt))
        per_class = {}
        aps = []
        for c in classes:
            d = self.per_class[c]
            cp, cr, cf = pr(d["tp"], d["fp"], d["fn"])
            ap = self.average_precision(c)
            if ap is not None:
                aps.append(ap)
            per_class[c] = {**d, "precision": cp, "recall": cr, "f1": cf, "ap50": ap,
                            "gt": self.n_gt.get(c, 0)}
        dist = []
        for i, b in enumerate(self.dist):
            lo, hi = DIST_BINS[i], DIST_BINS[i + 1]
            dist.append({"range_m": [lo, None if hi == float("inf") else hi], "gt": b["gt"],
                         "recall": (b["tp"] / float(b["gt"])) if b["gt"] else None})
        lat = sorted(self.latency_ms)
        return {
            "frames": self.frames, "tp": self.tp, "fp": self.fp, "fn": self.fn,
            "precision": p, "recall": r, "f1": f1,
            "map50": (sum(aps) / len(aps)) if aps else None,
            "per_class": per_class,
            "recall_by_distance": dist,
            "confusion": {g: dict(d) for g, d in self.confusion.items()},
            "latency_ms": {"mean": sum(lat) / len(lat), "p50": lat[len(lat) // 2],
                           "p95": lat[min(len(lat) - 1, int(0.95 * len(lat)))]} if lat else None,
        }


def format_summary(s):
    """Human-readable text version of Accumulator.summary()."""
    def f(v, d=3):
        return "  -  " if v is None else ("%.*f" % (d, v))
    lines = ["=" * 64, "CV MODEL REPORT   frames scored: %d" % s["frames"], "=" * 64,
             "Overall   precision %s   recall %s   F1 %s   mAP@0.5 %s" % (
                 f(s["precision"]), f(s["recall"]), f(s["f1"]), f(s["map50"])),
             "          TP %d   FP %d (false alarms)   FN %d (misses)" % (s["tp"], s["fp"], s["fn"])]
    if s.get("latency_ms"):
        lt = s["latency_ms"]
        lines.append("Inference %.0f ms mean / %.0f p50 / %.0f p95" % (lt["mean"], lt["p50"], lt["p95"]))
    lines += ["", "%-22s %5s %7s %7s %7s %7s" % ("Class", "GT", "Prec", "Recall", "F1", "AP50")]
    for c, d in s["per_class"].items():
        lines.append("%-22s %5d %7s %7s %7s %7s" % (c, d["gt"], f(d["precision"]), f(d["recall"]),
                                                    f(d["f1"]), f(d["ap50"])))
    lines += ["", "Recall by camera distance:"]
    for b in s["recall_by_distance"]:
        hi = "inf" if b["range_m"][1] is None else "%.1f" % b["range_m"][1]
        lines.append("  %.1f-%s m   n=%-4d recall %s" % (b["range_m"][0], hi, b["gt"], f(b["recall"])))
    if s.get("confusion"):
        lines += ["", "Confusions (ground truth -> what the model said):"]
        for g, row in sorted(s["confusion"].items()):
            for d, n in sorted(row.items(), key=lambda kv: -kv[1]):
                lines.append("  %-20s -> %-20s %d" % (g, d, n))
    if s.get("frames_not_scored"):
        lines += ["", "Frames not scored: " + ", ".join("%s=%d" % kv for kv in s["frames_not_scored"].items())]
    return "\n".join(lines) + "\n"
