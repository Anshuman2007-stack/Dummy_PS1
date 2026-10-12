"""Evaluation metrics used by the ROS evaluation nodes."""
import math
import statistics


def rmse(vals):
    vals = list(vals)
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float('nan')


def step_metrics(ts, ys, y0, ref, band=0.02, tail=0.5):
    """Step-response metrics of samples (ts, ys) for a step y0 -> ref at ts[0].

    rise      10-90 % rise time [s]
    settling  last time the response is outside +-band*|ref-y0| [s]
    overshoot peak beyond ref in % of the step
    sse       mean(y over the last ``tail`` s) - ref
    """
    span = ref - y0
    if not ts or span == 0:
        return {'rise': float('nan'), 'settling': float('nan'), 'overshoot': float('nan'),
                'sse': float('nan')}
    norm = [(y - y0) / span for y in ys]

    def first(f):
        for t, n in zip(ts, norm):
            if n >= f:
                return t
        return float('nan')
    rise = first(0.9) - first(0.1)
    overshoot = max(0.0, (max(norm) - 1.0) * 100.0)
    settle = 0.0
    for t, y in zip(ts, ys):
        if abs(y - ref) > band * abs(span):
            settle = t - ts[0]
    last = [y for t, y in zip(ts, ys) if t >= ts[-1] - tail]
    return {'rise': rise, 'settling': settle, 'overshoot': overshoot,
            'sse': statistics.mean(last) - ref}
