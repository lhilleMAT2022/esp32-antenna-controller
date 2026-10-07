"""Parse immediate bearings or UTC polynomial target schedules."""
from dataclasses import dataclass
import math

MAX_COEFFICIENTS = 6
MAX_STEPS = 3600
MAX_DURATION = 86400
SLEW_ERRORS = {0: "ok", 1: "invalid plan", 2: "node UTC is unsynchronized",
               3: "start time has passed", 4: "queue full", 5: "overlapping plan",
               6: "request ID conflicts", 7: "legacy move queue is occupied",
               255: "gateway radio send failed"}


@dataclass(frozen=True)
class SlewPlan:
    start_utc_s: int
    coefficients: tuple[float, ...]
    duration_s: int = 30
    steps: int = 3

    def bearing(self, step: int) -> float:
        t = self.duration_s * step / self.steps
        result = 0.0
        for coefficient in reversed(self.coefficients):
            result = result*t + coefficient
        if not math.isfinite(result):
            raise ValueError("Polynomial overflows within the requested duration")
        return result % 360

    def message(self, node: int, request: int) -> dict:
        return dict(t="sc", n=node, q=request, at=self.start_utc_s,
                    coef=list(self.coefficients), dur=self.duration_s, steps=self.steps)


def parse_bearing(fields: list[str]) -> float | SlewPlan:
    if not fields:
        raise ValueError("Provide a bearing, or <UTC epoch seconds> C [X1 X2 ...] [dur seconds] [step count]")
    if len(fields) == 1:
        bearing = float(fields[0])
        if not math.isfinite(bearing) or not 0 <= bearing < 360:
            raise ValueError("Immediate bearing must be in [0, 360)")
        return bearing
    try:
        start = int(fields[0])
    except ValueError:
        raise ValueError("Scheduled start must be an integer UTC epoch second") from None
    if not 1 <= start <= 4102444800:
        raise ValueError("Scheduled UTC start is outside the supported epoch range")
    coefficients = []
    i = 1
    while i < len(fields) and fields[i] not in ("dur", "step"):
        coefficients.append(float(fields[i]))
        i += 1
    options = {}
    while i < len(fields):
        key = fields[i]
        if key not in ("dur", "step") or key in options or i+1 >= len(fields):
            raise ValueError("Use each optional 'dur <seconds>' and 'step <count>' once")
        try:
            options[key] = int(fields[i+1])
        except ValueError:
            raise ValueError(f"{key} requires an integer") from None
        i += 2
    if not 1 <= len(coefficients) <= MAX_COEFFICIENTS or not all(map(math.isfinite, coefficients)):
        raise ValueError("Provide 1..6 finite coefficients (constant through fifth order)")
    duration, steps = options.get("dur", 30), options.get("step", 3)
    if not 1 <= duration <= MAX_DURATION or not 1 <= steps <= MAX_STEPS or duration*1000 < steps*100:
        raise ValueError("Duration must be 1..86400 s; steps 1..3600; spacing at least 100 ms")
    plan = SlewPlan(start, tuple(coefficients), duration, steps)
    for k in range(steps+1):
        plan.bearing(k)
    return plan
