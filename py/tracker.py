import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class AccelMetrics:
    accel_achieved: np.ndarray
    accel_commanded: np.ndarray
    error_instant: np.ndarray

class AccelTracker:
    def __init__(self):
        self.prev_velocity: Optional[np.ndarray] = None
        self.prev_timestamp: Optional[float] = None

    def update(self, 
               velocity: np.ndarray,
               accel_commanded: np.ndarray,
               timestamp: float) -> Optional[AccelMetrics]:
        accel_achieved = self._calculate_achieved_acceleration(velocity, timestamp)
        self.prev_velocity = velocity
        self.prev_timestamp = timestamp

        if accel_achieved is None:
            return None

        accel_error = accel_commanded - accel_achieved

        return AccelMetrics(
            accel_achieved=accel_achieved,
            accel_commanded=accel_commanded,
            error_instant=accel_error
        )

    def _calculate_achieved_acceleration(self,
                                         velocity: np.ndarray,
                                         timestamp: float) -> Optional[np.ndarray]:
        if self.prev_timestamp is None or self.prev_velocity is None:
            return None

        dv = velocity - self.prev_velocity
        dt = timestamp - self.prev_timestamp
        return dv / dt


