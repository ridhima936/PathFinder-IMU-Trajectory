"""IMU failure and degradation state detection."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, Dict, Optional

import numpy as np


class FailureState(str, Enum):
    OK = "ok"
    TEMPORARY_DROPOUT = "temporary_dropout"
    SEVERE_DEGRADATION = "severe_degradation"
    COMPLETE_FAILURE = "complete_failure"


@dataclass
class FailureConfig:
    zero_eps: float = 0.0  # exact-zero check threshold for all channels
    invalid_as_dead: bool = True
    stuck_window: int = 10
    stuck_tol: float = 1e-12
    dropout_gap_factor: float = 3.0  # gap > factor * median_dt => dropout/failure evidence
    complete_failure_consecutive: int = 5
    severe_invalid_window: int = 20
    severe_invalid_ratio: float = 0.5
    min_dt_s: float = 1e-5


class FailureDetector:
    """Detect temporary dropouts, severe degradation, and complete IMU failure.

    Notes:
    - Small but valid sensor readings are NOT treated as failure (stationary IMU can be valid).
    - Complete failure is triggered by sustained dead/invalid/zero readings or extreme gaps.
    """

    def __init__(self, config: FailureConfig = None, median_dt_s: Optional[float] = None):
        self.config = config or FailureConfig()
        self.median_dt_s = median_dt_s
        self.last_t_ns: Optional[int] = None
        self.consecutive_dead = 0
        self.consecutive_stuck = 0
        self.recent_dead: Deque[int] = deque(maxlen=self.config.severe_invalid_window)
        self.last_values: Optional[np.ndarray] = None
        self.failure_started = False
        self.failure_start_index: Optional[int] = None
        self.failure_start_t_ns: Optional[int] = None
        self.state = FailureState.OK
        self.stats: Dict[str, int] = {
            "dead_samples": 0,
            "stuck_samples": 0,
            "gap_events": 0,
            "invalid_samples": 0,
        }

    def update_median_dt(self, median_dt_s: float):
        if median_dt_s and median_dt_s > 0:
            self.median_dt_s = float(median_dt_s)

    def _is_invalid(self, gyro: np.ndarray, accel: np.ndarray) -> bool:
        vals = np.concatenate([gyro.reshape(-1), accel.reshape(-1)])
        return not np.all(np.isfinite(vals))

    def _is_all_zero(self, gyro: np.ndarray, accel: np.ndarray) -> bool:
        eps = self.config.zero_eps
        vals = np.concatenate([gyro.reshape(-1), accel.reshape(-1)])
        return bool(np.all(np.abs(vals) <= eps))

    def _is_stuck(self, gyro: np.ndarray, accel: np.ndarray) -> bool:
        cur = np.concatenate([gyro.reshape(-1), accel.reshape(-1)])
        if self.last_values is None:
            self.last_values = cur
            self.consecutive_stuck = 0
            return False
        if np.all(np.abs(cur - self.last_values) <= self.config.stuck_tol):
            self.consecutive_stuck += 1
        else:
            self.consecutive_stuck = 0
        self.last_values = cur
        return self.consecutive_stuck >= max(1, self.config.stuck_window - 1)

    def observe(self, index: int, t_ns: int, gyro: np.ndarray, accel: np.ndarray,
                gap_s: Optional[float] = None) -> Dict[str, object]:
        g = np.asarray(gyro, dtype=np.float64).reshape(3)
        a = np.asarray(accel, dtype=np.float64).reshape(3)
        invalid = self._is_invalid(g, a)
        all_zero = self._is_all_zero(g, a)
        stuck = self._is_stuck(g, a) if not invalid else False
        dead_sample = (invalid and self.config.invalid_as_dead) or all_zero

        # gap detection
        if gap_s is None and self.last_t_ns is not None:
            gap_s = (int(t_ns) - int(self.last_t_ns)) * 1e-9
        large_gap = False
        if gap_s is not None and self.median_dt_s and self.median_dt_s > 0:
            large_gap = gap_s > (self.config.dropout_gap_factor * self.median_dt_s)
        self.last_t_ns = int(t_ns)

        if invalid:
            self.stats["invalid_samples"] += 1
        if dead_sample:
            self.stats["dead_samples"] += 1
            self.consecutive_dead += 1
        else:
            self.consecutive_dead = 0
        if stuck:
            self.stats["stuck_samples"] += 1
        if large_gap:
            self.stats["gap_events"] += 1

        self.recent_dead.append(1 if dead_sample else 0)

        # State determination
        complete = self.failure_started
        if self.consecutive_dead >= self.config.complete_failure_consecutive:
            complete = True
        # A very large gap can indicate sensor inactivity/dropout; treat as complete only if sustained
        # by dead evidence elsewhere - here we mark severe, unless already in complete failure.
        recent_ratio = (sum(self.recent_dead) / len(self.recent_dead)) if self.recent_dead else 0.0
        severe = (recent_ratio >= self.config.severe_invalid_ratio) or stuck or (large_gap and dead_sample)

        if complete:
            self.state = FailureState.COMPLETE_FAILURE
            if not self.failure_started:
                self.failure_started = True
                self.failure_start_index = index
                self.failure_start_t_ns = int(t_ns)
        elif severe:
            self.state = FailureState.SEVERE_DEGRADATION
        elif large_gap or (dead_sample and self.consecutive_dead < self.config.complete_failure_consecutive):
            self.state = FailureState.TEMPORARY_DROPOUT
        else:
            self.state = FailureState.OK

        return {
            "index": index,
            "t_ns": int(t_ns),
            "state": self.state.value,
            "invalid": bool(invalid),
            "all_zero": bool(all_zero),
            "dead_sample": bool(dead_sample),
            "stuck": bool(stuck),
            "large_gap": bool(large_gap),
            "gap_s": float(gap_s) if gap_s is not None else None,
            "consecutive_dead": int(self.consecutive_dead),
            "failure_started": bool(self.failure_started),
        }

    def summary(self) -> Dict[str, object]:
        return {
            "state": self.state.value,
            "failure_started": self.failure_started,
            "failure_start_index": self.failure_start_index,
            "failure_start_t_ns": self.failure_start_t_ns,
            **self.stats,
        }
