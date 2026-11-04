"""Shared high-resolution monotonic time for trajectories and feedback ages.

On this Windows Python 3.12 host, time.monotonic uses GetTickCount64 with
15.625 ms resolution. perf_counter uses QPC and must be used consistently by
the producer, transport timestamps and controller; their epochs are not mixed.
"""
from time import perf_counter as now
