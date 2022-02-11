from __future__ import annotations
from abc import ABC, abstractmethod
from nav_msgs.msg import OccupancyGrid


class ROSMerger(ABC):

    def __init__(self: ROSMerger) -> None:
        pass

    @abstractmethod
    def request_merge(self: ROSMerger, new_map: OccupancyGrid) -> bool:
        pass

    @abstractmethod
    def get_latest(self: ROSMerger) -> OccupancyGrid:
        pass
