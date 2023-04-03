from enum import Enum
import json
from typing import Any, List


class CMDCategory(Enum):
    """Function desccription FIXME"""

    WORKFLOW = 1
    ROBOT = 2
    STATION = 3


class CMDMessage:
    def __init__(self, category: CMDCategory, cmd: str, params: List[Any] = []) -> None:
        self.category = category
        self.cmd = cmd
        self.params = params

    def to_json(self) -> str:
        """Function desccription FIXME"""
        obj_dict = self.__dict__
        obj_dict["category"] = obj_dict["category"].value
        return json.dumps(obj_dict)

    @classmethod
    def from_json(cls, json_msg: str):
        """Function desccription FIXME"""
        msg_dict = json.loads(json_msg)
        return cls(
            category=CMDCategory(msg_dict["category"]),
            cmd=msg_dict["cmd"],
            params=msg_dict["params"],
        )
