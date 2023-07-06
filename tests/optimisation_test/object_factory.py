from __future__ import annotations
from typing import Any, Dict, List, TYPE_CHECKING
if TYPE_CHECKING:
    from archemist.core.state.material import Material,Liquid, Solid
    from archemist.core.state.batch import Batch
    from archemist.core.state.robot import Robot, RobotOpDescriptor, RobotModel
    # from optimisation_model import OptimisationModel
    from optimisation_state import OptimizationState, OptimisationModel
    from archemist.core.state.station import Station, StationOpDescriptor, StationModel

import importlib
import pkgutil
from bson.objectid import ObjectId


class OptimizationFactory:
    
    @staticmethod
    def create_from_dict(config_dict: Dict) -> OptimizationState:
        # from archemist.core.state.optimisation_state import OptimizationState
        from optimisation_state import OptimizationState
        return OptimizationState.from_dict(config_dict)

    @staticmethod
    def create_from_model(optimization_model: OptimisationModel) -> OptimizationState:
        module = importlib.import_module(optimization_model._module)
        cls = getattr(module, optimization_model._type)
        return cls(optimization_model)

