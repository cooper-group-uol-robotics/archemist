# External
from mongoengine import Document, EmbeddedDocument
from typing import Union, List, Iterator, Any, TypeVar

# Core
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.persistence.object_factory import RobotFactory, StationFactory


class ListFieldAdapter:
    def __init__(self, model: Document, field_name: str):
        self._model = model
        self._field_name = field_name

    def append(self, field_object: Union[Document, EmbeddedDocument]):
        self._model.update(**{f"push__{self._field_name}": field_object})

    def popleft(self) -> Union[Document, EmbeddedDocument]:
        return self._pop(left=True)

    def pop(self) -> Union[Document, EmbeddedDocument]:
        return self._pop(left=False)

    def extend(self, objects_list: List[Union[Document, EmbeddedDocument]]):
        self._model.update(**{f"push_all__{self._field_name}": list(objects_list)})

    def remove(self, field_object: Union[Document, EmbeddedDocument]):
        self._model.update(**{f"pull__{self._field_name}": field_object})

    def _pop(self, left: bool) -> Union[Document, EmbeddedDocument]:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        if objects_list:
            list_pop_indx = 0 if left else -1
            field_pop_indx = -1 if left else 1
            new_list = objects_list.pop(list_pop_indx)
            self._model.update(**{f"pop__{self._field_name}": field_pop_indx})
            return new_list
        else:
            raise IndexError("pop from empty list")

    def __getitem__(self, index: int) -> Union[Document, EmbeddedDocument]:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return objects_list[index]

    def __setitem__(self, index: int, value: Any):
        self._model.update(**{f"{self._field_name}__{index}": value})

    def __iter__(self) -> Iterator:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return iter(list(objects_list))

    def __next__(self) -> Union[Document, EmbeddedDocument]:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return next(list(objects_list))

    def __bool__(self):
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return True if objects_list else False

    def __len__(self):
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return len(objects_list)


class OpListAdapter(ListFieldAdapter):
    def __init__(
        self,
        model: Document,
        field_name: str,
        factory_cls: Union[RobotFactory, StationFactory],
    ):
        self._factory_cls = factory_cls
        super().__init__(model, field_name)

    def append(self, adapter_object: Union[RobotOpDescriptor, StationOpDescriptor]):
        return super().append(adapter_object.model)

    def pop(self) -> Union[RobotOpDescriptor, StationOpDescriptor]:
        return self._factory_cls.create_op_from_model(super().pop())

    def popleft(self) -> Union[RobotOpDescriptor, StationOpDescriptor]:
        return self._factory_cls.create_op_from_model(super().popleft())

    def extend(self, objects_list: List[Union[RobotOpDescriptor, StationOpDescriptor]]):
        return super().extend(list(objects_list))

    def remove(self, adapter_object: Union[RobotOpDescriptor, StationOpDescriptor]):
        return super().remove(adapter_object.model)

    def __iter__(self) -> Iterator:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return iter(
            [
                self._factory_cls.create_op_from_model(adapter)
                for adapter in objects_list
            ]
        )

    def __next__(self) -> List[Union[RobotOpDescriptor, StationOpDescriptor]]:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return next(
            [
                self._factory_cls.create_op_from_model(adapter)
                for adapter in objects_list
            ]
        )

    def __getitem__(self, index: int) -> Union[RobotOpDescriptor, StationOpDescriptor]:
        return self._factory_cls.create_op_from_model(super().__getitem__(index))

    def __setitem__(
        self, index: int, op: Union[RobotOpDescriptor, StationOpDescriptor]
    ):
        super().__setitem__(index, op.model)


class StateObjListAdapter(ListFieldAdapter):
    def __init__(self, model: Document, field_name: str, state_obj_cls: TypeVar):
        self.state_obj_cls = state_obj_cls
        super().__init__(model, field_name)

    def append(self, state_object: Any):
        return super().append(state_object.model)

    def pop(self) -> Any:
        return self.state_obj_cls(super().pop())

    def popleft(self) -> Union[RobotOpDescriptor, StationOpDescriptor]:
        return self.state_obj_cls(super().popleft())

    def extend(self, objects_list: List[Any]):
        return super().extend([state.model for state in objects_list])

    def remove(self, state_object: Any):
        return super().remove(state_object.model)

    def __iter__(self) -> Iterator:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return iter([self.state_obj_cls(state) for state in objects_list])

    def __next__(self) -> List[Any]:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return next([self.state_obj_cls(state) for state in objects_list])

    def __getitem__(self, index: int) -> Any:
        return self.state_obj_cls(super().__getitem__(index))

    def __setitem__(self, index: int, state_object: Any):
        return super().__setitem__(index, state_object.model)
