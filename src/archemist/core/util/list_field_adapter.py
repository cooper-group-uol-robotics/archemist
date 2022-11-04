from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.persistence.object_factory import RobotFactory, StationFactory
from mongoengine import Document, EmbeddedDocument
from typing import Union,List, Iterator, Any, TypeVar

class ListFieldAdapter:
    def __init__(self, model: Document, field_name: str):
        self._model = model
        self._field_name = field_name

    def append(self, object: Union[Document, EmbeddedDocument]):
        self._model.update(**{f'push__{self._field_name}':object})

    def popleft(self) -> Union[Document, EmbeddedDocument]:
        return self._pop(left=True)

    def pop(self) -> Union[Document, EmbeddedDocument]:
        return self._pop(left=False)

    def extend(self, objects_list: List[Union[Document, EmbeddedDocument]]):
        self._model.update(**{f'push_all__{self._field_name}':[obj for obj in objects_list]})

    def _pop(self, left: bool) -> Union[Document, EmbeddedDocument]:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model,self._field_name)
        if objects_list:
            list_pop_indx = 0 if left else -1
            field_pop_indx = -1 if left else 1
            object = objects_list.pop(list_pop_indx)
            self._model.update(**{f'pop__{self._field_name}':field_pop_indx})
            return object
        else:
            raise IndexError('pop from empty list')

    def __getitem__(self, index: int) -> Union[Document, EmbeddedDocument]:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model,self._field_name)
        return objects_list[index]

    def __setitem__(self, index: int, value: Any):
        self._model.update(**{f'{self._field_name}__{index}':value})

    def __iter__(self) -> Iterator:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return iter([object for object in objects_list])

    def __next__(self) -> Union[Document, EmbeddedDocument]: 
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return next([object for object in objects_list])

    def __bool__(self):
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return True if objects_list else False

    def __len__(self):
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return len(objects_list)

class OpListAdapter(ListFieldAdapter):
    def __init__(self, model: Document, field_name: str, factory_cls: Union[RobotFactory,StationFactory]):
        self._factory_cls = factory_cls
        super().__init__(model, field_name)

    def append(self, object: Union[RobotOpDescriptor,StationOpDescriptor]):
        return super().append(object.model)

    def pop(self) -> Union[RobotOpDescriptor,StationOpDescriptor]:
        return self._factory_cls.create_op_from_model(super().pop())

    def popleft(self) -> Union[RobotOpDescriptor,StationOpDescriptor]:
        return self._factory_cls.create_op_from_model(super().popleft())

    def extend(self, objects_list: List[Union[RobotOpDescriptor,StationOpDescriptor]]):
        return super().extend([object.model for object in objects_list])

    def __iter__(self) -> Iterator:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return iter([self._factory_cls.create_op_from_model(object) for object in objects_list])

    def __next__(self) -> List[Union[RobotOpDescriptor,StationOpDescriptor]]: 
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return next([self._factory_cls.create_op_from_model(object) for object in objects_list])

    def __getitem__(self, index: int) -> Union[RobotOpDescriptor,StationOpDescriptor]:
        return self._factory_cls.create_op_from_model(super().__getitem__(index))

    def __setitem__(self, index: int, op: Union[RobotOpDescriptor,StationOpDescriptor]):
        super().__setitem__(index, op.model)

class StateObjListAdapter(ListFieldAdapter):
    def __init__(self, model: Document, field_name: str, state_obj_cls: TypeVar):
        self.state_obj_cls = state_obj_cls
        super().__init__(model, field_name)

    def append(self, object: Any):
        return super().append(object.model)

    def pop(self) -> Any:
        return self.state_obj_cls(super().pop())

    def popleft(self) -> Union[RobotOpDescriptor,StationOpDescriptor]:
        return self.state_obj_cls(super().popleft())

    def extend(self, objects_list: List[Any]):
        return super().extend([object.model for object in objects_list])

    def __iter__(self) -> Iterator:
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return iter([self.state_obj_cls(object) for object in objects_list])

    def __next__(self) -> List[Any]: 
        self._model.reload(self._field_name)
        objects_list = getattr(self._model, self._field_name)
        return next([self.state_obj_cls(object) for object in objects_list])

    def __getitem__(self, index: int) -> Any:
        return self.state_obj_cls(super().__getitem__(index))

    def __setitem__(self, index: int, object: Any):
        return super().__setitem__(index, object.model)

    

