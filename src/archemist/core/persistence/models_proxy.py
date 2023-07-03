from mongoengine import Document, EmbeddedDocument
from mongoengine.base.datastructures import BaseList, BaseDict
from typing import Any, Callable
from enum import Enum, auto
from weakref import proxy

class ParentType(Enum):
    DOCUMENT = auto()
    LIST = auto()
    DICT = auto()
    EMBED = auto()
    EMBED_LIST_ELEMENT = auto()
    EMBED_DICT_ELEMENT = auto()

class ModelProxy:
    def __init__(self, model_instance: Document):
        self._model = model_instance

        # Get the fields dynamically from the model
        fields = self._model._fields
        for field_name in fields:
            # Create a property for each field
            setattr(self.__class__, field_name, self._create_property(field_name))

    def _create_property(self, field_name):
        # Create a getter function for the property
        def getter(self):
            # Reload the model to get the latest data
            self._model.reload(field_name)
            field_value = getattr(self._model, field_name)
            if isinstance(field_value, EmbeddedDocument):
                return EmbedModelProxy(field_value, self._model, field_name, ParentType.DOCUMENT)
            elif isinstance(field_value, BaseList):
                return ListFieldWrapper(field_value, self._model, field_name, ParentType.DOCUMENT)
            elif isinstance(field_value, BaseDict):
                return DictFieldWrapper(field_value, self._model, field_name,  ParentType.DOCUMENT)
            elif isinstance(field_value, Document):
                return ModelProxy(field_value)
            return field_value

        # Create a setter function for the property
        def setter(self, value):
            setattr(self._model, field_name, value)
            # Save the updated model
            self._model.update(**{f"{field_name}":value})

        # Create the property using the getter and setter functions
        return property(getter, setter)
    
    @property
    def model(self):
        self._model.reload()
        return self._model

class EmbedModelProxy:
    def __init__(self, embed_instance: EmbeddedDocument, parent_instance: Document, field_string: str, parent_type: ParentType):
        self._embedded = embed_instance
        self._parent = parent_instance
        self._field_string = field_string
        self._parent_type = parent_type
        if parent_type == ParentType.DOCUMENT:
            self._field_name = field_string
        else:
            self._field_name, self._field_index = field_string.split("__")

    def __getattr__(self, attr):
        self._reload()
        attr_value = getattr(self._embedded, attr)
        if self._parent_type == ParentType.LIST:
            parent_type = ParentType.EMBED_LIST_ELEMENT
        elif self._parent_type == ParentType.DICT:
            parent_type = ParentType.EMBED_DICT_ELEMENT
        else:
            parent_type = ParentType.EMBED
        if isinstance(attr_value, BaseList):
            return ListFieldWrapper(attr_value, self._parent, f"{self._field_string}__{attr}", parent_type)
        elif isinstance(attr_value, BaseDict):
            return DictFieldWrapper(attr_value, self._parent, f"{self._field_string}__{attr}", parent_type)
        elif isinstance(attr_value, Document):
            return ModelProxy(attr_value)
        else:
            return attr_value
        
    def __setattr__(self, attr, value):
        if attr in ["_embedded", "_parent", "_field_string", "_parent_type", "_field_name",
                    "_field_index"]:
            super().__setattr__(attr, value)
        else:
            setattr(self._embedded, attr, value)
            self._parent.update(**{f"set__{self._field_string}__{attr}":value})

    def _reload(self):
        self._parent.reload(self._field_name)
        field_value = getattr(self._parent, self._field_name)
        if self._parent_type == ParentType.DOCUMENT:
            setattr(self, "_embedded", field_value)
        elif self._parent_type == ParentType.LIST:
            setattr(self, "_embedded", field_value[int(self._field_index)])
        elif self._parent_type == ParentType.DICT:
            setattr(self, "_embedded", field_value[self._field_index])

    @property
    def model(self):
        self._reload()
        return self._embedded

class ListFieldWrapper:
    def __init__(self, list_instance: BaseList, parent_instance: Document, field_string: str, parent_type: ParentType):
        self._list_instance = list_instance
        self._parent = parent_instance
        self._field_string = field_string
        self._parent_type = parent_type
        if parent_type == ParentType.DOCUMENT:
            self._list_field_name = self._field_string
        elif parent_type == ParentType.EMBED:
            self._parent_field_name, self._list_field_name = field_string.split("__")
        elif parent_type == ParentType.EMBED_LIST_ELEMENT or parent_type == ParentType.EMBED_DICT_ELEMENT:
            self._parent_field_name, self._parent_field_index, \
                self._list_field_name = field_string.split("__")
            self._test_and_fix_ref()
    
    def __getitem__(self, index):
        self._reload()
        item = self._list_instance[index]
        if isinstance(item, EmbeddedDocument):
            if self._parent_type == ParentType.DOCUMENT:
                return EmbedModelProxy(item, self._parent, f"{self._list_field_name}__{index}",
                                       ParentType.LIST)
            else:
                raise NotImplementedError("current implementation doesn't support nested embedded documents")
        elif isinstance(item, Document):
            return ModelProxy(item)
        else:
            return item
    
    def __setitem__(self, index, value):
        self._list_instance[index] = value
        self._parent.update(**{f"set__{self._field_string}__{index}":value})

    def __len__(self):
        self._reload()
        return len(self._list_instance)
    
    def __iter__(self):
        self._reload()
        for index, item in enumerate(self._list_instance):
            if isinstance(item, EmbeddedDocument):
                if self._parent_type == ParentType.DOCUMENT:
                    yield EmbedModelProxy(item, self._parent, f"{self._list_field_name}__{index}",
                                       ParentType.LIST)
                else:
                    raise NotImplementedError("current implementation doesn't support nested embedded documents")
            elif isinstance(item, Document):
                yield ModelProxy(item)
            else:
                yield item

    def append(self, value): # this should work with wrappers
        self._parent.update(**{f"push__{self._field_string}":value})

    def pop(self, left=False):
        self._reload()
        if left:
            item = self._list_instance.pop(0) # this needs to be converted
            self._parent.update(**{f'pop__{self._field_string}':-1})
        else:
            item = self._list_instance.pop()
            self._parent.update(**{f'pop__{self._field_string}':1})
        return item

    def extend(self, obj_list):
        self._parent.update(**{f'push_all__{self._field_string}':[obj for obj in obj_list]})
    
    def _reload(self):
        if self._parent_type == ParentType.DOCUMENT:
            self._parent.reload(self._list_field_name)
            list_value = getattr(self._parent, self._list_field_name)
        elif self._parent_type == ParentType.EMBED:
            self._parent.reload(self._parent_field_name)
            embed_value = getattr(self._parent, self._parent_field_name)
            list_value = getattr(embed_value, self._list_field_name)
        elif self._parent_type == ParentType.EMBED_LIST_ELEMENT:
            self._parent.reload(self._parent_field_name)
            embed_list = getattr(self._parent, self._parent_field_name)
            embed_value = embed_list[int(self._parent_field_index)]
            list_value = getattr(embed_value, self._list_field_name)
            self._test_and_fix_ref()
        elif self._parent_type == ParentType.EMBED_DICT_ELEMENT:
            self._parent.reload(self._parent_field_name)
            embed_list = getattr(self._parent, self._parent_field_name)
            embed_value = embed_list[self._parent_field_index]
            list_value = getattr(embed_value, self._list_field_name)
            self._test_and_fix_ref()
        
        setattr(self, "_list_instance", list_value)

    def _test_and_fix_ref(self):
        # check if weak ref is broken in case of a nested embedded document
        try:
            hasattr(self._list_instance._instance, "__weakref__")
        except:
            self._list_instance._instance = proxy(self._parent)

class ListProxy:
    def __init__(self, list_wrapper: ListFieldWrapper,   callable: Callable):
        self._list_wrapper = list_wrapper
        self._callable = callable

    def __getitem__(self, index):
        return self._callable(self._list_wrapper.__getitem__(index))
    
    def __setitem__(self, index, object):
        return self._list_wrapper.__setitem__(index, object.model)
    
    def __len__(self):
        return self._list_wrapper.__len__()
    
    def __iter__(self):
        yield self._callable(next(self._list_wrapper.__iter__()))

    def append(self, object):
        return self._list_wrapper.append(object.model)

    def pop(self, left=True):
        return self._callable(self._list_wrapper.pop(left))
    
    def extend(self, obj_list):
        return self._list_wrapper.extend([obj.model for obj in obj_list])

class DictFieldWrapper:
    def __init__(self, dict_instance: BaseDict, parent_instance: Document, field_string: str, parent_type: ParentType):
        self._dict_instance = dict_instance
        self._parent = parent_instance
        self._field_string = field_string
        self._parent_type = parent_type

        if parent_type == ParentType.DOCUMENT:
            self._dict_field_name = self._field_string
        elif parent_type == ParentType.EMBED:
            self._parent_field_name, self._dict_field_name = field_string.split("__")
        elif parent_type == ParentType.EMBED_LIST_ELEMENT or parent_type == ParentType.EMBED_DICT_ELEMENT:
            self._parent_field_name, self._parent_field_index, \
                self._dict_field_name = field_string.split("__")
            
            self._test_and_fix_ref()

    def __getitem__(self, key):
        self._reload()
        item = self._dict_instance[key]
        if isinstance(item, EmbeddedDocument):
            if self._parent_type == ParentType.DOCUMENT:
                return EmbedModelProxy(item, self._parent, f"{self._dict_field_name}__{key}",
                                    ParentType.DICT)
            else:
                raise NotImplementedError("current implementation doesn't support nested embedded documents")
        elif isinstance(item, Document):
            return ModelProxy(item)
        else:
            return item

    def __setitem__(self, key, value):
        self._dict_instance[key] = value
        self._parent.update(**{f"set__{self._field_string}__{key}":value})

    def __delitem__(self, key):
        del self._dict_instance[key]
        self._parent.update(**{f"unset__{self._field_string}__{key}":True})

    def __len__(self):
        self._reload()
        return len(self._dict_instance)

    def __iter__(self):
        self._reload()
        for key, item in self._dict_instance.items():
            if isinstance(item, EmbeddedDocument):
                if self._parent_type == ParentType.DOCUMENT:
                    yield EmbedModelProxy(item, self._parent, f"{self._dict_field_name}__{key}",
                                    ParentType.DICT)
                else:
                    raise NotImplementedError("current implementation doesn't support nested embedded documents")
            elif isinstance(item, Document):
                yield key, ModelProxy(item)
            else:
                yield key, item

    def _reload(self):
        if self._parent_type == ParentType.DOCUMENT:
            self._parent.reload(self._dict_field_name)
            dict_value = getattr(self._parent, self._dict_field_name)
        elif self._parent_type == ParentType.EMBED:
            self._parent.reload(self._parent_field_name)
            embed_value = getattr(self._parent, self._parent_field_name)
            dict_value = getattr(embed_value, self._dict_field_name)
        elif self._parent_type == ParentType.EMBED_LIST_ELEMENT:
            self._parent.reload(self._parent_field_name)
            embed_list = getattr(self._parent, self._parent_field_name)
            embed_value = embed_list[int(self._parent_field_index)]
            dict_value = getattr(embed_value, self._dict_field_name)
            self._test_and_fix_ref()
        elif self._parent_type == ParentType.EMBED_DICT_ELEMENT:
            self._parent.reload(self._parent_field_name)
            embed_map = getattr(self._parent, self._parent_field_name)
            embed_value = embed_map[self._parent_field_index]
            dict_value = getattr(embed_value, self._dict_field_name)
            self._test_and_fix_ref()
        
        setattr(self, "_dict_instance", dict_value)

    def _test_and_fix_ref(self):
        # check if weak ref is broken in case of a nested embedded document
        try:
            hasattr(self._dict_instance._instance, "__weakref__")
        except:
            self._dict_instance._instance = proxy(self._parent)

class DictProxy:
    def __init__(self, dict_wrapper: DictFieldWrapper,   callable: Callable):
        self._dict_wrapper = dict_wrapper
        self._callable = callable

    def __getitem__(self, key):
        return self._callable(self._dict_wrapper.__getitem__(key))
    
    def __setitem__(self, key, object):
        return self._dict_wrapper.__setitem__(key, object.model)
    
    def __delitem__(self, key):
        return self._dict_wrapper.__delitem__(key)
    
    def __len__(self):
        return self._dict_wrapper.__len__()
    
    def __iter__(self):
        yield self._callable(next(self._dict_wrapper.__iter__()))
        