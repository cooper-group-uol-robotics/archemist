from importlib import import_module
from pkgutil import iter_modules

def import_stations_models():
    for module_itr in iter_modules(path=__path__,prefix=f'{__name__}.'):
        model_module = f'{module_itr.name}.model'
        import_module(model_module)