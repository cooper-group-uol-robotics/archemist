class Location:
    def __init__(self, name: str, node_id: int, graph_id: int, map_id: int, desk_pos: str):
        self._name = name
        self._node_id = node_id
        self._graph_id = graph_id
        self._map_id = map_id
        self._desk_pos = desk_pos

    @property
    def name(self):
        return self._name

    @property
    def node_id(self):
        return self._node_id

    @property
    def graph_id(self):
        return self._graph_id

    @property
    def map_id(self):
        return self._map_id
