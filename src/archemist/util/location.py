class Location:
    def __init__(self, node_id: int, graph_id: int, frame_name: str):
        self._node_id = node_id
        self._graph_id = graph_id
        self._frame_name = frame_name

    @property
    def node_id(self):
        return self._node_id

    @property
    def graph_id(self):
        return self._graph_id

    @property
    def frame_name(self):
        return self._frame_name
