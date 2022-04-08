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

    def __eq__(self, o: object) -> bool:
        return o._node_id == self._node_id and o._graph_id == self._graph_id and o._frame_name == self._frame_name

    def get_map_coordinates(self):
        return (self._node_id, self._graph_id)

    def __str__(self):
        return 'node_id:{0}, graph_id{1}, frame_name:{2}'.format(self.node_id, self.graph_id, self.frame_name)

    def to_dict(self):
        return {'node_id':self._node_id, 'graph_id':self._graph_id, 'frame_name':self._frame_name}
