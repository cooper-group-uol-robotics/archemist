import pickle
class rosMsgCoder:
    def code(self, dict):
        str = pickle.dumps(dict)
        return str

    def decode(self, str):
        dict = pickle.loads(str)
        return dict
