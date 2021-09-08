import pickle
import codecs
class rosMsgCoder:
    def code(self, dict):
        str = codecs.encode(pickle.dumps(dict), "base64").decode()
        return str

    def decode(self, str):
        dict = pickle.loads(codecs.decode(str.encode(), "base64"))
        return dict
