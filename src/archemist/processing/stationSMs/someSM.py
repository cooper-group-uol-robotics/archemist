from transitions import Machine

class SomeSM(Machine):

    def __init__(self):
        #self.rack_holder_location = rack_holder_location
        #self.load_location = load_location
        #self.batch_mode = batch_mode

        states = ['start', 'retrieve_batch', 'place_batch', 'load_sample', 'station_process', 'unload_sample', 'finish']
        Machine.__init__(self, states=states, initial='start')
        self.add_ordered_transitions(loop=False)