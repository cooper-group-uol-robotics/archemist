from transitions import Machine

class SomeSM():

    def __init__(self, batch_mode):
        #self.rack_holder_location = rack_holder_location
        #self.load_location = load_location
        self.batch_mode = batch_mode

        states = ['start', 'retrieve_batch', 'place_batch', 'load_sample', 'station_process', 'unload_sample', 'finish']
        self.machine = Machine(self, states=states, initial='start')
        #self.machine.add_ordered_transitions(loop=False)
        self.machine.add_transition('proceed','start','retrieve_batch')

    def get_batch_mode(self):
        return self.batch_mode