from archemist.persistence.persistenceManager import PersistenceManager
import multiprocessing as mp
from time import sleep
from panda_handler_emu import EmuPandaHandler
from ika_handler_emu import EmuIkaPlateRCTDigital_Handler
from fisher_handler_emu import EmuFisherWeightingStation_Handler

def run_panda_handler():
    p_manager = PersistenceManager('test')
    state = p_manager.construct_state_from_db()
    robot = state.get_robot('PandaFranka', 1)
    panda_handler = EmuPandaHandler(robot)
    panda_handler.run()

def run_ika_handler():
    p_manager = PersistenceManager('test')
    state = p_manager.construct_state_from_db()
    station = state.get_station('IkaPlateRCTDigital', 2)
    ika_handler = EmuIkaPlateRCTDigital_Handler(station)
    ika_handler.run()

def run_fisher_handler():
    p_manager = PersistenceManager('test')
    state = p_manager.construct_state_from_db()
    station = state.get_station('FisherWeightingStation', 5)
    finsher_handler = EmuFisherWeightingStation_Handler(station)
    finsher_handler.run()


if __name__ == '__main__':
    try:
        # launch handlers this assumes the state was constructed from a config file before hand
        handlers = [run_panda_handler,run_ika_handler,run_fisher_handler]
        procs = [mp.Process(target=handler) for handler in handlers]
        for proc in procs:
            proc.daemon = True
            proc.start()
        print('here')
        while any(proc.is_alive() for proc in procs):
            sleep(0.1)
    except KeyboardInterrupt:
        for proc in procs:
            proc.terminate()
            proc.join()
    finally:
        print('All processes terminated')
