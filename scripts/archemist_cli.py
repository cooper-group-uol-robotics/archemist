from __future__ import print_function, unicode_literals
from PyInquirer import prompt, print_json
from time import sleep
import zmq

main_menu = [
    {
        'type': 'list',
        'name': 'main_menu',
        'message': 'Please select from one of the options below:',
        'choices': ['Start/Resume', 'Pause','Add clean batch','Start charging', 'Stop charging', 'Resume LBR app', 'Terminate']
    }
]

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.connect('tcp://127.0.0.1:5555')
    while True:
        try:
            selection = prompt(main_menu)
            if selection['main_menu'] == 'Start/Resume':
                print('Staring workflow')
                socket.send_string('start')
            elif selection['main_menu'] == 'Pause':
                print('Pausing workflow')
                socket.send_string('pause')
            elif selection['main_menu'] == 'Add clean batch':
                print('Adding clean batch to the workflow')
                socket.send_string('add_batch')
            elif selection['main_menu'] == 'Start charging':
                print('Order manual charge')
                socket.send_string('charge')
            elif selection['main_menu'] == 'Stop charging':
                print('Stoping the charging proces')
                socket.send_string('stop_charge')
            elif selection['main_menu'] == 'Resume LBR app':
                print('Resuming the arm application')
                socket.send_string('resume_app')
            elif selection['main_menu'] == 'Terminate':
                print('Terminating workflow')
                socket.send_string('terminate')
                break
            sleep(.1)
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    main()

