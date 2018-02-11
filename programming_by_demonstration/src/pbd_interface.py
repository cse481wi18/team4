#! /usr/bin/env python

import sys, os, time
import fetch_api
from pbd_recorder import Recorder

"""
user inputs should be numbers corresponding to their option
"""
class Interface:
    def __init__(self):
        self._main_menu_actions = {
            'main_menu': self._main_menu,
            '1': self._create_new_program,
            '2': exit,
        }
        self._create_program_actions = {
            '1': self._save_pose,
            '2': self._open_grip,
            '3': self._close_grip,
            '4': self._save_program,
        }

    def run(self):
        self._grip = fetch_api.Gripper()
        self._recorder = Recorder()

        self._main_menu()

    def _main_menu(self):
        os.system('clear')
        print 'Welcome!'
        print 'Please choose the option you want: '
        print '1. Create program'
        print '2. Quit'
        choice = raw_input('  >>>  ')
        self._exec_menu(self._main_menu_actions, choice)
        return

    def _exec_menu(self, menu_options, choice):
        os.system('clear')
        ch = choice.lower()
        if ch == '':
            menu_options['main_menu']()
        else:
            try:
                menu_options[ch]()
            except KeyError:
                print 'Invalid selection, please try again:\n'
                choice = raw_input('  >>>  ')
                self._exec_menu(menu_options, choice)
        return

    def _create_new_program(self):
        print 'Relaxing arm...'
        self._recorder.arm_limp()
        return self._creating_program()

    def _creating_program(self):
        print 'Creating program...'
        print '1. Save Pose'
        print '2. Open Grip'
        print '3. Close Grip'
        print '4. Save Program'
        choice = raw_input('  >>>  ')
        return self._exec_menu(self._create_program_actions, choice)

    def _save_pose(self):
        print 'Should the pose be saved relative to the base frame or to a tag?'
        print '0. base frame'
        curr_tags = self._get_tags()
        for key, val in enumerate(curr_tags):
            print '{}. Tag {}'.format(key, val)
        choice = int(raw_input('  >>>  '))
        # check bounds:
        if choice > len(curr_tags) or choice < 0:
            print 'Invalid choice: {}'.format(choice)
            self._save_pose()
        curr_tags.update({0: -1})  # add base frame option
        self._recorder.record_pose(curr_tags[choice])
        print 'Pose saved!'
        self._creating_program()

    def _save_program(self):
        print 'Save program name as?'
        name = raw_input('  >>>  ')
        self._recorder.save_path(name)
        print 'Program saved!'
        time.sleep(0.5)
        self._main_menu()

    def _get_tags(self):
        """
        return map of current tags

        :return: format: {
        1: tag 3,
        2: tag 8
        }
        ...
        """
        tags = self._recorder.get_tags()
        dict_tags = {}
        for idx, tag in enumerate(tags):
            dict_tags[idx] = tag
        return dict_tags

    def open_grip(self):
        print 'Opening grip...'
        self._grip.open()
        self._creating_program()

    def close_grip(self):
        print 'Closing grip...'
        self._grip.close()
        self._creating_program()


# Exit program
def exit():
    sys.exit()

def main():
    torso = fetch_api.Torso()
    print 'Raising torso height...'
    torso.set_height(torso.MAX_HEIGHT)
    demo_runner = Interface()
    demo_runner.run()


# Main Program
if __name__ == '__main__':
    main()
