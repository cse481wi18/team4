#! /usr/bin/env python

import sys, os
 

main_menu_actions  = {}  
create_program_menu_actions = {}


def main():
    os.system('clear')
    
    print 'Welcome,\n'
    print 'Please choose the option you want:\n' # user inputs should be numbers corresponding to their option
    print '1. Create program\n'
    print '2. Quit\n'
    choice = raw_input(' >>  ')
    exec_menu(main_menu_actions, choice)
    return

def exec_menu(menu_options, choice):
    os.system('clear')
    ch = choice.lower()
    if ch == '':
        menu_options['main_menu']()
    else:
        try:
            menu_options[ch]()
        except KeyError:
            print 'Invalid selection, please try again:\n'
            choice = raw_input(' >>  ')
            exec_menu(menu_options, choice)
    return

def create_new_program():
    print 'Relaxing arm...\n'
    # TODO call relax arm
    return creating_program()

def creating_program():
    print 'Creating program...\n'
    print '1. Save Pose\n'
    print '2. Open Grip\n'
    print '3. Close Grip\n'
    choice = raw_input(' >>  ')
    return exec_menu(create_program_actions, choice)

def save_pose():
    print 'Should the pose be relative to the base frame or to a tag?\n'
    print '0. base frame\n'
    curr_tags = get_tags()
    for key, val in enumerate(curr_tags):
        print '{}. Tag {}'.format(key, val)
    choice = raw_input(' >>  ')
    curr_tags.update({'0': '-1'}) # add base frame option

    # todo - call thingy
    print 'Position saved\n'
    creating_program()

def get_tags():
    """
    return map of current tags

    :return: format: {
    1: tag 3,
    2: tag 8
    }
    ...
    """
    return {}

# Back to main menu
def back():
    main_menu_actions['main_menu']()

# Exit program
def exit():
    sys.exit()

def open_grip():
    # todo
    creating_program()
    pass
def close_grip():
    # todo
    creating_program()
    pass

main_menu_actions = {
    'main_menu': main,
    '0': exit,
}

# print '1. Save Pose'
#     print '2. Open Grip'
#     print '3. Close Grip'
create_program_actions = {
    '1': save_pose,
    '2': open_grip,
    '3': close_grip,
}


# Main Program
if __name__ == '__main__':
    #Launch main menu
    main()
