#! /usr/bin/env python

import sys, os, time
 

# main_menu_actions  = {}
# create_program_menu_actions = {}


def main():
    os.system('clear')
    print 'Welcome!'
    print 'Please choose the option you want:' # user inputs should be numbers corresponding to their option
    print '1. Create program'
    print '2. Quit'
    choice = raw_input('  >>>  ')
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
            choice = raw_input('  >>>  ')
            exec_menu(menu_options, choice)
    return

def create_new_program():
    print 'Relaxing arm...'
    # TODO call relax arm
    return creating_program()

def creating_program():
    print 'Creating program...'
    print '1. Save Pose'
    print '2. Open Grip'
    print '3. Close Grip'
    print '4. Save Program'
    choice = raw_input('  >>>  ')
    return exec_menu(create_program_actions, choice)

def save_pose():
    print 'Should the pose be saved relative to the base frame or to a tag?'
    print '0. base frame'
    curr_tags = get_tags()
    for key, val in enumerate(curr_tags):
        print '{}. Tag {}'.format(key, val)
    choice = int(raw_input('  >>>  '))
    # check bounds:
    if choice > len(curr_tags) or choice < 0:
        print 'Invalid choice.'
        save_pose()
    curr_tags.update({'0': '-1'}) # add base frame option

    # todo - call thingy
    print 'Pose saved'
    creating_program()

def save_program():
    # todo save program
    print 'Program saved!'
    time.sleep(1)
    main()

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

def open_grip():
    print 'Opening grip...'
    # todo
    creating_program()
    pass

def close_grip():
    print 'Closing grip...'
    # todo
    creating_program()
    pass

# Exit program
def exit():
    sys.exit()

main_menu_actions = {
    'main_menu': main,
    '1': create_new_program,
    '2': exit,
}

# print '1. Save Pose'
#     print '2. Open Grip'
#     print '3. Close Grip'
create_program_actions = {
    '1': save_pose,
    '2': open_grip,
    '3': close_grip,
    '4': save_program,
}


# Main Program
if __name__ == '__main__':
    main()
