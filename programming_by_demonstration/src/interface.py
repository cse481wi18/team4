#! /usr/bin/env python

import sys, os
 

main_menu_actions  = {}  
create_program_menu_actions = {}


def main():
    os.system('clear')
    
    print "Welcome,\n"
    print "Please choose the option you want:" #user inputs should be numbers corresponding to their option
    print "1. Create program"
    print "2. Save program"
    print "\n0. Quit"
    choice = raw_input(" >>  ")
    exec_menu(choice)
    return

def exec_menu(choice):
    os.system('clear')
    ch = choice.lower()
    if ch == '':
        menu_actions['main_menu']()
    else:
        try:
            menu_actions[ch]()
        except KeyError:
            print "Invalid selection, please try again.\n"
            menu_actions['main_menu']()
    return

def create_program():
    print "Ready to create program...\n"
    # TODO call relax arm
    print "Should the pose be relative to the base frame or to a tag?"
    print ". Back"
    print "0. Quit"
    choice = raw_input(" >>  ")
    exec_menu(choice)
    return



def save_program():
    print "if the pose should be relative to the base frame or to a tag?"
    print "9. Back"
    print "0. Quit" 
    choice = raw_input(" >>  ")
    exec_menu(choice)
    return

# Back to main menu
def back():
    menu_actions['main_menu']()

# Exit program
def exit():
    sys.exit()

menu_actions = {
    'main_menu': main_menu,
    '1': menu1,
    '2': menu2,
    '9': back,
    '0': exit,
}


# Main Program
if __name__ == "__main__":
    #Launch main menu
    main()
