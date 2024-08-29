import pyspacemouse
import time


def button_0(state, buttons, pressed_buttons):
    print("Button:", pressed_buttons)


def button_0_1(state, buttons, pressed_buttons):
    print("Buttons:", pressed_buttons)


def someButton(state, buttons):
    print("Some button")


def callback():

    # success = pyspacemouse.open(dof_callback=pyspacemouse.print_state, button_callback=pyspacemouse.print_buttons)
    success = pyspacemouse.open()
    if success:
        while True:
            state = pyspacemouse.read() 
            print(state.buttons)
            time.sleep(0.01)


if __name__ == '__main__':
    callback()