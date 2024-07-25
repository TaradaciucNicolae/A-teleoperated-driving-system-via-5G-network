import pygame
import socket
from time import sleep


def pygame_init():
    pygame.init()

    pygame.joystick.init()

    return pygame


def controller_init(pygame_controller):

    pygame_controller.init()

    pygame_controller.joystick.init()

    joystick_count = pygame_controller.joystick.get_count()

    if joystick_count == 0:
        print("No joysticks found.")
        return None
    else:

        controller = pygame.joystick.Joystick(0)
        controller.init()

        print(f"Joystick found: {controller.get_name()}")

        return controller


def controller_read_input(pygame_controller, controller, prev_axes, prev_buttons, prev_hat):

    pygame_controller.event.get()

    axes = [controller.get_axis(i) for i in range(controller.get_numaxes())]
    buttons = [controller.get_button(i) for i in range(controller.get_numbuttons())]
    hat = [controller.get_hat(0)[0]]

    if (buttons != prev_buttons) or (axes != prev_axes) or (hat != prev_hat):

        ready_to_send = 1
    else:
        ready_to_send = 0

    return axes, buttons, hat, ready_to_send


def controller_disconnect(pygame_controller):

    pygame_controller.quit()


def client_socket_init():


    host = '172.30.170.224'  # IP address of Rasp
    port = 6789

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client_socket.connect((host, port))
    return client_socket


def client_socket_send(client_socket, axes):
    axes_str = ','.join((map(str, axes)))

    client_socket.sendall(axes_str.encode())


def client_socket_disconnect(client_socket):

    client_socket.close()


def main():
    pygame_controller = pygame_init()
    client_socket = client_socket_init()
    controller = controller_init(pygame_controller)

    if controller is None:
        return
        
    prev_axes = [0.0] * controller.get_numaxes()
    prev_buttons = [0] * controller.get_numbuttons()

    hat_state = controller.get_hat(0)
    hat_state_length = len(hat_state)
    prev_hat = [0] * hat_state_length

    loop_on_off = True
    while loop_on_off:

        axes_controller, buttons_controller, hat_controller, ready_to_send = controller_read_input(pygame_controller,
                                                                                                   controller,
                                                                                                   prev_axes,
                                                                                                   prev_buttons,
                                                                                                   prev_hat)
        if ready_to_send == 1:



            hat_test = hat_controller[0]

            while hat_test != 0:
                axes_controller, buttons_controller, hat_controller, ready_to_send = controller_read_input(
                                                                                    pygame_controller,
                                                                                    controller,
                                                                                    prev_axes,
                                                                                    prev_buttons,
                                                                                    prev_hat)

                just_needed_axes = axes_controller[:1] + axes_controller[4:] + hat_controller
                rounded_axes = [round(num, 3) for num in just_needed_axes]
                client_socket_send(client_socket, rounded_axes)
                hat_test = hat_controller[0]
                print(hat_test)
                sleep(0.1)


            just_needed_axes = axes_controller[:1] + axes_controller[4:] + hat_controller
            rounded_axes = [round(num, 3) for num in just_needed_axes]
            client_socket_send(client_socket, rounded_axes)
            print(rounded_axes)

            prev_buttons = buttons_controller
            prev_axes = axes_controller
            prev_hat = hat_controller
            if any(buttons_controller):
                loop_on_off = False

    controller_disconnect(pygame_controller)
    client_socket_disconnect(client_socket)


if __name__ == '__main__':
    main()
