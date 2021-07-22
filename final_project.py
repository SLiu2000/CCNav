from CCNav import CCNav, point_pos_2, in_range, point_pos

import pygame
import socket
from math import sin, cos, radians, pi, hypot, degrees, atan
import time

pygame.init()

# Display parameters
window_size = 1000
max_dist = 3

# Set up the drawing window
screen = pygame.display.set_mode([window_size, window_size])

HOST = '10.42.0.1'  # The server's hostname or IP address
PORT = 60000  # The port used by the server


def to_display_scale(x : ()) -> ():
    return (x[0] * (window_size / max_dist) * 1/2) + (window_size / 2), (-x[1] * (window_size / max_dist) * 1/2) + (window_size / 2)


def to_map_scale(x : ()) -> ():
    return (x[0] - (window_size / 2)) * (max_dist / (window_size / 2)), ((window_size / 2) - x[1]) * (max_dist / (window_size / 2))

def draw_lidar_offset(data: [], window, offset : int, colour=(0, 0, 255)):
    data_len = len(data)
    for i, d in enumerate(data):
        if not d == 0:
            x, y = point_pos_2(d, (i / data_len) * 360 - 90 + offset)
            pygame.draw.circle(window, colour, to_display_scale((x, y)), 2)


def get_data(s):
    # Wait for data from server
    data = None
    while (data is None) or not (len(data) == 523 or len(data) == 533):
        data = s.recv(8192)
        try:
            data = [float(i) if len(i) > 1 else 0 for i in data.decode("utf-8").split(',')]
        except:
            data = []
    return data[:3], data[3:]


def capture_landmarks(window, data: [], landmark_count: int) -> [()]:
    landmarks = []
    group_counts = []
    data_len = len(data)
    for _ in range(landmark_count):
        chosen_points = []
        while 1:
            # Fill the background with white
            window.fill((255, 255, 255))
            # Did the user click the window close button?
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            # Draw canter
            pygame.draw.circle(window, (255, 0, 0), (window_size / 2, window_size / 2), 5)
            # Draw selected points
            for point in chosen_points:
                pygame.draw.circle(window, (255, 0, 0), to_display_scale(point), 3)
            # Draw datapoints
            draw_lidar_offset(data, window, 0)
            # Flip the display
            pygame.display.update()
            # Check for mouse click
            if pygame.mouse.get_pressed()[0]:
                x, y = pygame.mouse.get_pos()
                chosen_points.append(to_map_scale((x, y)))
                time.sleep(.35)
            if len(chosen_points) > 1:
                chosen = []
                for i, d in enumerate(data):
                    if not d == 0:
                        x, y = point_pos(0, 0, d, (i / data_len) * 360)
                        if in_range(chosen_points[0], chosen_points[1], (x, y)):
                            chosen.append((x, y))
                avg_x = sum([i[0] for i in chosen]) / len(chosen)
                avg_y = sum([i[1] for i in chosen]) / len(chosen)
                landmarks.append((avg_x, avg_y))
                group_counts.append(len(chosen))
                print("Added landmark @{x} meters away".format(x=hypot(avg_x, avg_y)))
                break
    return landmarks, group_counts


# Run until the user asks to quit
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print('Connected to robot')
    landmarks = []
    # Discover landmarks
    mouse_positions = []
    initial_position, lidar_data = get_data(s)
    s.close()
    initial_landmarks, initial_group_count = capture_landmarks(screen, lidar_data, 4)

print(initial_landmarks)
print(initial_group_count)
# Create CCNav object
nav = CCNav(initial_landmarks, initial_group_count, tolerance=.35)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    count = 0
    while True:
        _, lidar_data = get_data(s)
        print("Displaying [{i}]".format(i=count))

        nav.update(lidar_data)

        print("Global location: " + str(nav.get_location()))
        print("Global heading: " + str(nav.get_global_rotational_offset()))

        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Fill the background with white
        screen.fill((255, 255, 255))

        new_landmarks = nav.get_landmarks()

        # draw_landmarks(new_landmarks, screen, .3, draw_area=False, draw_connections=True)
        # Draw center
        pygame.draw.circle(screen, (255, 0, 0), (window_size / 2, window_size / 2), 5)

        # Draw robot heading
        robot_heading = point_pos_2(.2, nav.get_global_rotational_offset() - 90)
        pygame.draw.line(screen, (0, 0, 255), (window_size / 2, window_size / 2),
                         to_display_scale((robot_heading[0], -robot_heading[1])))

        # Draw global origin
        pygame.draw.circle(screen, (0, 255, 0), (to_display_scale((-nav.get_location()[0], -nav.get_location()[1]))), 5)

        # Draw datapoints
        # draw_lidar(data, screen)
        draw_lidar_offset(lidar_data, screen, -nav.get_global_rotational_offset())

        # Update the display
        pygame.display.update()



# Done! Time to quit.
pygame.quit()