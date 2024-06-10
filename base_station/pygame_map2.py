import pygame
import sys

import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

from scipy.ndimage import gaussian_filter
from constants import Object, coord_list, mandatory_points, object_map, color_map




# Set the dimensions of the screen (10m x 5m) => (1000 x 500)
# 1m => 100px
screen_width = 1000
screen_height = 400


WHITE = (220, 220, 220)
BLACK = (0, 0, 0)
LINECOLOR = (0, 0, 0)

pygame.init()


font = pygame.font.SysFont(None, 20)
drawing = False
start_pos = None
end_pos = None


undo_surfaces = []


def draw_obj_rect(surface: pygame.Surface, center_x: int, center_y: int, obj: Object):
    pygame.draw.rect(surface, obj.color, (center_x, center_y - obj.width, obj.width, obj.height))

def draw_obj_circle(surface: pygame.Surface, center_x: int, center_y: int, obj: Object):
    pygame.draw.circle(surface, obj.color, (center_x, center_y), obj.radius)

def draw_map():
    surface = pygame.Surface((screen_width, screen_height))
    surface.fill(WHITE)
    for id, tag, coord  in coord_list:
        center_x = int(coord[0] * 100)  # Scale coordinates
        center_y = -int(coord[1] * 100)
        required = id in mandatory_points
        obj = object_map[tag]
 
        if tag == 'A1':
            if required:
                pygame.draw.rect(surface, BLACK, (center_x - 2, center_y - 17, 19, 19))
            draw_obj_rect(surface, center_x, center_y, obj)
        if tag == 'A2':
            draw_obj_rect(surface, center_x, center_y, obj)
        if tag == 'B1':
            if required:
                pygame.draw.circle(surface, BLACK, (center_x, center_y), 12)
            draw_obj_circle(surface, center_x, center_y, obj)
        if tag == 'B2':
            draw_obj_circle(surface, center_x, center_y, obj)
        if tag == 'T':
            draw_obj_rect(surface, center_x, center_y, obj)
        if tag == 'SP':
            draw_obj_rect(surface, center_x, center_y, obj)
        if tag == 'WP':
            draw_obj_rect(surface, center_x, center_y, obj)
        if tag == 'FP':
            draw_obj_circle(surface, center_x, center_y, obj)
            draw_obj_rect(surface, center_x, center_y, object_map["F"])
    return surface


def get_cost_map(surface):
    surfacemap = pygame.surfarray.pixels3d(surface)
    costmap = np.full((screen_height, screen_width), 100)
    # print(costmap.shape)
    for i, pxarray in enumerate(surfacemap):
        for j, px in enumerate(pxarray):
            color = tuple(px)
            if color != WHITE and color != BLACK:
                tag = color_map[color]
                costmap[j][i] = object_map[tag].cost

    # Give high costs on borders
    costmap[0:2,:].fill(0)
    costmap[-1:-3,:].fill(0)
    costmap[:,0:2].fill(0)
    costmap[:,-1:-3].fill(0)

    sigma = 10  # Adjust sigma for different blur levels
    # costmap = gaussian_filter(costmap, sigma=sigma).astype('int8')
    return costmap

def save_pgm(filename, data):
    height, width = data.shape
    max_val = np.max(data)

    with open(filename, 'wb') as f:
        f.write(b'P5\n')  # PGM magic number for binary gray map
        f.write(f'{width} {height}\n'.encode())  # Image width and height
        f.write(f'{max_val}\n'.encode())  # Maximum pixel value
        f.write(data.astype(np.uint8).tobytes())  # Image data


def grayscale_map(map):
    return map

map_surface = draw_map()
costmap = get_cost_map(map_surface)
grayscale = grayscale_map(costmap).astype('uint8')
# print(grayscale)
save_pgm('gray.pgm', grayscale)
# image = Image.open('input_image.jpg')

# Convert the image to grayscale
# gray_image = image.convert('L')

def draw_heatmap():
    #np.savetxt('inp.txt', costmap)
    #x = np.loadtxt('inp.txt')

    sns.heatmap(costmap)
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.show()

def calculate_angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    angle = np.arctan2(y2 - y1, x2 - x1)
    return angle


def setwaypoints():
    pygame.init()
    saved_coords = []
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("ISRO Map")

    canvas_surface = pygame.Surface((screen_width, screen_height), pygame.SRCALPHA, 32)
    canvas_surface = canvas_surface.convert_alpha()

    
    # cost_surface = pygame.Surface((screen_width, screen_height), pygame.SRCALPHA, 32)
    # cost_surface = cost_surface.convert_alpha()
    # Main loop
    while True:
        
        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                print(saved_coords)
                # pygame.quit()
                return saved_coords
                # sys.exit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                pos = (pos[0]/100, -pos[1]/100)
                saved_coords.append(pos)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_u:
                    if undo_surfaces:
                        canvas_surface = undo_surfaces.pop()
                        saved_coords.pop()
        
        screen.blit(map_surface, (0, 0))
        for id, tag, coord  in coord_list:
            center_x = int(coord[0])  # Scale coordinates
            center_y = -int(coord[1])
            
            text_surface = font.render(str(id), True, BLACK)
            text_rect = text_surface.get_rect(center=(center_x, center_y))
            screen.blit(text_surface, text_rect)
        
        screen.blit(canvas_surface, (0, 0))

        if len(saved_coords) > 2:
            pygame.draw.lines(screen, BLACK, False, saved_coords, 10)

        pos = pygame.mouse.get_pos()
        pygame.draw.circle(screen, LINECOLOR, pos, 50, 1)
        
        # cost_surface.fill(RED)
        # screen.blit(cost_surface, (0,0))

        pygame.display.flip()
        pygame.time.Clock().tick(60)

def setnav():
    pygame.init()
    saved_coord = []
    saved_angle = []
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("ISRO Map")

    canvas_surface = pygame.Surface((screen_width, screen_height), pygame.SRCALPHA, 32)
    canvas_surface = canvas_surface.convert_alpha()

    
    # cost_surface = pygame.Surface((screen_width, screen_height), pygame.SRCALPHA, 32)
    # cost_surface = cost_surface.convert_alpha()
    # Main loop
    first_click= False
    second_click = False
    while True:
        
        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                print(saved_coord)
                print(saved_angle)
                # pygame.quit()
                return saved_coord, saved_angle
                # sys.exit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                pos = (pos[0]/100, pos[1]/100)
                if not first_click:
                    saved_coord.append(pos)
                    first_click = True
                elif not second_click:
                    first_pos = saved_coord[-1]
                    angle = calculate_angle(first_pos, pos)
                    saved_angle.append(angle)
                    second_click = True
                undo_surfaces.append(canvas_surface.copy())
                pygame.draw.circle(canvas_surface, LINECOLOR, pos, 5)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_u:
                    if undo_surfaces:
                        canvas_surface = undo_surfaces.pop()
                        saved_coord.pop()
        
        screen.blit(map_surface, (0, 0))
        for id, tag, coord  in coord_list:
            center_x = int(coord[0])  # Scale coordinates
            center_y = -int(coord[1])
            
            text_surface = font.render(str(id), True, BLACK)
            text_rect = text_surface.get_rect(center=(center_x, center_y))
            screen.blit(text_surface, text_rect)
        
        screen.blit(canvas_surface, (0, 0))

        if len(saved_coord) > 2:
            pygame.draw.lines(screen, BLACK, False, saved_coord, 10)

        pos = pygame.mouse.get_pos()
        pygame.draw.circle(screen, LINECOLOR, pos, 50, 1)
        
        # cost_surface.fill(RED)
        # screen.blit(cost_surface, (0,0))

        pygame.display.flip()
        pygame.time.Clock().tick(60)


def get_map():
    return grayscale


# if __name__ == "__main__":
#     # draw_heatmap()
#     pass
