import numpy as np
import mcl_helper as mh

# Constants
GRID_WIDTH = 96 + 2 # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2 # inches (one extra inch on each side for border)
ppi = 3
WINDOW_WIDTH = GRID_WIDTH * ppi
WINDOW_HEIGHT = GRID_HEIGHT * ppi

def init_grid():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space
    grid[4:46, 4:10] = 0
    grid[4:22, 10:22] = 0
    grid[4:10, 22:40] = 0
    grid[28:40, 28:34] = 0
    grid[4:22, 40:46] = 0
    grid[16:22, 46:64] = 0
    grid[4:46, 64:70] = 0
    grid[40:46, 10:64] = 0
    grid[16:22, 64:88] = 0
    grid[4:46, 88:94] = 0

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1
    
    # Set Loading Zone
    grid[4:22, 4:22] = 7
    
    # Set Drop off Zones
    grid[4:13, 64:70] = 8
    grid[4:13, 88:94] = 8
    grid[28:37, 28:34] = 8
    grid[37:46, 88:94] = 8

    return grid

def init_loading_zone_grid():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set Loading Zone
    grid[3:23, 3:23] = 7
    
    # Set free movable space
    grid[20:47, 3:11] = 270
    grid[3:11, 20:47] = 180
    grid[11:23, 39:47] = 270
    grid[15:23, 47:95] = 180
    grid[3:15, 63:71] = 90
    grid[39:47, 11:71] = 180
    grid[27:39, 27:35] = 90
    grid[23:39, 63:71] = 270
    grid[3:15, 87:95] = 90
    grid[23:47, 87:95] = 270

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid

def init_drop_off_zone_grid1():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[4:46, 4:10] = 270 #270
    grid[3:11, 22:47] = 180 #180
    grid[11:23, 39:47] = 270 #270
    grid[15:23, 47:95] = 180 #180
    grid[3:15, 63:71] = 90 #90
    grid[39:47, 10:71] = 180 #180
    grid[23:39, 63:71] = 270 #270
    grid[3:15, 87:95] = 90 #90
    grid[23:47, 87:95] = 270 #270
    
    
    grid[3:22, 4:23] = 180 #180
    grid[3:40, 3:11] = 90 # 90
    grid[39:47, 3:28] = 360 #360
    grid[28:47, 27:35] = 270 #270
    
    # Define drop off zone 1
    grid[27:37, 27:35] = 8
    

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1
    

    return grid

def init_drop_off_zone_grid2():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    
    grid[4:10, 22:46] = 180 #180
    grid[10:22, 40:46] = 270 #270
    grid[15:23, 46:95] = 180 #180
    grid[4:16, 64:70] = 90 #90
    grid[39:47, 10:71] = 180 #180
    grid[22:40, 63:71] = 270 #270
    grid[3:15, 87:95] = 90 #90
    grid[23:47, 87:95] = 270 #270
    grid[4:47, 3:11] = 270 #270
    
    
    grid[4:23, 3:23] = 270 #270
    grid[4:10, 4:40] = 360 #360
    grid[3:17, 39:47] = 90 #90
    grid[15:23, 39:64] = 360 #360
    grid[4:23, 63:71] = 270 #270
    grid[3:11, 3:39] = 360 #360
    
    # Define drop off zone 2
    grid[4:13, 63:71] = 8
    

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid


def init_drop_off_zone_grid3():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[3:47, 3:11] = 270
    grid[3:10, 21:47] = 180
    grid[9:23, 39:47] = 270
    grid[15:23, 45:95] = 180
    grid[3:16, 63:71] = 90
    grid[39:47, 11:71] = 180
    grid[22:39, 63:71] = 270
    grid[3:16, 87:95] = 90
    grid[3:47, 87:95] = 270
    
    
    grid[4:23, 4:23] = 270
    grid[3:11, 3:40] = 360
    grid[3:16, 39:47] = 90
    grid[15:23, 39:87] = 360
    
    # Define drop off zone 3
    grid[3:11, 87:95] = 8

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid

def init_drop_off_zone_grid4():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[3:47, 3:11] = 270
    grid[3:10, 21:47] = 180
    grid[9:23, 39:47] = 270
    grid[15:23, 45:95] = 180
    grid[3:16, 63:71] = 90
    grid[39:47, 11:71] = 180
    grid[22:39, 63:71] = 270
    grid[3:16, 87:95] = 90
    grid[3:47, 87:95] = 90
    
    grid[4:23, 4:23] = 270
    grid[3:11, 3:40] = 360
    grid[3:16, 39:47] = 90
    grid[15:23, 39:87] = 360
   
    # Define drop off zone 4
    grid[39:47, 87:95] = 8
    

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid

def display_grid(reduced_grid):
    for i in range(len(reduced_grid)):
        for j in range(len(reduced_grid[0])):
            if reduced_grid[i][j] == 0:
                print(" ", end=" ")
            elif reduced_grid[i][j] == 1:
                print("-", end=" ")
            elif reduced_grid[i][j] == 2:
                print(" ", end=" ")
            elif reduced_grid[i][j] == 7:
                print("L", end=" ")
            elif reduced_grid[i][j] == 8:
                print("B", end=" ")
            elif reduced_grid[i][j] == 90:
                print("↓", end=" ")
            elif reduced_grid[i][j] == 180:
                print("←", end=" ")
            elif reduced_grid[i][j] == 270:
                print("↑", end=" ")
            elif reduced_grid[i][j] == 360:
                print("→", end=" ")
        print()
        
# grid = init_grid()
# display_grid(grid)

# loading_zone = init_loading_zone_grid()
# display_grid(loading_zone)
# print(loading_zone[41][12])

drop_directions = init_drop_off_zone_grid3()
display_grid(drop_directions)