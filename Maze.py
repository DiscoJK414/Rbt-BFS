import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

def loadImage(path):
    img=cv.imread(path)
    if(img is None):
        return FileNotFoundError("Image not Found")
    imgrgb=cv.cvtColor(img, cv.COLOR_BGR2RGB)
    return img,imgrgb

def detectMarkers(imgrgb):
    hsv=cv.cvtColor(imgrgb, cv.COLOR_RGB2HSV)
    kernel = np.ones((5,5), np.uint8)
    #Start
    lowergreen = np.array([35, 50, 50])
    uppergreen = np.array([85, 255, 255])
    premark1 = cv.morphologyEx(cv.inRange(hsv, lowergreen, uppergreen), cv.MORPH_CLOSE, kernel)
    Mark1=cv.moments(premark1)
    if Mark1["m00"] == 0:
        raise ValueError("Start not found")
    start=int(Mark1["m10"]/Mark1["m00"]), int(Mark1["m01"]/Mark1["m00"])


    #Target
    lowerblue = np.array([90, 50, 50])
    upperblue = np.array([150, 255, 255])
    premark2 = cv.morphologyEx(cv.inRange(hsv, lowerblue, upperblue), cv.MORPH_CLOSE, kernel)
    Mark2=cv.moments(premark2)
    if Mark2["m00"] == 0:
        raise ValueError("Target not found")
    goal = (int(Mark2["m10"]/Mark2["m00"]), int(Mark2["m01"]/Mark2["m00"]))

    return start,goal

def griddy(imgrgb):
    gray = cv.cvtColor(imgrgb, cv.COLOR_RGB2GRAY)
    # Convert to black (walls) and white (paths)
    binary = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, blockSize=11, C=2)
    kernel = np.ones((2,2), np.uint8)
    binary = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel)
    binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, kernel)

    grid = (binary == 255).astype(int)
    return grid

#Defining functions for adjacent nodes
def get_neighbors(a, b, goal):
    neighbors = [(a-1,b),(a+1,b),(a,b-1),(a,b+1)]
    # Sort neighbors by distance to goal (Euclidean)
    neighbors.sort(key=lambda x: (x[0]-goal[0])**2 + (x[1]-goal[1])**2)
    return neighbors

#Main BFS logic function
def BFS(grid,start,goal):  
    n=len(grid)
    m=len(grid[0])  
    #Creating a array for storing visited nodes
    marked=[]
    #Initialy marking every Node as False
    for i in range(0,n):
        rows=[]
        for j in range(0,m):
            rows.append(False)
        marked.append(rows)

    #Queue of nodes
    queue=[start]
    marked[start[0]][start[1]]=True #Start is marked

    #To store the order of nodes
    traceback={}
    explored=0

    while(len(queue)>0):
        #Pop the first element of queue
        a,b=queue.pop(0)
        explored+=1
        
        if((a,b)==goal):
            path=[]
            while((a,b)!=start):
                path.append((a,b))
                a,b=traceback[(a,b)]
            path.append(start)
            path.reverse()
            return path,explored
        #Adjacent Nodes
        Neightbours=Neightbours = get_neighbors(a,b,goal)
        for x,y in Neightbours:
            if(0<=x<n and 0<=y<m):
                if not marked[x][y] and grid[x][y]==1:
                    marked[x][y]=True
                    traceback[(x,y)]=(a,b)
                    queue.append((x,y))

    return None, explored

def find_nearest_path(grid, point, search_radius=30):
    x,y = point
    r,c=y,x
    # Ensure point is within grid boundaries
    n, m = grid.shape
    if not (0 <= r < n and 0 <= c < m):
        raise IndexError("Point is outside grid boundaries.")
        
    # Check current point
    if grid[r, c] == 1:
        return r, c

    # Search in a square area around the point
    for radius in range(1, search_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                new_r, new_c = r + dr, c + dc
                
                # Check boundaries
                if 0 <= new_r < n and 0 <= new_c < m:
                    if grid[new_r, new_c] == 1:
                        return new_r, new_c
    raise RuntimeError(f"Could not find a path pixel near the marker at {point}.")


def draw(imgrgb, path, thickness=5):
    img_copy = imgrgb.copy()
    
    # Use BGR color for OpenCV, then convert back to RGB for matplotlib
    color = (255, 0, 0)  # Red in BGR
    
    # Convert path to drawing coordinates (swap columns for OpenCV)
    path_points = []
    for point in path:
        # Swap row/col to x/y and ensure integer coordinates
        path_points.append([point[1], point[0]])  # (x, y) = (col, row)
    
    path_np = np.array(path_points, dtype=np.int32).reshape((-1, 1, 2))
    
    # Draw the path
    cv.polylines(img_copy, [path_np], isClosed=False, color=color, thickness=thickness, lineType=cv.LINE_AA)
    
    # Also draw start and end points for visibility
    cv.circle(img_copy, (path_points[0][0], path_points[0][1]), 8, (0, 255, 0), -1)  # Green start
    cv.circle(img_copy, (path_points[-1][0], path_points[-1][1]), 8, (0, 0, 255), -1) # Red end
    
    return img_copy

def solve_maze(imagepath):
    imgbgr,imgrgb=loadImage(imagepath)
    start1,goal1=detectMarkers(imgrgb)
    grid=griddy(imgrgb)
    start = find_nearest_path(grid, start1, search_radius=30)
    goal = find_nearest_path(grid, goal1, search_radius=30)
    path,explored = BFS(grid,start,goal)
    if path is None:
        print("No path found!")
        return
    
    print("Path length   :", len(path))
    print("Nodes explored   :", explored)

    solved = draw(imgrgb, path)
    plt.imshow(solved)
    plt.title("Solved Maze")
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    solve_maze("Mazepic.png")


