import os 
import matplotlib.pyplot as plt 
import matplotlib.image as mp_img


SPAWN_POINT = 4
MIN_TRIAL = 1
MAX_TRIAL = 5

def plot():
    img = mp_img.imread('/home/nwarren317/ros2_ws/f24_robotics/Homework1/apartment.png')
    figure, axis = plt.subplots()
    axis.imshow(img, extent=[-0.85, 11.45, -0.5, 9.9])
    

    for trial in range(MIN_TRIAL, MAX_TRIAL + 1):
        file_path = f"spawn{SPAWN_POINT}_trial{trial}.txt"
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                paths = f.readlines()
                #splits the strings into two arrays for each x and y in the file
                x_vals, y_vals = zip(*[map(float, line.strip().split(',')) for line in paths]) 

                #saves max point
                max_x = x_vals[0]
                max_y = y_vals[0]
                
                #stuff
                distance = 0
                max_distance = 0

                #calculates total distance
                for x in range(len(x_vals)):
                    if x > 0:
                        distance+=get_distance(x_vals[x-1],y_vals[x-1],x_vals[x],y_vals[x])
                        dist_to_start = get_distance(x_vals[0], y_vals[0], x_vals[x], y_vals[x])
                        if(dist_to_start > max_distance):
                            max_x = x_vals[x]
                            max_y = y_vals[x]
                            max_distance = dist_to_start

                axis.plot(x_vals, y_vals, marker='o', label=f'Trial {trial}')
        print(f"Max Point ({max_x},{max_y})")
        print(f"Total Dist = {distance}")
        print(f"Max Dist = {max_distance}")
    plt.title(f"Paths from start location {SPAWN_POINT}")
    plt.xlabel("X")
    plt.ylabel("Y")

    plt.xlim(0,10.5)
    plt.ylim(0,9.3)
    plt.grid()
    plt.legend()

    save_path = f"spawn_{SPAWN_POINT}_paths.png"
    plt.savefig(save_path)
    plt.close(figure)

def get_distance(x1,y1,x2,y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

if __name__ == '__main__':
    plot()