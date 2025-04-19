from PIL import Image
import numpy as np
import os

def read_txt(file_path):
    data = []
    with open(file_path, 'r') as file:
        for rows in file:
            row = rows.split()
            float_row = [float(item) for item in row]
            data.append(float_row[3:6])
    return data

class Canvas:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.pixels = [[0 for _ in range(width)] for _ in range(height)]
        self.cursor = [width/2, height/2]
        self.next_point = self.cursor
        
    def draw_line(self,x1,x2,y1,y2):
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if dx > 0 else -1
        sy = 1 if dy > 0 else -1

        err = dx - dy
        while True:
            self.pixels[int(y1)][int(x1)] = 255
            
            if x1 == x2 and y1 == y2:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
                
    def clear_img(self):
        self.pixels = [[0 for _ in range(self.width)] for _ in range(self.height)]
        
    def print_img(self):
        for row in self.pixels:
            for pixel in row:
                print('*' if pixel else ' ', end='')
            print()
            
def read_all_files(directory):
    file_contents = {}
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        if os.path.isfile(filepath):
            with open(filepath, "r", encoding="utf-8") as f:
                file_contents[filename] = f.read()
    return file_contents
            
            
data = read_all_files("./TraningData_9_5/")
for filename, content in data.items():

    gyro = read_txt("./TraningData_9_5/"+filename)

    canvas = Canvas(36,36)

    x_constant = 0
    y_constant = 0

    for dot in gyro:
        x_constant += abs(dot[2])
        y_constant += abs(dot[0])

    x_constant = 17 / x_constant
    y_constant = 17 / y_constant
    print(filename)

    for dot in gyro:
        dot[2] *= x_constant
        dot[0] *= y_constant
        canvas.next_point[0] -= dot[0]
        canvas.next_point[1] += dot[2]
        canvas.draw_line(canvas.cursor[0],canvas.next_point[0],canvas.cursor[1],canvas.next_point[1])
        canvas.cursor = canvas.next_point
        
    canvas.print_img()

    image = np.array(canvas.pixels)
    image = Image.fromarray(image)
    #image.show()