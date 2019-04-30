from tkinter import Tk, Label, Button, Canvas
import math
import numpy as np
import matplotlib.pyplot as plt


class MapGUI:
    # Initialization
    def __init__(self, master):
        self.master = master
        master.title("Map GUI")
        master.geometry("150x150")

        #Buttons
        self.import_map_button = Button(master, text="Import Map", command = self.showMap)
        self.import_map_button.pack()

        self.read_text_button = Button(master, text="Read Data", command = self.readData)
        self.read_text_button.pack()

        self.close_button = Button(master, text="Close", command = master.quit)
        self.close_button.pack()

    # Retrieve distance-angle pairs from a text file
    def readData(self):
        coords = []
        with open("./readings.txt", "rt") as f:
          for line in f:
            try:
              reading = line.split()

              coord = [float(reading[0]), float(reading[1])]
              coords.append(coord)

            except:
                print('Error encountered while reading file')
        return coords

    # Draw map on plot
    def showMap(self):
        measurements = self.readData()
        coords = []
        x = []
        y = []

        # Turn distance-angle pairs into usable coordinates
        for measurement in measurements:
          x.append(self.getXCoord(measurement))
          y.append(self.getYCoord(measurement))

        plt.scatter(x, y)
        plt.show()
        print('Map Generated')


    # Convert angles and measurements into a plot coordinate
    def getCoords(self, measurement):
        x = measurement[0] * math.cos(math.radians(measurement[0]))
        y = measurement[1] * math.sin(math.radians(measurement[0]))
        return x, y

    def getXCoord(self, measurement):
        x = measurement[0] * math.cos(math.radians(measurement[0]))
        return x

    def getYCoord(self, measurement):
        y = measurement[1] * math.sin(math.radians(measurement[0]))
        return y


root = Tk()
my_gui = MapGUI(root)
root.mainloop()
