#!/usr/bin/env python

import tkinter as tk
import tkintermapview
from tkinter import ttk
import pickle
import rospkg
import sys

import drone_data

try:
    pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')
except:
    print("Unable to locate package path. Try sourcing your ROS workspace.")
    sys.exit()

class MappingApp(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.title('SAR Target Mapping GUI')
        self.geometry('500x500')

        self.locations = drone_data.targets
        self.inital_zoom = 20
        self.initial_position = (37.413534, -121.996561)
        self.markers = []

        self.labelFrame = tk.LabelFrame(self)
        self.labelFrame.pack(pady=20)

        self.map_widget = tkintermapview.TkinterMapView(self.labelFrame, width=1000, height=700, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellite
        # self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google normal
        self.map_widget.set_position(self.initial_position[0], self.initial_position[1])
        self.map_widget.set_zoom(self.inital_zoom)

        self.update_map()
        
        # self.mapFrame.pack(pady=10)
        # self.map_widget.pack()
    
    def slide(self):
        self.map_widget.set_zoom(self.zoom_slider.get())
    
    def update_map(self):
        self.read_data()
        # print("Updating Map...")
        # print("Landmines: ", self.locations)

        if len(self.markers) == 0:
            for location in self.locations:
                index = self.locations.index(location)
                text = "Target " + str(index + 1) + ": " + location[2]
                self.markers.append(self.map_widget.set_marker(location[1][0], location[1][1], text=text, text_color="white"))
        elif len(self.markers) < len(self.locations):
            for i in range(len(self.locations)):

                if i < len(self.markers):
                    self.markers[i].set_position(self.locations[i][1][0], self.locations[i][1][1])
                else:
                    index = i
                    text = "Target " + str(index + 1) + ": " + location[2]
                    self.markers.append(self.map_widget.set_marker(self.locations[i][1][0], self.locations[i][1][1], text=text, text_color="white"))
        else:
            for location in self.locations:
                index = self.locations.index(location)
                self.markers[index].set_position(location[1][0], location[1][1])

        # self.map_widget.pack()
        self.map_widget.after(100, self.update_map)
    
    def read_data(self):
        f = open(pkg_path + '/logs/data.pickle', 'rb')
        data = pickle.load(f)
        print("Mapping Landmine : ", data)
        self.locations = data
        f.close()
        


root = MappingApp()
root.mainloop()