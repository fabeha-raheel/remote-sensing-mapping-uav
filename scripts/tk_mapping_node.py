#!/usr/bin/env python

import tkinter as tk
import tkintermapview
from tkinter import ttk
import pickle

import drone_data

class MappingApp(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.title('Landmine Mapping GUI')
        self.geometry('500x500')

        self.locations = drone_data.landmines
        self.inital_zoom = 20
        self.initial_position = (-35.3632621, 149.1652374)
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
                text = "Detection " + str(index + 1)
                self.markers.append(self.map_widget.set_marker(location[1][0], location[1][1], text=text))
        elif len(self.markers) < len(self.locations):
            for i in range(len(self.locations)):

                if i < len(self.markers):
                    self.markers[i].set_position(self.locations[i][1][0], self.locations[i][1][1])
                else:
                    index = i
                    text = "Detection " + str(index + 1)
                    self.markers.append(self.map_widget.set_marker(self.locations[i][1][0], self.locations[i][1][1], text=text))
        else:
            for location in self.locations:
                index = self.locations.index(location)
                self.markers[index].set_position(location[1][0], location[1][1])

        # self.map_widget.pack()
        self.map_widget.after(100, self.update_map)
    
    def read_data(self):
        f = open('/home/ugv/rtab_ws/src/remote-sensing-mapping-uav/logs/data.pickle', 'rb')
        data = pickle.load(f)
        print("Mapping Landmine : ", data)
        self.locations = data
        f.close()
        # with open('/home/ugv/lmd_ws/src/lmd_sim/logs/lmd_data.pickle', 'rb') as f:
        #     data = pickle.load(f)
        #     self.locations = data
        


root = MappingApp()
root.mainloop()