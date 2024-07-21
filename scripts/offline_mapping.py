#!/usr/bin/env python3
import tkinter as tk
import tkintermapview
from tkinter import ttk
import pickle

import drone_data

MAPPING_LOG = '/home/ugv/rtab_ws/src/remote-sensing-mapping-uav/logs/data.pickle'

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

        self.create_map()
    
    def create_map(self):

        self.read_data()
        for location in self.locations:
            index = self.locations.index(location)
            text = "Landmine " + str(index + 1)
            self.markers.append(self.map_widget.set_marker(location[1][0], location[1][1], text=text))
    
    # def read_data(self):
    #     f = open(MAPPING_LOG, 'r')
    #     data = list(f.read())
    #     print("Mapping Landmine : ", data)
    #     self.locations = data
    #     f.close()

    def read_data(self):
        f = open(MAPPING_LOG, 'rb')
        data = pickle.load(f)
        print("Mapping Landmine : ", data)
        self.locations = data
        f.close()

root = MappingApp()
root.mainloop()