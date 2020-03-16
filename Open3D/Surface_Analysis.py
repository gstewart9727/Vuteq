# Filename      : Surface_Analysis.py
# Version       : 0.0.0
# Version Date  :
# Programmer    : Gabriel Stewart
# Description   : 


# Import libraries
import numpy as np
import ctypes   
import threading
import tkinter as ttk
from tkinter.messagebox import showinfo
import interactive_visualization as iv
import register as rg
from multiprocessing import Process,Queue,Pipe
import subprocess


class Application(ttk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.grid()
        self.create_window()
        self.create_widgets()
        self.parent_conn, self.child_conn = Pipe()

    def create_window(self):
        # Define sizes for gui window
        w = 350
        h = 1080

        # Get screen sizes
        ws = root.winfo_screenwidth()
        hs = root.winfo_screenheight()

        # calculate x and y coordinates for the Tk root window
        x = ws - w
        y = 0

        # Create underline font
        # f = ttk.Font(l, l.cget("font"))
        # f.configure(underline = True)

        # Create Labels for Settings Section
        statsLabel = ttk.Label(root, text='Operation Info').grid(row=1, column=1)
        # statsLabel.configure(font=f)
        stageLabel = ttk.Label(root, text='Stage:').grid(row=2, column=1)
        sourceLabel = ttk.Label(root, text='Source Data').grid(row=3, column=1)
        sourcePoints = ttk.Label(root, text='Points:').grid(row=4, column=1)
        targetLabel = ttk.Label(root, text='Target Data').grid(row=5, column=1)
        targetPoints = ttk.Label(root, text='Points:').grid(row=6, column=1)
        modeLabel = ttk.Label(root, text='Mode').grid(row=8, column=1)
        thresholdLabel = ttk.Label(root, text='Deviation Threshold').grid(row=10, column=1)
        thresholdTxt1 = ttk.Label(root, text='Deviation below ').grid(row=11, column=1)
        thresholdTxt2 = ttk.Label(root, text='is permitted').grid(row=11, column=3)
        toleranceLabel = ttk.Label(root, text='Deviation Tolerance').grid(row=12, column=1)
        toleranceTxt1 = ttk.Label(root, text='Allow ').grid(row=13, column=1)
        toleranceTxt2 = ttk.Label(root, text='deviated points').grid(row=13, column=3)

        # Create window and title
        root.title("Vuteq - Surface Analysis")
        root.geometry('%dx%d+%d+%d' % (w, h, x, y))
        root.resizable(0, 0) 

    def create_widgets(self):
        # Create Mode buttons
        quickButton = ttk.Button(root, text="Quick", command=self.setQuickMode).grid(row=9, column=1)
        verboseButton = ttk.Button(root, text="Verbose", command=self.setVerboseMode).grid(row=9, column=2)
        trainButton = ttk.Button(root, text="Train", command=self.train).grid(row=9, column=3)

        # Create Deviation Entries
        self.devThresh = ttk.Entry(root)
        self.devThresh.grid(row=11, column=2)
        self.devThresh.insert(10, 3.5)
        self.devTol = ttk.Entry(root)
        self.devTol.grid(row=13, column=2)
        self.devTol.insert(10, 10)

    def setQuickMode(self):
        print("Enabled quick analysis")
        self.devThreshVal = self.devThresh.get()
        self.devTolVal = self.devTol.get()
        p = Process(target=rg.run, args=(self.child_conn, self.devThreshVal, self.devTolVal, False))   
        p.start()     

    def setVerboseMode(self):
        print("Enabled verbose analysis")
        self.devThreshVal = self.devThresh.get()
        self.devTolVal = self.devTol.get()
        p = Process(target=rg.run, args=(self.child_conn, self.devThreshVal, self.devTolVal, True))   
        p.start()
        print(self.parent_conn.recv())

    def train(self):
        iv.demo_crop_geometry()

if __name__ == "__main__":
    root = ttk.Tk()
    app = Application(master=root)
    app.mainloop()