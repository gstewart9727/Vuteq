# Filename      : Surface_Analysis.py
# Version       : 0.0.0
# Version Date  :
# Programmer    : Gabriel Stewart
# Description   : 


# Import libraries
import numpy as np
import ctypes   
import threading
import tkinter as tk
from tkinter import font, ttk, filedialog
import interactive_visualization as iv
import register as rg
from multiprocessing import Process,Queue,Pipe
import subprocess
import os
import win32gui
import time
from PIL import ImageTk, Image



class Application(tk.Frame):

    # Function      : __init__
    # Parameters    : 
    # Returns       : None
    # Description   : Initializer function. Calls functions to generate required GUI elements
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.grid()
        self.create_window()
        self.create_widgets()

        # Create queues
        self.task_queue = Queue()
        self.done_queue = Queue()
        self.responseHandler()

    # Function      : create_window
    # Parameters    : 
    # Returns       : None
    # Description   : Initializer function. Calls functions to generate required GUI elements
    def create_window(self):
        # Define sizes for gui window
        w = 350
        h = 1080

        # Get screen sizes
        ws = root.winfo_screenwidth() + 3
        hs = root.winfo_screenheight() - 70

        # calculate x and y coordinates for the Tk root window
        x = -10
        y = 0

        self.visualFrame = tk.Frame(root, width=938, height=500, relief=tk.RAISED, borderwidth=1)
        self.optionsFrame = tk.Frame(root, relief=tk.RAISED, borderwidth=1)
        self.outputFrame = tk.Frame(root, height=125, relief=tk.RAISED, borderwidth=1)
        self.visualFrame.grid(row=0, column=0, sticky="nes", pady=5, padx=5)
        self.optionsFrame.grid(row=0, column=1, sticky="nws", pady=5, padx=5)
        self.outputFrame.grid(row=1, columnspan=2, sticky="ew", padx=5, pady=5)
        self.outputFrame.grid_propagate(False)

        # Create Labels for Settings Section
        self.statusLabel = tk.Label(self.optionsFrame, text='Operation Info')
        self.statusLabel.grid(row=0, column=1, sticky="w")
        self.stageLabel = tk.Label(self.optionsFrame, text='  Stage:')
        self.stageLabel.grid(row=1, column=1, sticky="w")
        self.stageValue = tk.Label(self.optionsFrame, fg='Blue', text='Idle')
        self.stageValue.grid(row=1, column=2, sticky="w")
        self.sourceLabel = tk.Label(self.optionsFrame, text='Source Data')
        self.sourceLabel.grid(row=3, column=1, sticky="w")
        self.sourcePoints = tk.Label(self.optionsFrame, text='Points:')
        self.sourcePoints.grid(row=4, column=1, sticky="w")
        self.sourceDownPoints = tk.Label(self.optionsFrame, text='DS Points:')
        self.sourceDownPoints.grid(row=5, column=1, sticky="w")
        self.targetLabel = tk.Label(self.optionsFrame, text='Target Data')
        self.targetLabel.grid(row=6, column=1, sticky="w")
        self.targetPoints = tk.Label(self.optionsFrame, text='Points:')
        self.targetPoints.grid(row=7, column=1, sticky="w")
        self.targetDownPoints = tk.Label(self.optionsFrame, text='DS Points:')
        self.targetDownPoints.grid(row=8, column=1, sticky="w")
        self.modeLabel = tk.Label(self.optionsFrame, text='Mode')
        self.modeLabel.grid(row=10, column=1, sticky="w")
        self.thresholdLabel = tk.Label(self.optionsFrame, text='Deviation Threshold')
        self.thresholdLabel.grid(row=12, column=1)
        self.thresholdTxt1 = tk.Label(self.optionsFrame, text='Deviation below ')
        self.thresholdTxt1.grid(row=13, column=1, sticky='e')
        self.thresholdTxt2 = tk.Label(self.optionsFrame, text='is permitted')
        self.thresholdTxt2.grid(row=13, column=3, sticky='w')
        self.toleranceLabel = tk.Label(self.optionsFrame, text='Deviation Tolerance')
        self.toleranceLabel.grid(row=14, column=1)
        self.toleranceTxt1 = tk.Label(self.optionsFrame, text='Allow ')
        self.toleranceTxt1.grid(row=15, column=1, sticky='e')
        self.toleranceTxt2 = tk.Label(self.optionsFrame, text='deviated points')
        self.toleranceTxt2.grid(row=15, column=3, sticky='w')
        self.cropLocation = tk.Label(self.optionsFrame, text='Crop ROI File: ')
        self.cropLocation.grid(row=17, column=1, sticky='e')
        self.targetLocation = tk.Label(self.optionsFrame, text='Target CAD File: ')
        self.targetLocation.grid(row=18, column=1, sticky='e')

        # Create seperators
        s1 = ttk.Separator(self.optionsFrame, orient='horizontal')
        s1.grid(row=9, columnspan=5, sticky="ew", pady=5)
        s2 = ttk.Separator(self.optionsFrame, orient='horizontal')
        s2.grid(row=16, columnspan=5, sticky="ew", pady=5)
        s3 = ttk.Separator(self.optionsFrame, orient='horizontal')
        s3.grid(row=19, columnspan=5, sticky="ew", pady=5)

        # Place logo
        im = Image.open('VUTEQ-Logo-1.jpg')
        resized = im.resize((300, 100),Image.ANTIALIAS)
        tkimage = ImageTk.PhotoImage(resized)
        logoLabel = tk.Label(self.optionsFrame, image=tkimage)
        logoLabel.image = tkimage
        logoLabel.grid(row=20,columnspan=5)   

        # Create font
        f = font.Font(self.statusLabel, self.statusLabel.cget("font"))
        f.configure(underline=True)
        self.statusLabel.configure(font=f)
        self.sourceLabel.configure(font=f)
        self.targetLabel.configure(font=f)
        self.modeLabel.configure(font=f)

        # Create window and title
        root.title("Surface Analysis")
        root.geometry('%dx%d+%d+%d' % (ws, hs, x, y))
        root.resizable(0, 0) 

    def create_widgets(self):

        # Create file entries
        self.cropFile = tk.Entry(self.optionsFrame)
        self.cropFile.insert(0, '..\Training Mold\cropped_1.ply')
        self.cropFile.grid(row=17, column=2)
        self.CADfile = tk.Entry(self.optionsFrame)
        self.CADfile.insert(0, '..\Training Mold\Main-Refined.ply')
        self.CADfile.grid(row=18, column=2)

        # Create Mode buttons
        quickButton = tk.Button(self.optionsFrame, text="Quick", width=10, command=self.setQuickMode)
        quickButton.grid(row=11, column=1)
        verboseButton = tk.Button(self.optionsFrame, text="Verbose", width=10, command=self.setVerboseMode)
        verboseButton.grid(row=11, column=2)
        trainButton = tk.Button(self.optionsFrame, text="Train", width=10, command=self.train)
        trainButton.grid(row=11, column=3)
        cropButton = tk.Button(self.optionsFrame, text="Browse", width=10, command=lambda : self.selectFile(self.cropFile))
        cropButton.grid(row=17, column=3, pady=5)
        targetButton = tk.Button(self.optionsFrame, text="Browse", width=10, command=lambda : self.selectFile(self.CADfile))
        targetButton.grid(row=18, column=3, pady=5)

        # Create Deviation Entries
        self.devThresh = tk.Entry(self.optionsFrame)
        self.devThresh.grid(row=13, column=2)
        self.devThresh.insert(10, 3.5)
        self.devTol = tk.Entry(self.optionsFrame)
        self.devTol.grid(row=15, column=2)
        self.devTol.insert(10, 10)

        # Create output text box
        self.outputText = tk.Text(self.outputFrame)
        self.outputText.grid(row=0, columnspan=2, sticky="nsew")
        # self.outputText.configure(state="disabled")

    def setQuickMode(self):
        print("Enabled quick analysis")

        # Start process to perform registration and return results
        p = Process(target=rg.run, args=(self.task_queue, self.done_queue, self.devThresh.get(), self.devTol.get(), False, self.CADfile.get(), self.cropFile.get())) 
        p.start() 

    def setVerboseMode(self):
        print("Enabled verbose analysis")
        self.devThreshVal = self.devThresh.get()
        self.devTolVal = self.devTol.get()

        # Start process to perform registration and return results
        p = Process(target=rg.run, args=(self.task_queue, self.done_queue, self.devThresh.get(), self.devTol.get(), True, self.CADfile.get(), self.cropFile.get())) 
        p.start() 

    def selectFile(self, entryBox):

        # Open select file dialog
        self.filename =  filedialog.askopenfilename(initialdir = "./",title = "Select ply file",filetypes = (("ply files","*.ply"),("all files","*.*")))
        entryBox.insert(0, self.filename)


    def train(self):
        p = Process(target=iv.crop_geometry, args=())   
        p.start()

    def responseHandler(self):

        if (self.done_queue.empty() == False):
            response = self.done_queue.get()
            print (response)

            fields = response.split('|')
            if (fields[0] == 'stage'):
                self.stageValue['text'] = fields[1]
            elif (fields[0] == 'time'):
                self.outputText.insert(tk.END, fields[1] + "\n")
            elif (fields[0] == 'result'):
                self.outputText.insert(tk.END, fields[1] + "\n")
            elif (fields[0] == 'sourcePoints'):
                self.sourcePoints['text'] = fields[1]
            elif (fields[0] == 'targetPoints'):
                self.targetPoints['text'] = fields[1]
            elif (fields[0] == 'sourcePointsDS'):
                self.sourceDownPoints['text'] = fields[1]
            elif (fields[0] == 'targetPointsDS'):
                self.targetDownPoints['text'] = fields[1]
            elif (fields[0] == 'finish'):
                self.stageValue['text'] = 'Idle'    

        self.after(25, self.responseHandler)        
        
            

if __name__ == "__main__":
    root = tk.Tk()
    app = Application(master=root)
    app.mainloop()