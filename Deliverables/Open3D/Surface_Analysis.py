# Filename      : Surface_Analysis.py
# Version       : 0.1
# Version Date  : 2020-03-26
# Programmer    : Gabriel Stewart
# Description   : This file contains the source code for the GUI class. The methods for creating and updating the GUI
#                 as well as interacting with child threads is contained here.


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

# Class         : GUI
# Description   : This class handles the creation and maintenance of the GUI. As well as starting/handling child threads
class GUI(object):

    # Function      : __init__
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : Initializer function. Calls functions to generate required GUI elements
    def __init__(self):
        # Create instance of Tkinter window
        root = self.root = tk.Tk()

        # Establish grid as the layout system for the window
        root.grid()
        self.create_window()
        self.create_widgets()

        # Create queues
        self.task_queue = Queue()
        self.done_queue = Queue()

        # Start registration process in its own thread, it will perform downsampling then wait for instructions
        self.p = Process(target=rg.run, args=(self.task_queue, self.done_queue, self.CADfile.get(), self.cropFile.get())) 
        self.p.start() 

        # Create handler for window closing
        root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Start handler for GUI updates
        self.responseHandler()
        

    # Function      : create_window
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : This function creates visual aspects of the GUI application
    def create_window(self):
        # Define sizes for gui window
        w = 350
        h = 1080

        # Get screen sizes
        ws = self.root.winfo_screenwidth() + 3
        hs = self.root.winfo_screenheight() - 70

        # calculate x and y coordinates for the Tk root window
        x = -10
        y = 0

        # Create and configure frames which will contain widgets for different purposes
        self.visualFrame = tk.Frame(self.root, width=938, height=500, relief=tk.RAISED, borderwidth=1)
        self.optionsFrame = tk.Frame(self.root, relief=tk.RAISED, borderwidth=1)
        self.outputFrame = tk.Frame(self.root, height=125, relief=tk.RAISED, borderwidth=1)
        self.visualFrame.grid(row=0, column=0, sticky="nes", pady=5, padx=5)
        self.optionsFrame.grid(row=0, column=1, rowspan=2, sticky="nws", pady=5, padx=5)
        self.outputFrame.grid(row=1, sticky="ew", padx=5, pady=5)

        # Change weight and propogation settings to allow manual resizing of columns
        self.outputFrame.grid_propagate(False)
        self.outputFrame.grid_columnconfigure(0, weight=1)
        self.optionsFrame.grid_rowconfigure(20, weight=1)

        # Create Labels for Settings Section
        self.statusLabel = tk.Label(self.optionsFrame, text='Operation Info')
        self.statusLabel.grid(row=0, column=1, sticky="w")
        self.stageLabel = tk.Label(self.optionsFrame, text='Stage:')
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
        logoLabel.grid(row=20, columnspan=5, rowspan=5)   

        # Create font
        f = font.Font(self.statusLabel, self.statusLabel.cget("font"))
        f.configure(underline=True)
        self.statusLabel.configure(font=f)
        self.sourceLabel.configure(font=f)
        self.targetLabel.configure(font=f)
        self.modeLabel.configure(font=f)

        # Create window and title
        self.root.title("Surface Analysis")
        self.root.geometry('%dx%d+%d+%d' % (ws, hs, x, y))
        self.root.resizable(0, 0) 

    # Function      : create_widgets
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : This function creates interactive aspects of the GUI application
    def create_widgets(self):

        # Create crop file and CAD file entry boxes
        sv1 = tk.StringVar()
        sv2 = tk.StringVar()
        sv1.trace("w", lambda name, index, mode, sv1=sv1: self.resetRegister())
        sv2.trace("w", lambda name, index, mode, sv2=sv2: self.resetRegister())
        self.cropFile = tk.Entry(self.optionsFrame, textvariable=sv1)
        self.cropFile.insert(0, '..\Training Mold\cropped_exp.ply')
        self.cropFile.grid(row=17, column=2)
        self.CADfile = tk.Entry(self.optionsFrame, textvariable=sv2)
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

        # Create deviation entry boxes
        self.devThresh = tk.Entry(self.optionsFrame)
        self.devThresh.grid(row=13, column=2)
        self.devThresh.insert(10, 3.5)
        self.devTol = tk.Entry(self.optionsFrame)
        self.devTol.grid(row=15, column=2)
        self.devTol.insert(10, 10)

        # Create output text box
        self.outputText = tk.Text(self.outputFrame)
        self.outputText.grid(row=0, columnspan=2, sticky="nsew")

    # Function      : setQuickMode
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : This function updates GUI elements pertaining to the stage and then passes
    #                 the task specifications on the message queue to the registration process
    def setQuickMode(self):
        self.outputText.delete('1.0', tk.END)
        self.stageValue['text'] = 'Running...'
        self.start = time.time()
        task = str(self.devThresh.get()) + '|' +  str(self.devTol.get()) + '|' + 'quick' + '|'
        self.task_queue.put(task)

    # Function      : setVerboseMode
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : This function updates GUI elements pertaining to the stage and then passes
    #                 the task specifications on the message queue to the registration process
    def setVerboseMode(self):
        self.outputText.delete('1.0', tk.END)
        self.stageValue['text'] = 'Running...'
        self.start = time.time()
        task = str(self.devThresh.get()) + '|' +  str(self.devTol.get()) + '|' + 'verbose' + '|'
        self.task_queue.put(task)

    # Function      : selectFile
    # Parameters    : entryBox - Reference to the entry box being updated in current context
    # Returns       : None
    # Description   : This function allows the user to select a file for various purposes
    def selectFile(self, entryBox): 
        # Open select file dialog
        filename =  filedialog.askopenfilename(initialdir = "./",title = "Select ply file",filetypes = (("ply files","*.ply"),("all files","*.*")))
        self.path = filename.replace('/', '\\')
        entryBox.delete(0, 'end')
        entryBox.insert(0, self.path)

    # Function      : train
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : Start Open3D provided applciation for gathering ROI file
    def train(self):

        # Display cropping instructions
        self.outputText.delete('1.0', tk.END)
        self.outputText.insert(tk.END, 'Manual geometry cropping' + "\n")
        self.outputText.insert(tk.END, '1) Press "Q" to exit live camera feed and begin cropping' + "\n")
        self.outputText.insert(tk.END, '2) Press "Y" twice to align geometry with negative direction of y-axis' + "\n")
        self.outputText.insert(tk.END, '3) Press "K" to lock screen and to switch to selection mode' + "\n")
        self.outputText.insert(tk.END, '4) Drag for rectangle selection, or use ctrl + left click for polygon selection' + "\n")
        self.outputText.insert(tk.END, '5) Press "C" to get a selected geometry and to save it' + "\n")
        self.outputText.insert(tk.END, '6) Press "F" to switch to freeview mode' + "\n")
        self.outputText.update()

        # Before we start a new process for gathering a ROI, we must first close the registration process
        if self.p.is_alive():
            self.p.terminate() 

        # Start crop geometry function in its own process
        p = Process(target=iv.crop_geometry, args=(self.done_queue,))   
        p.start()

        # After cropping process has completed, restart the registration process
        p.join()
        self.resetRegister()

    # Function      : responseHandler
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : This function processes responses from registration thread and updates GUI elements
    def responseHandler(self):
        
        # Wait for message in queue
        if (self.done_queue.empty() == False):
            # Receive and display message received
            response = self.done_queue.get()
            print (response)

            # Break message down my pipe delimiter
            fields = response.split('|')
            if (fields[0] == 'stage'):  # Message containing current stage
                self.stageValue['text'] = fields[1]
                self.stageValue.update()
            elif (fields[0] == 'time'): # Message containing time value
                self.outputText.insert(tk.END, fields[1] + "\n")
            elif (fields[0] == 'sourcePoints'): # Message containing number of points in source pointcloud
                self.sourcePoints['text'] = fields[1]
            elif (fields[0] == 'targetPoints'): # Message containing number of points in target pointcloud
                self.targetPoints['text'] = fields[1]
            elif (fields[0] == 'sourcePointsDS'):   # Message containing number of points in downsampled source pointcloud
                self.sourceDownPoints['text'] = fields[1]
            elif (fields[0] == 'targetPointsDS'):   # Message containing number of points in downsampled target pointcloud
                self.targetDownPoints['text'] = fields[1]
            elif (fields[0] == 'error'):   # Message containing error information
                # Display error information
                ctypes.windll.user32.MessageBoxW(0, fields[1], "ERROR", 0x1000)
                
                # If error message is regarding a filename, open file selection box
                if fields[2] == '1':
                    self.selectFile(self.CADfile)
                
                # Reset registration process
                self.resetRegister()
                self.stageValue['text'] = 'Idle'
            elif (fields[0] == 'result'):   # Message containing results of registration
                # Display time taken for process
                self.time_finish = time.time() - self.start
                self.outputText.insert(tk.END, 'Elapsed Time:\t\t{:.4f}'.format(self.time_finish) + ' seconds\n')

                # Display and highlight number of deviated points
                colortag = "color-" + "Red"
                self.outputText.tag_configure(colortag, foreground="Red")
                self.outputText.insert(tk.END, 'Deviated Points: ' + fields[2] + "\n", colortag)

                # If number of deviated points exceeds threshold, display warning
                if int(fields[2]) > int(self.devTol.get()):
                    ctypes.windll.user32.MessageBoxW(0, "Check mold surface! Obstruction Detected.", "WARNING", 0x1000)

        # Recursive function calls itself after 10ms
        self.root.after(10, self.responseHandler)   

    # Function      : resetRegister
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : This function attempts to close the queues and threads before restarting them
    #                 with updated parameters
    def resetRegister(self):
        try:
            self.task_queue.close()
            self.done_queue.close()
            if self.p.is_alive():
                self.p.terminate()  
            
            self.task_queue = Queue()
            self.done_queue = Queue()
            self.p = Process(target=rg.run, args=(self.task_queue, self.done_queue, self.CADfile.get(), self.cropFile.get())) 
            self.p.start()
        except AttributeError:
            self.task_queue = None
            self.done_queue = None

    # Function      : on_closing
    # Parameters    : Reference to class
    # Returns       : None
    # Description   : This function handles the window close event and shuts down application resources
    def on_closing(self):
        self.task_queue.close()
        self.done_queue.close()
        if self.p.is_alive():
            self.p.terminate()  

        self.root.destroy()

if __name__ == "__main__":
    gui = GUI()
    gui.root.mainloop()