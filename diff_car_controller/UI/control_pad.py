#test.py
try:
    import tkinter as tk  # for python 3
except:
    import Tkinter as tk  # for python 2
import pygubu


class Application:
    def __init__(self):

        #1: Create a builder
        self.builder = builder = pygubu.Builder()

        #2: Load an ui file
        builder.add_from_file('control_pad.ui')

        #3: Create the widget using a master as parent
        self.mainwindow = builder.get_object('runapp')
        #self.configwindow = builder.get_object('configapp')
        
        # Connect to Delete event
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.quit)
        
        #4: Preserve the master
        #self.master = self.runwindow
        
        #5: Connect Callbacks
        self.builder.connect_callbacks(self)
        
    def quit(self, event=None):
        self.mainwindow.quit()
    
    def run(self):
        #self.runwindow.mainloop()
        self.mainwindow.mainloop()
        
    def on_run_mode_button(self):
        # : change the windows
        print("in the on_run_mode_button")
        #self.mainwindow.destroy()
        self.mainwindow = self.builder.get_object('runapp')
        #self.builder.connect_callbacks(self)
        pass
        
    def on_config_mode_button(self):
        print("in the on_config_mode_button")
        #self.mainwindow.destroy()
        self.mainwindow = self.builder.get_object('configapp')
        #self.mainwindow.destroy()
        pass
        
if __name__ == '__main__':
    #root = tk.Tk()
    app = Application()
    app.run()

