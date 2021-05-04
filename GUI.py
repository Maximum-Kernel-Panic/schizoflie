
import tkinter as tk
from tkinter import messagebox
import random
import threading
import time
import sys
from controller_multi import Controller

class SimpleWindu(threading.Thread):
    def __init__(self,controller):
        threading.Thread.__init__(self)
        self.rate = 100 # [Hz]
        self.activejoy=False
        self.daemon = True
        self.controller=controller

    def _add_components(self):
        self.upFrame=tk.Frame(self.window)
        self.botFrame=tk.Frame(self.window)
        self.coordFrame=tk.Frame(self.upFrame)
        self.posFrame=tk.Frame(self.upFrame)
        self.buttonFrame=tk.Frame(self.botFrame)
        self.coordFrame.pack(side=tk.LEFT)
        self.posFrame.pack(side=tk.TOP)
        self.buttonFrame.pack(side=tk.LEFT)
        self.upFrame.pack(side=tk.TOP,fill=tk.BOTH)
        self.botFrame.pack(side=tk.BOTTOM)
        posLbl=tk.Label(self.posFrame,text="Current Position:",font=("Arial",15))
        posLbl.pack()

        def flyclick():
            fly.configure(bg="green")
            land.configure(bg="red")
            self.controller.takeoff()
            self.xdisp.delete(0,tk.END)
            self.xdisp.insert(tk.END,round(self.controller.setPointX,2))
            self.ydisp.delete(0,tk.END)
            self.ydisp.insert(tk.END,round(self.controller.setPointY,2))
            self.zdisp.delete(0,tk.END)
            self.zdisp.insert(tk.END,round(self.controller.setPointZ,2))
        def landclick():
            fly.configure(bg="red")
            land.configure(bg="green")
            self.controller.flying=False
            messagebox.showinfo(title=None,message="Landing the Crazyflie!")

        def killclick():
            self.controller.KILL=True
            messagebox.showwarning(title=None,message="KILLSWITCH ENGAGE!")

        def joystick_mode():
            if not self.activejoy:
                self.controller.enable_joystick_mode()
                joystick.configure(bg="green")
                print("lel")
                self.activejoy=True
            else:
                self.controller.disable_joystick_mode()
                joystick.configure(bg="red")
                self.activejoy=False

        def send_ref(self):
            inputX=float(self.xdisp.get())
            inputY=float(self.ydisp.get())
            inputZ=float(self.zdisp.get())
            if (abs(inputX)<10 and abs(inputY)<10 and abs(inputZ)<2):
                self.controller.setRef(inputX,inputY,inputZ)
                self.oldX=inputX
                self.oldY=inputY
                self.oldZ=inputZ
                print("hi")
            else:
                messagebox.showerror(title="Bad Values",message="Setpoint out of bounds!")
                self.xdisp.delete(0,tk.END)
                self.xdisp.insert(tk.END,self.oldX)
                self.ydisp.delete(0,tk.END)
                self.ydisp.insert(tk.END,self.oldY)
                self.zdisp.delete(0,tk.END)
                self.zdisp.insert(tk.END,self.oldZ)

        fly=tk.Button(self.buttonFrame,text="Take off",command=flyclick)
        fly.pack(side=tk.LEFT)
        land=tk.Button(self.buttonFrame,text="Land",command=landclick)
        land.pack(side=tk.RIGHT)
        joystick=tk.Button(self.botFrame,text="Joystick Mode",command=joystick_mode)
        joystick.pack(side=tk.RIGHT)
        kill=tk.Button(self.botFrame,text="KILLSWITCH",command=killclick)
        kill.pack(side=tk.RIGHT)

        def add_setpoint_fields(self):
            setlbl=tk.Label(self.coordFrame,text="Current Set-Points:",font=("Arial",12))
            setlbl.pack(side=tk.TOP)
            xlbl=tk.Label(self.coordFrame,text="X:",font=("Arial",11))
            xlbl.pack()
            self.xdisp=tk.Entry(self.coordFrame, width=6, bg="light yellow")
            self.xdisp.insert(tk.END,"0.0")
            self.oldX=0
            self.xdisp.bind("<Return>", (lambda event: send_ref(self)))
            self.xdisp.pack()
            ylbl=tk.Label(self.coordFrame,text="Y:",font=("Arial",11))
            ylbl.pack()
            self.ydisp=tk.Entry(self.coordFrame, width=6, bg="light yellow")
            self.ydisp.insert(tk.END,"0.0")
            self.oldY=0
            self.ydisp.bind("<Return>", (lambda event: send_ref(self)))
            self.ydisp.pack()
            zlbl=tk.Label(self.coordFrame,text="Z:",font=("Arial",11))
            zlbl.pack()
            self.zdisp=tk.Entry(self.coordFrame, width=6, bg="light yellow")
            self.zdisp.insert(tk.END,"0.0")
            self.oldZ=0
            self.zdisp.bind("<Return>", (lambda event: send_ref(self)))
            self.zdisp.pack()

        def add_refpos_fields(self):
            rFrame=tk.Frame(self.posFrame)
            rFrame.pack(side=tk.LEFT)
            lFrame=tk.Frame(self.posFrame)
            lFrame.pack(side=tk.RIGHT)
            xlbl=tk.Label(rFrame,text="Current X:")
            xlbl.pack()
            self.xref=tk.Label(lFrame,text="0.0")
            self.xref.pack()
            ylbl=tk.Label(rFrame,text="Current Y:")
            ylbl.pack()
            self.yref=tk.Label(lFrame,text="0.0")
            self.yref.pack()
            zlbl=tk.Label(rFrame,text="Current Z:")
            zlbl.pack()
            self.zref=tk.Label(lFrame,text="0.0")
            self.zref.pack()

        add_refpos_fields(self)
        add_setpoint_fields(self)

    def updateref(self):
        self.xref.configure(text="{}".format(round(self.controller.x,2)))
        self.yref.configure(text="{}".format(round(self.controller.y,2)))
        self.zref.configure(text="{}".format(round(self.controller.z,2)))
        self.window.after(250,self.updateref)

    def run(self):
        """Control loop"""
        try:
            self.window = tk.Tk()
            self.window.title("Crazyflie Drone")
            self.window.geometry('350x200')
            self._add_components()
            self.window.after(0,self.updateref)
            self.window.mainloop()
        except (KeyboardInterrupt):
            pass

    def loop_sleep(self, timeStart):
        """ Sleeps the control loop to make it run at a specified rate """
        deltaTime = 1.0/float(self.rate) - (time.time() - timeStart)
        if deltaTime > 0:
            time.sleep(deltaTime)
        else:
            print('Could not make controller loop deadline')


if __name__ == '__main__':
    windu=SimpleWindu()
