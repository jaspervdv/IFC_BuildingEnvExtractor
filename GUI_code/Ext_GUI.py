import os
import time
import tkinter
from tkinter import ttk, filedialog, messagebox
import json
import subprocess
import threading
import re

from pathlib import Path

class LoDSettings:
    def __init__(self):
        self.lod00 = tkinter.IntVar(value=1)
        self.lod02 = tkinter.IntVar(value=1)
        self.lod03 = tkinter.IntVar(value=1)
        self.lod04 = tkinter.IntVar(value=1)
        self.lod10 = tkinter.IntVar(value=1)
        self.lod12 = tkinter.IntVar(value=1)
        self.lod13 = tkinter.IntVar(value=1)
        self.lod22 = tkinter.IntVar(value=1)
        self.lod30 = tkinter.IntVar(value=0)
        self.lod32 = tkinter.IntVar(value=0)
        self.lod50 = tkinter.IntVar(value=0)

    def hasLoD(self):
        if (not self.lod00.get() and not self.lod02.get() and not self.lod03.get() and not self.lod10.get()
                and not self.lod12.get() and not self.lod13.get() and not self.lod22.get() and not self.lod32.get()
                and not self.lod50.get()):
            return False
        return True

class FootprintSettings:
    def __init__(self):
        self.make_footprint = tkinter.IntVar(value=1)
        self.make_roofprint = tkinter.IntVar(value=1)
        self.footprint_based = tkinter.IntVar(value=0)

class DivSettings:
    def __init__(self):
        self.ignore_proxy = tkinter.IntVar(value=1)
        self.use_default = tkinter.IntVar(value=1)
        self.custom_enabled = tkinter.IntVar(value=0)
        self.simple_geo = tkinter.IntVar(value=0)

class OtherSettings:
    def __init__(self):
        self.make_interior = tkinter.IntVar(value=0)
        self.summary_voxels = tkinter.IntVar(value=0)
        self.make_report = tkinter.IntVar(value=1)


class Tooltip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tooltip_window = None
        self.after_id = None
        self.widget.bind("<Enter>", self.on_enter)
        self.widget.bind("<Leave>", self.on_leave)

    def on_enter(self, event=None):
        self.after_id = self.widget.after(500, self.show_tooltip)

    def on_leave(self, event=None):
        # Cancel tooltip if mouse leaves before it shows
        if self.after_id:
            self.widget.after_cancel(self.after_id)
            self.after_id = None
        self.hide_tooltip()

    def show_tooltip(self):
        if not self.tooltip_window:
            x, y, _, _ = self.widget.bbox("insert")
            x += self.widget.winfo_rootx() + 15
            y += self.widget.winfo_rooty() - 20
            self.tooltip_window = tkinter.Toplevel(self.widget)
            self.tooltip_window.wm_overrideredirect(True)
            self.tooltip_window.wm_geometry(f"+{x}+{y}")
            label = tkinter.Label(self.tooltip_window, text=self.text, background="white", borderwidth=1, relief="solid")
            label.pack()

    def hide_tooltip(self):
        if self.tooltip_window:
            self.tooltip_window.destroy()
            self.tooltip_window = None


def getDefaultDivObjects():
    return "IfcWall\tIfcCurtainWall\tIfcWallStandardCase\tIfcRoof\tIfcSlab\tIfcWindow\tIfcColumn\tIfcBeam\tIfcDoor\tIfcCovering\tIfcMember\tIfcPlate"


def toggleEnableDiv(widget, default_toggle, proxy_toggle, default_bool, proxy_bool):
    if  widget['state'] == "disabled":
        widget['state'] = tkinter.NORMAL
        widget['bg'] = 'SystemWindow'
        widget['fg'] = 'SystemWindowText'

        default_toggle["state"] = tkinter.DISABLED
        proxy_toggle["state"] = tkinter.DISABLED
    else:
        widget['state'] = tkinter.DISABLED
        widget['bg'] = "#F0F0F0"
        widget['fg'] = "#707070"

        default_toggle["state"] = tkinter.NORMAL
        proxy_toggle["state"] = tkinter.NORMAL

        updateDivMessage(widget, default_bool, proxy_bool)

    return


def toggleEnableEntry(entry_widget, additional_bool_list):
    for additional_bool in additional_bool_list:
        if  additional_bool:
            entry_widget['state'] = tkinter.NORMAL
            return
    entry_widget['state'] = tkinter.DISABLED
    return


def runCode(input_path,
            output_path,
            lod_settings,
            footprint_settings,
            div_settings,
            other_settings,
            footprint_elevation,
            voxel_size,
            div_string,
            bool_run):

    ifc2_exe_path = "Ifc_Envelope_Extractor_ifc2x3.exe"
    ifc4_exe_path = "Ifc_Envelope_Extractor_ifc4.exe"
    ifc4x3_exe_path = "Ifc_Envelope_Extractor_ifc4x3.exe"

    # check voxel input
    try:
        float(voxel_size)
    except:
        tkinter.messagebox.showerror("Settings Error", "Error: no valid voxel size supplied")
        return

    if(float(voxel_size) <= 0):
        tkinter.messagebox.showerror("Settings Error", "Error: voxel size should be larger than 0")
        return

    # check footprint input
    try:
        float(footprint_elevation)
    except:
        tkinter.messagebox.showerror("Settings Error", "Error: no valid footprint elevation supplied")
        return

    # check if an LoD output is selected
    if not(lod_settings.hasLoD()):
        tkinter.messagebox.showerror("Settings Error",  "Error: no LoD output selected")
        return

    if(lod_settings.lod02 and not footprint_settings.make_footprint and not footprint_settings.make_roofprint):
        tkinter.messagebox.showerror("Settings Error", "Error: no LoD0.2 footprint or roofoutline selected")
        return

    # check paths

    input_path_list = re.split(r'(?<!{) (?![^{]*})', input_path)
    input_path_list = [part.replace('{', '').replace('}', '') for part in input_path_list]

    for path in input_path_list:
        if(not os.path.isfile(path)):
            tkinter.messagebox.showerror("Settings Error",  "Error: No Valid input file supplied")
            return

    if(not os.path.isdir(os.path.dirname(output_path)) or len(output_path) == 0):
        tkinter.messagebox.showerror("Settings Error", "Error: No Valid output folder supplied\n (GUI can not create new folders)")
        return

    json_path = Path(input_path_list[0]).stem + "_config.json"

    # write data to json
    json_dictionary = {}

    json_dictionary["Filepaths"] = {}
    json_dictionary["Filepaths"]["Input"] = input_path_list
    json_dictionary["Filepaths"]["Output"] = output_path

    json_dictionary["Voxel"] = {}
    json_dictionary["Voxel"]["Size"] = float(voxel_size)
    json_dictionary["Voxel"]["Store values"] = other_settings.summary_voxels.get()

    json_dictionary["IFC"] = {}
    if not div_settings.custom_enabled.get():
        json_dictionary["IFC"]["Default div"] = div_settings.use_default.get()
        json_dictionary["IFC"]["Ignore proxy"] = div_settings.ignore_proxy.get()
        json_dictionary["IFC"]["Div objects"] = []
    else:
        json_dictionary["IFC"]["Default div"] = False
        json_dictionary["IFC"]["Ignore proxy"] = False
        json_dictionary["IFC"]["Div objects"] = div_string.split()

    if div_settings.simple_geo.get():
        print("hit")
        json_dictionary["IFC"]["Simplify geometry"] = 2
    else:
        json_dictionary["IFC"]["Simplify geometry"] = 0

    json_dictionary["JSON"] = {}
    json_dictionary["JSON"]["Footprint elevation"] = float(footprint_elevation)
    json_dictionary["JSON"]["Generate interior"] = other_settings.make_interior.get()

    json_dictionary["Generate report"] = other_settings.make_report.get()

    lod_list = []
    if(lod_settings.lod00.get()):
        lod_list.append(0.0)
    if (lod_settings.lod02.get()):
        lod_list.append(0.2)
        json_dictionary["JSON"]["Generate footprint"] = footprint_settings.make_footprint.get()
        json_dictionary["JSON"]["Generate roof outline"] = footprint_settings.make_roofprint.get()
    if(lod_settings.lod03.get()):
        lod_list.append(0.3)
    if (lod_settings.lod04.get()):
        lod_list.append(0.4)
    if (lod_settings.lod10.get()):
        lod_list.append(1.0)
    if (lod_settings.lod12.get()):
        lod_list.append(1.2)
        json_dictionary["JSON"]["Footprint based"] = footprint_settings.footprint_based.get()
    if (lod_settings.lod13.get()):
        lod_list.append(1.3)
        json_dictionary["JSON"]["Footprint based"] = footprint_settings.footprint_based.get()
    if (lod_settings.lod22).get():
        lod_list.append(2.2)
        json_dictionary["JSON"]["Footprint based"] = footprint_settings.footprint_based.get()
    if (lod_settings.lod30.get()):
        lod_list.append(3.0)
    if (lod_settings.lod32.get()):
        lod_list.append(3.2)
    if (lod_settings.lod50.get()):
        lod_list.append(5.0)
    json_dictionary["LoD output"] = lod_list


    with open(json_path, "w") as outfile:
        json.dump(json_dictionary, outfile)

    # get schema of the file
    if not bool_run:
        tkinter.messagebox.showinfo("succes", "Info: Config file has been successfully created")
        return


    scheme_found = False
    for path in input_path_list:
        counter = 0
        for line in open(path):
            if "FILE_SCHEMA(('IFC2X3'))" in line or "FILE_SCHEMA (('IFC2X3'))" in line:
                code_path = findValidPath(ifc2_exe_path, "Ifc2x3")
                if not(code_path == None):
                    runExe(code_path, json_path)
                    scheme_found = True
                break
            if  "FILE_SCHEMA(('IFC4'))" in line or "FILE_SCHEMA (('IFC4'))" in line:
                code_path = findValidPath(ifc4_exe_path, "Ifc4")
                if not (code_path == None):
                    runExe(code_path, json_path)
                    scheme_found = True
                break
            if "FILE_SCHEMA(('IFC4X3'))" in line or "FILE_SCHEMA (('IFC4X3'))" in line:
                code_path = findValidPath(ifc4x3_exe_path, "Ifc4x3")
                if not (code_path == None):
                    runExe(code_path, json_path)
                    scheme_found = True
                break

            if scheme_found:
                break

            if counter == 100:
                tkinter.messagebox.showerror("File Error",
                                             "Error: Was unable to find IFC schema in file")
                return
            counter += 1
    os.remove(json_path)
    return


def findValidPath(code_path, addition):
    exe_path_root = "../Pre_Build/"

    if (os.path.isfile(os.path.abspath(code_path))):
        return code_path
    else:
        code_path = exe_path_root + code_path
        if not (os.path.isfile(os.path.abspath(code_path))):
            tkinter.messagebox.showerror("Exe Error",
                                         "Error: Unable to find suitable executable (" + addition + ")")
            return None
        return code_path


def runExe(code_path, json_path):
    try:

        stop_event = threading.Event()
        stop_event.clear()
        # Run the executable and capture its output
        env_extractor_process = subprocess.Popen(
            [
                code_path,
                json_path
            ],
        )
        def poll_process():
            while not stop_event.is_set():
                if env_extractor_process.poll() is not None:  # Check if process has finished
                    break

            if stop_event.is_set():  # If the loop exits because of `stop_event`, terminate the process
                if env_extractor_process.poll() is None:  # If process is still running
                    env_extractor_process.terminate()
                    tkinter.messagebox.showinfo("Process cancelled", "Process cancelled by user")
            else:
                if env_extractor_process.returncode == 0:
                    tkinter.messagebox.showinfo("Success", "Success: Process completed successfully")
                else:
                    tkinter.messagebox.showerror("Processing Error", "Error: Error during process")

            run_button.config(text="Run", command=lambda: runCode(
                entry_inputpath.get(),
                entry_outputpath.get(),
                lod_settings,
                footprint_settings,
                div_settings,
                other_settings,
                entry_footprint.get(),
                entry_voxelsize.get(),
                message_div_objects.get('1.0', tkinter.END),
                True
            ))
            generate_button.config(state="normal")
            close_button.config(state="normal")


        def stop_process():
            stop_event.set()

        # swap out run button
        run_button.config(text="Cancel", command=lambda:stop_process())
        generate_button.config(state="disabled")
        close_button.config(state="disabled")

        # Start the polling thread
        time.sleep(0.5)
        threading.Thread(target=poll_process, daemon=True).start()

    except subprocess.CalledProcessError as e:
        tkinter.messagebox.showerror("Processing Error",
                                     "Error: Was unable to process the file")

    return

def browse_(box, is_folder, window, initial_file):
    folder_path = ""

    if (not is_folder):
        folder_path =  filedialog.askopenfilenames(
            filetypes=[("IFC file", ".ifc")],
            defaultextension=".ifc"
        )
    else:
        # get the inital filename
        initial_file_name =  Path(initial_file).name
        folder_path = filedialog.asksaveasfilename(
            filetypes=[("JSON file", ".json"), ("CityJSON file", ".city.json")],
            defaultextension="city.json",
            initialfile=Path(initial_file_name).stem + ".json"
        )

    if len(folder_path) == 0:
        return

    box.delete(0, tkinter.END)
    box.insert(0, folder_path)
    window.focus_force()
    return

def increment(value_field, increment_value):
    try:
        float(value_field.get())
    except:
        value_field.delete(0, tkinter.END)
        value_field.insert(0, "0")

    incremented_Value = round(float(value_field.get()) + increment_value, 2)
    value_field.delete(0, tkinter.END)
    value_field.insert(0, incremented_Value)
    return

def decrement(value_field, increment_value):
    try:
        float(value_field.get())
    except:
        value_field.delete(0, tkinter.END)
        value_field.insert(0, "0")

    incremented_Value = round(float(value_field.get()) - increment_value, 2)
    value_field.delete(0, tkinter.END)
    value_field.insert(0, incremented_Value)
    return

def updateDivMessage(message_window, useDefault, useProxy):
    message_window['state'] = tkinter.NORMAL

    if useDefault.get():
        message_window.delete('1.0', tkinter.END)
        message_window.insert(tkinter.INSERT, getDefaultDivObjects() + "\t")
    else:
        message_window.delete('1.0', tkinter.END)
    if not useProxy.get():
        message_window.insert(tkinter.END, "IfcBuildingElementProxy")
    message_window['state'] = tkinter.DISABLED
    return


# main variables
size_entry_small = 13
size_button_small = 2
size_button_normal = 8

# setup the window and the grid
main_window = tkinter.Tk()
main_window.geometry('500x560')
main_window.resizable(1,0)
main_window.title("IfcEnvExtactor GUI")

# create settings classes
lod_settings = LoDSettings()
footprint_settings = FootprintSettings()
div_settings = DivSettings()
other_settings = OtherSettings()

# the entry functions for the main ifc file
text_file_browse = tkinter.Label(main_window, text="Input IFC path(s):")
text_file_browse.pack(pady=4)
frame_file_browse = tkinter.Frame(main_window)
frame_file_browse.pack(fill=tkinter.X)
entry_inputpath = tkinter.Entry(frame_file_browse, text= "Input path(s)")
entry_inputpath.pack(side=tkinter.LEFT, fill=tkinter.X, expand=True, padx=4)
button_browse = tkinter.Button(frame_file_browse, text="Browse", width=size_button_normal,
                               command= lambda: browse_(entry_inputpath, False, main_window, ""))
button_browse.pack(side=tkinter.LEFT, padx=4)

separator = ttk.Separator(main_window, orient='horizontal')
separator.pack(fill='x', pady=10)

# the entry functions for the output file
text_folder_browse = tkinter.Label(main_window, text="Output file path:")
text_folder_browse.pack()
frame_folder_browse = tkinter.Frame(main_window)
frame_folder_browse.pack(fill=tkinter.X)
entry_outputpath = tkinter.Entry(frame_folder_browse, text= "Output path")
entry_outputpath.pack(side=tkinter.LEFT, fill=tkinter.X, expand=True, padx=4)
button_browse2 = tkinter.Button(frame_folder_browse, text="Browse" , width=size_button_normal,
                                command= lambda: browse_(entry_outputpath, True, main_window, entry_inputpath.get()))
button_browse2.pack(side=tkinter.LEFT, padx=4)

separator = ttk.Separator(main_window, orient='horizontal')
separator.pack(fill='x', pady=10)

# make a frame to split the two different lod input settings
frame_lod_settings_complete = tkinter.Frame(main_window)
frame_lod_settings_complete.pack()

# the LoD levels that are desired to be output
frame_lod_settings_gen = tkinter.Frame(frame_lod_settings_complete)
frame_lod_settings_gen.pack(side=tkinter.LEFT, padx=5)

text_lod_settings = tkinter.Label(frame_lod_settings_gen, text="Desired LoD generation:")
text_lod_settings.pack()

frame_lod_settings1 = tkinter.Frame(frame_lod_settings_gen)
frame_lod_settings1.pack()
frame_lod_settings2 = tkinter.Frame(frame_lod_settings_gen)
frame_lod_settings2.pack()
frame_lod_settings3 = tkinter.Frame(frame_lod_settings_gen)
frame_lod_settings3.pack()

toggle_makelod00 = ttk.Checkbutton(frame_lod_settings1, text="LoD0.0", variable=lod_settings.lod00)
toggle_makelod02 = ttk.Checkbutton(frame_lod_settings1,
                                   text="LoD0.2",
                                   variable=lod_settings.lod02,
                                   command= lambda : [toggleEnableEntry(toggle_makefootprint, {lod_settings.lod02.get()}),
                                                      toggleEnableEntry(toggle_makeroofprint, {lod_settings.lod02.get()}),
                                                      toggleEnableEntry(toggle_makeinterior, {
                                                          lod_settings.lod02.get(),
                                                          lod_settings.lod03.get(),
                                                          lod_settings.lod12.get(),
                                                          lod_settings.lod22.get(),
                                                          lod_settings.lod32.get(),
                                                          lod_settings.lod50.get()
                                                      })]
                                   )
toggle_makelod03 = ttk.Checkbutton(frame_lod_settings1, text="LoD0.3", variable=lod_settings.lod03,
                                   command=lambda: [
                                       toggleEnableEntry(toggle_makeinterior, {
                                           lod_settings.lod02.get(),
                                           lod_settings.lod03.get(),
                                           lod_settings.lod12.get(),
                                           lod_settings.lod22.get(),
                                           lod_settings.lod32.get(),
                                           lod_settings.lod50.get()
                                       })
                                   ])
toggle_makelod04 = ttk.Checkbutton(frame_lod_settings1, text="LoD0.4", variable=lod_settings.lod04,
                                   command=lambda: [])
toggle_makelod10 = ttk.Checkbutton(frame_lod_settings2, text="LoD1.0", variable=lod_settings.lod10)
toggle_makelod12 = ttk.Checkbutton(frame_lod_settings2,
                                   text="LoD1.2",
                                   variable=lod_settings.lod12,
                                   command=lambda: [
                                       toggleEnableEntry(toggle_makeinterior, {
                                           lod_settings.lod02.get(),
                                           lod_settings.lod03.get(),
                                           lod_settings.lod12.get(),
                                           lod_settings.lod22.get(),
                                           lod_settings.lod32.get(),
                                           lod_settings.lod50.get()
                                       }),
                                       toggleEnableEntry(toggle_footprint, {
                                            lod_settings.lod12.get(),
                                            lod_settings.lod13.get(),
                                            lod_settings.lod22.get()
                                       })
                                   ])

toggle_makelod13 = ttk.Checkbutton(frame_lod_settings2, text="LoD1.3", variable=lod_settings.lod13,
                                   command=lambda: [ toggleEnableEntry(toggle_footprint, {
                                            lod_settings.lod12.get(),
                                            lod_settings.lod13.get(),
                                            lod_settings.lod22.get()
                                       })
                                   ])

toggle_makelod22 = ttk.Checkbutton(frame_lod_settings2, text="LoD2.2", variable=lod_settings.lod22,
                                   command=lambda: [
                                       toggleEnableEntry(toggle_makeinterior, {
                                           lod_settings.lod02.get(),
                                           lod_settings.lod03.get(),
                                           lod_settings.lod12.get(),
                                           lod_settings.lod22.get(),
                                           lod_settings.lod32.get(),
                                           lod_settings.lod50.get()
                                       }),
                                       toggleEnableEntry(toggle_footprint, {
                                            lod_settings.lod12.get(),
                                            lod_settings.lod13.get(),
                                            lod_settings.lod22.get()
                                       })
                                   ])
toggle_makelod30 = ttk.Checkbutton(frame_lod_settings3,
                                   text="LoD3.0",
                                   variable=lod_settings.lod30
                                   )

toggle_makelod32 = ttk.Checkbutton(frame_lod_settings3,
                                   text="LoD3.2",
                                   variable=lod_settings.lod32,
                                   command=lambda: [
                                       toggleEnableEntry(toggle_makeinterior, {
                                           lod_settings.lod02.get(),
                                           lod_settings.lod03.get(),
                                           lod_settings.lod12.get(),
                                           lod_settings.lod22.get(),
                                           lod_settings.lod32.get(),
                                           lod_settings.lod50.get()
                                       })
                                   ])

toggle_makelod50 = ttk.Checkbutton(frame_lod_settings3,
                                   text="LoDV.0",
                                   variable=lod_settings.lod50,
                                   command=lambda: [
                                       toggleEnableEntry(toggle_makeinterior, {
                                           lod_settings.lod02.get(),
                                           lod_settings.lod03.get(),
                                           lod_settings.lod12.get(),
                                           lod_settings.lod22.get(),
                                           lod_settings.lod32.get(),
                                           lod_settings.lod50.get()
                                       })
                                   ])

toggle_makelod00.pack(side=tkinter.LEFT)
toggle_makelod02.pack(side=tkinter.LEFT)
toggle_makelod03.pack(side=tkinter.LEFT)
toggle_makelod04.pack(side=tkinter.LEFT)
toggle_makelod10.pack(side=tkinter.LEFT)
toggle_makelod12.pack(side=tkinter.LEFT)
toggle_makelod13.pack(side=tkinter.LEFT)
toggle_makelod22.pack(side=tkinter.LEFT)
toggle_makelod30.pack(side=tkinter.LEFT)
toggle_makelod32.pack(side=tkinter.LEFT)
toggle_makelod50.pack(side=tkinter.LEFT)

# makeSplit
frame_lod_settings_sep = ttk.Separator(frame_lod_settings_complete, orient=tkinter.VERTICAL)
frame_lod_settings_sep.pack(side=tkinter.LEFT, expand=True)

separator_lod_settings = ttk.Separator(frame_lod_settings_sep, orient=tkinter.VERTICAL)
separator_lod_settings.pack(fill='y', pady=40, padx=5, expand=True)

# toggle if footprint or roofoutline is desired
frame_lod_settings_foot = tkinter.Frame(frame_lod_settings_complete)
frame_lod_settings_foot.pack(side=tkinter.RIGHT)

text_lod_settings = tkinter.Label(frame_lod_settings_foot, text="Additional settings")
text_lod_settings.pack()

toggle_makefootprint = ttk.Checkbutton(frame_lod_settings_foot, text="Generate footprint", variable=footprint_settings.make_footprint)
toggle_makeroofprint = ttk.Checkbutton(frame_lod_settings_foot, text="Export roof outline", variable=footprint_settings.make_roofprint)
toggle_footprint = ttk.Checkbutton(frame_lod_settings_foot, text="Footprint based abstraction", variable=footprint_settings.footprint_based)
toggle_makeinterior = ttk.Checkbutton(frame_lod_settings_foot, text="Generate interiors", variable=other_settings.make_interior)
toggle_summaryvoxel = ttk.Checkbutton(frame_lod_settings_foot, text="Approximate areas and volumes", variable=other_settings.summary_voxels)

toggle_makefootprint.pack(side=tkinter.TOP, fill=tkinter.X)
toggle_makeroofprint.pack(side=tkinter.TOP, fill=tkinter.X)
toggle_footprint.pack(side=tkinter.TOP, fill=tkinter.X)
toggle_makeinterior.pack(side=tkinter.TOP, fill=tkinter.X)
toggle_summaryvoxel.pack(side=tkinter.TOP, fill=tkinter.X)

separator2 = ttk.Separator(main_window, orient='horizontal')
separator2.pack(fill='x', pady=10)

# the voxel size that is desired
frame_foot_voxel = tkinter.Frame(main_window)
frame_foot_voxel.pack()

frame_voxel = tkinter.Frame(frame_foot_voxel)
frame_voxel.pack(side=tkinter.LEFT, padx=30)

text_voxel_settings = tkinter.Label(frame_voxel, text="Voxel size:")
text_voxel_settings.pack()

entry_voxelsize = tkinter.Entry(frame_voxel, text= "voxelsize", width=size_entry_small)
entry_voxelsize.insert(0, "1.0")
entry_voxelsize.pack(side=tkinter.LEFT)

button_min_voxelsize = tkinter.Button(frame_voxel, text="-", width=size_button_small,
                                      command= lambda : decrement(entry_voxelsize, 0.1))
button_min_voxelsize.pack(side=tkinter.LEFT, padx=(5,0))
button_plus_voxelsize = tkinter.Button(frame_voxel, text="+", width=size_button_small,
                                       command= lambda : increment(entry_voxelsize, 0.1))
button_plus_voxelsize.pack(side=tkinter.LEFT)

# the footprint height
frame_footprint = tkinter.Frame(frame_foot_voxel)
frame_footprint.pack(side=tkinter.LEFT)

text_footprint_settings = tkinter.Label(frame_footprint, text="Footprint elevation:")
text_footprint_settings.pack()

entry_footprint = tkinter.Entry(frame_footprint, text= "footprint elevation", width=size_entry_small)
entry_footprint.insert(0, "0.0")
entry_footprint.pack(side=tkinter.LEFT)

button_min_footprint = tkinter.Button(frame_footprint, text="-", width=size_button_small,
                                      command= lambda : decrement(entry_footprint, 0.01))
button_min_footprint.pack(side=tkinter.LEFT, padx=(5,0))
button_plus_footprint = tkinter.Button(frame_footprint, text="+", width=size_button_small,
                                       command= lambda : increment(entry_footprint, 0.01))
button_plus_footprint.pack(side=tkinter.LEFT)

separatorFootprint = ttk.Separator(main_window, orient='horizontal')
separatorFootprint.pack(fill='x', pady=10)

makerep_toggle = ttk.Checkbutton(main_window, text="Create report", variable=other_settings.make_report)

# The div objects
message_div_objects = tkinter.Text(main_window,  width=300, height=5, bg="#F0F0F0", fg="#707070")

frame_div_objects = tkinter.Frame(main_window)
frame_div_objects.pack()

igoreproxy_toggle = ttk.Checkbutton(frame_div_objects,
                                    text="Ignore proxy elements",
                                    variable=div_settings.ignore_proxy,
                                    command= lambda : updateDivMessage(message_div_objects,div_settings.use_default, div_settings.ignore_proxy))
igoreproxy_toggle.pack(side=tkinter.LEFT, padx=30)

useDefault_toggle = ttk.Checkbutton(frame_div_objects,
                                    text="Use default div objects",
                                    variable=div_settings.use_default,
                                    command= lambda : updateDivMessage(message_div_objects, div_settings.use_default, div_settings.ignore_proxy))
useDefault_toggle.pack(side=tkinter.LEFT)

# div communication
message_div_objects.insert(tkinter.INSERT, getDefaultDivObjects())
message_div_objects['state'] = tkinter.DISABLED
message_div_objects.pack(fill='x', padx=5, pady=10)

frame_final_objects = tkinter.Frame(main_window)
frame_final_objects.pack()

enableCustom_toggle = ttk.Checkbutton(frame_final_objects,
                                    text="Custom div objects",
                                    variable=div_settings.custom_enabled,
                                    command= lambda : toggleEnableDiv(
                                        message_div_objects,
                                        useDefault_toggle,
                                        igoreproxy_toggle,
                                        div_settings.use_default,
                                        div_settings.ignore_proxy))
enableCustom_toggle.pack(side=tkinter.LEFT)

simpleGeo_toggle = ttk.Checkbutton(frame_final_objects,
                                    text="Use simple geo",
                                    variable=div_settings.simple_geo)
simpleGeo_toggle.pack(side=tkinter.LEFT, padx=5)

# other buttons
separator3 = ttk.Separator(main_window, orient='horizontal')
separator3.pack(fill='x', pady=10)

frame_other = tkinter.Frame(main_window)
frame_other.pack(fill=tkinter.X)

run_button = tkinter.Button(frame_other, text="Run", width=size_button_normal, command=lambda: runCode(
    entry_inputpath.get(),
    entry_outputpath.get(),
    lod_settings,
    footprint_settings,
    div_settings,
    other_settings,
    entry_footprint.get(),
    entry_voxelsize.get(),
    message_div_objects.get('1.0', tkinter.END),
    True
))
run_button.pack(side=tkinter.LEFT, padx=(5,0))

generate_button = tkinter.Button(frame_other, text="Generate", width=size_button_normal, command= lambda: runCode(
    entry_inputpath.get(),
    entry_outputpath.get(),
    lod_settings,
    footprint_settings,
    div_settings,
    other_settings,
    entry_footprint.get(),
    entry_voxelsize.get(),
    message_div_objects.get('1.0', tkinter.END),
    False
))
generate_button.pack(side=tkinter.LEFT, padx=(0,0))

text_toolTip = tkinter.Label(frame_other, text="hover over settings for tooltip")
text_toolTip.pack(side=tkinter.LEFT, padx=(15,0))

close_button = tkinter.Button(frame_other, text="Close", width=size_button_normal, command= lambda : main_window.destroy())
close_button.pack(side=tkinter.RIGHT, padx=(0,5))

#tooltips
Tooltip(entry_inputpath, "Input path(s), supports multifile IFC models")
Tooltip(button_browse, "Input path(s), supports multifile IFC models")

Tooltip(entry_outputpath, "output path, application can not create new folders")
Tooltip(button_browse2, "output path, application can not create new folders")


desired_lod_tooltip_txt = "Desired LoD abstractions to be created"
Tooltip(frame_lod_settings1, desired_lod_tooltip_txt)
Tooltip(frame_lod_settings2, desired_lod_tooltip_txt)
Tooltip(frame_lod_settings3, desired_lod_tooltip_txt)

Tooltip(toggle_makefootprint, "If active a footprint will be created at the footprint elevation (lod0.2 only)")
Tooltip(toggle_makeroofprint, "If active a roof outline will be created (lod0.2 only)")
Tooltip(toggle_footprint, "If active the footprint will be used to restrict the output (LoD1.2, 1.3 & 2.2)")
Tooltip(toggle_makeinterior, "If active spaces will be stored (Lod0.2, 1.2, 2.2, 3.2 & 3.2) and storey "
                             "objects will be created (loD0.2 only)")
Tooltip(toggle_summaryvoxel, "If active volumes and areas of the shells, storey and spaces are approximated using the voxel grid")

Tooltip(entry_voxelsize, "Voxel size to be used for the analysis")
Tooltip(button_min_voxelsize, "Decrement size by 0.1")
Tooltip(button_plus_voxelsize, "increment size by 0.1")

Tooltip(entry_footprint, "Z height at which the footprint section is taken and the ground floor of the abstractions will be placed")
Tooltip(button_min_footprint, "Decrement size by 0.01")
Tooltip(button_plus_footprint, "Decrement size by 0.01")

Tooltip(igoreproxy_toggle, "If active IfcBuildingProxyElements will be excluded from the process")
Tooltip(useDefault_toggle, "If active default div objects will be used during the process")
Tooltip(message_div_objects, "Used div objects during the process, separated by a space. Can be unlocked by toggling the Custom div objects button")
Tooltip(enableCustom_toggle, "Enables the custom div objects list")
Tooltip(simpleGeo_toggle, "Use simplefied geometry for the evaluations (voids are ignored)")

Tooltip(run_button, "Run the tool")
Tooltip(generate_button, "Generate and store the Configuration JSON")
Tooltip(close_button, "Exit the application")

main_window.mainloop()