import os
import tkinter
from tkinter import ttk, filedialog, messagebox
import json
import subprocess

def runCode(input_path,
            output_path,
            make_lod00,
            make_lod02,
            make_lod10,
            make_lod12,
            make_lod13,
            make_lod22,
            make_lod32,
            bool_igoreproxy,
            make_report,
            footprint_elevation,
            voxel_size):

    json_path = "../Pre_Build/~temp.json"
    ifc4_exe_path = "../Pre_Build/Ifc_Envelope_Extractor_ifc4.exe"
    ifc2_exe_path = "../Pre_Build/Ifc_Envelope_Extractor_ifc2x3.exe"

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
    if(not make_lod00 and not make_lod02 and not make_lod10 and not
        make_lod12 and not make_lod13 and not make_lod22 and not make_lod32):
        tkinter.messagebox.showerror("Settings Error",  "Error: no LoD output selected")
        return

    # check paths
    if(not os.path.isfile(input_path)):
        tkinter.messagebox.showerror("Settings Error",  "Error: No Valid input file supplied")
        return

    if(not os.path.isdir(output_path) and not len(output_path) == 0):
        tkinter.messagebox.showerror("Settings Error", "Error: No Valid output folder supplied\n (GUI can not create new folders)")
        return

    # generate output path
    if(len(output_path) == 0):
        output_path = os.path.dirname(input_path) + "/exports"

    print(os.path.dirname(input_path))

    # write data to json
    json_dictionary = {}
    json_dictionary["Filepaths"] = {
        "Input" : [input_path],
        "Output" : output_path
    }
    json_dictionary["voxelSize"] = float(voxel_size)
    json_dictionary["Footprint elevation"] = float(footprint_elevation)
    json_dictionary["Output report"] = make_report
    json_dictionary["Ignore Proxy"]= bool_igoreproxy

    lod_list = []

    if(make_lod00):
        lod_list.append(0.0)
    if (make_lod02):
        lod_list.append(0.2)
    if (make_lod10):
        lod_list.append(1.0)
    if (make_lod12):
        lod_list.append(1.2)
    if (make_lod13):
        lod_list.append(1.3)
    if (make_lod00):
        lod_list.append(2.2)
    if (make_lod32):
        lod_list.append(3.2)

    json_dictionary["LoD output"] = lod_list

    with open(json_path, "w") as outfile:
        json.dump(json_dictionary, outfile)

    # get schema of the file
    counter  = 0
    is_ifc4 = True

    for line in open(input_path):
        if "FILE_SCHEMA(('IFC2X3'))" in line:
            is_ifc4 = False
            break

        if  "FILE_SCHEMA(('IFC4'))" in line:
            break

        if counter == 100:
            tkinter.messagebox.showerror("File Error",
                                         "Error: Was unable to find IFC schema in file")
            return
        counter += 1


    # call ifc exe
    if is_ifc4:
        runExe(ifc4_exe_path, json_path)
    else:
        runExe(ifc2_exe_path, json_path)

    os.remove(json_path)

    return

def browse_(box, is_folder, window):
    folder_path = ""

    if (not is_folder):
        folder_path =  filedialog.askopenfilename()
    else:
        folder_path = filedialog.askdirectory()

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


def runExe(code_path, json_path):
    try:
        # Run the executable and capture its output
        result = subprocess.Popen(
            [
                code_path,
                json_path
            ],
            # stdout=subprocess.DEVNULL,
            # stderr=subprocess.DEVNULL
        )
        while result.poll() is None:
            continue
            # do something

        if result.returncode == 0:
            tkinter.messagebox.showerror("Succes",
                                         "Succes: Succes")
        else:
            tkinter.messagebox.showerror("Processing Error",
                                         "Error: Error during process")
            return

    except subprocess.CalledProcessError as e:
        tkinter.messagebox.showerror("Processing Error",
                                     "Error: Was unable to process the file")
        return


# setup the window and the grid
main_window = tkinter.Tk()
main_window.geometry('500x470')
main_window.resizable(1,0)
main_window.title("IfcEnvExtactor GUI")

# the entry functions for the main ifc file
text_file_browse = tkinter.Label(main_window, text="Input IFC path:")
text_file_browse.pack()
frame_file_browse = tkinter.Frame(main_window)
frame_file_browse.pack(fill=tkinter.X)
entry_inputpath = tkinter.Entry(frame_file_browse, text= "Input path")
entry_inputpath.pack(side=tkinter.LEFT, fill=tkinter.X, expand=True, padx=4)
button_browse = tkinter.Button(frame_file_browse, text="Browse", command= lambda: browse_(entry_inputpath, False, main_window))
button_browse.pack(side=tkinter.LEFT, padx=4)

separator = ttk.Separator(main_window, orient='horizontal')
separator.pack(fill='x', pady=10)

# the entry functions for the output file
text_folder_browse = tkinter.Label(main_window, text="Output folder path:")
text_folder_browse.pack()
frame_folder_browse = tkinter.Frame(main_window)
frame_folder_browse.pack(fill=tkinter.X)
entry_outputpath = tkinter.Entry(frame_folder_browse, text= "Output path")
entry_outputpath.pack(side=tkinter.LEFT, fill=tkinter.X, expand=True, padx=4)
button_browse2 = tkinter.Button(frame_folder_browse, text="Browse", command= lambda: browse_(entry_outputpath, True, main_window))
button_browse2.pack(side=tkinter.LEFT, padx=4)

separator = ttk.Separator(main_window, orient='horizontal')
separator.pack(fill='x', pady=10)

# the LoD levels that are desired to be output
text_lod_settings = tkinter.Label(main_window, text="Desired LoD generation:")
text_lod_settings.pack()

frame_lod_settings1 = tkinter.Frame(main_window)
frame_lod_settings1.pack()
frame_lod_settings2 = tkinter.Frame(main_window)
frame_lod_settings2.pack()
frame_lod_settings3 = tkinter.Frame(main_window)
frame_lod_settings3.pack()

bool_lod00 = tkinter.IntVar(value=1)
toggle_makelod00 = ttk.Checkbutton(frame_lod_settings1, text="LoD0.0", variable=bool_lod00)
bool_lod02 = tkinter.IntVar(value=1)
toggle_makelod02 = ttk.Checkbutton(frame_lod_settings1, text="LoD0.2", variable=bool_lod02)
bool_lod10 = tkinter.IntVar(value=1)
toggle_makelod10 = ttk.Checkbutton(frame_lod_settings1, text="LoD1.0", variable=bool_lod10)
bool_lod12 = tkinter.IntVar(value=1)
toggle_makelod12 = ttk.Checkbutton(frame_lod_settings2, text="LoD1.2", variable=bool_lod12)
bool_lod13 = tkinter.IntVar(value=1)
toggle_makelod13 = ttk.Checkbutton(frame_lod_settings2, text="LoD1.3", variable=bool_lod13)
bool_lod22 = tkinter.IntVar(value=1)
toggle_makelod22 = ttk.Checkbutton(frame_lod_settings2, text="LoD2.2", variable=bool_lod22)
bool_lod32 = tkinter.IntVar(value=0)
toggle_makelod32 = ttk.Checkbutton(frame_lod_settings3, text="LoD3.2", variable=bool_lod32)

toggle_makelod00.pack(side=tkinter.LEFT)
toggle_makelod02.pack(side=tkinter.LEFT)
toggle_makelod10.pack(side=tkinter.LEFT)
toggle_makelod12.pack(side=tkinter.LEFT)
toggle_makelod13.pack(side=tkinter.LEFT)
toggle_makelod22.pack(side=tkinter.LEFT)
toggle_makelod32.pack(side=tkinter.LEFT)

separator2 = ttk.Separator(main_window, orient='horizontal')
separator2.pack(fill='x', pady=10)

# the voxel size that is desired
text_voxel_settings = tkinter.Label(main_window, text="Voxel size:")
text_voxel_settings.pack()

frame_voxel = tkinter.Frame(main_window)
frame_voxel.pack()

entry_voxelsize = tkinter.Entry(frame_voxel, text= "voxelsize")
entry_voxelsize.insert(0, "1.0")
entry_voxelsize.pack(side=tkinter.LEFT)

button_min_voxelsize = tkinter.Button(frame_voxel, text="-", command= lambda : decrement(entry_voxelsize, 0.1))
button_min_voxelsize.pack(side=tkinter.LEFT)
button_plus_voxelsize = tkinter.Button(frame_voxel, text="+", command= lambda : increment(entry_voxelsize, 0.1))
button_plus_voxelsize.pack(side=tkinter.LEFT)

separatorVoxel = ttk.Separator(main_window, orient='horizontal')
separatorVoxel.pack(fill='x', pady=10)

# the footprint height
text_footprint_settings = tkinter.Label(main_window, text="footprint elevation:")
text_footprint_settings.pack()

frame_footprint = tkinter.Frame(main_window)
frame_footprint.pack()

entry_footprint = tkinter.Entry(frame_footprint, text= "footprint elevation")
entry_footprint.insert(0, "-0.15")
entry_footprint.pack(side=tkinter.LEFT)

button_min_footprint = tkinter.Button(frame_footprint, text="-", command= lambda : decrement(entry_footprint, 0.01))
button_min_footprint.pack(side=tkinter.LEFT)
button_plus_footprint = tkinter.Button(frame_footprint, text="+", command= lambda : increment(entry_footprint, 0.01))
button_plus_footprint.pack(side=tkinter.LEFT)

separatorFootprint = ttk.Separator(main_window, orient='horizontal')
separatorFootprint.pack(fill='x', pady=10)

bool_makerep = tkinter.IntVar(value=1)
makerep_toggle = ttk.Checkbutton(main_window, text="Create report", variable=bool_makerep)

# The div objects
bool_igoreproxy = tkinter.IntVar(value=1)
igoreproxy_toggle = ttk.Checkbutton(main_window, text="Ignore proxy elements", variable=bool_igoreproxy)
igoreproxy_toggle.pack()

# other buttons
separator3 = ttk.Separator(main_window, orient='horizontal')
separator3.pack(fill='x', pady=10)

frame_other = tkinter.Frame(main_window)
frame_other.pack()
run_button = tkinter.Button(frame_other, text="Run", command=lambda: runCode(
    entry_inputpath.get(),
    entry_outputpath.get(),
    bool_lod00.get(),
    bool_lod02.get(),
    bool_lod10.get(),
    bool_lod12.get(),
    bool_lod13.get(),
    bool_lod22.get(),
    bool_lod32.get(),
    bool_makerep.get(),
    bool_igoreproxy.get(),
    entry_footprint.get(),
    entry_voxelsize.get()
))
run_button.pack(side=tkinter.LEFT)

close_button = tkinter.Button(frame_other, text="Close", command= lambda : main_window.destroy())
close_button.pack(side=tkinter.LEFT)

main_window.mainloop()