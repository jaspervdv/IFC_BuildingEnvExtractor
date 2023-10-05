# IfcEnvelopeExtractor

The IfcEnvelopeExtractor enables users to automatically extract the building shell of an IFC-model and convert it to a CityJSON model. Automating this process allows designs to be quickly and easily analyzed on a city scale without the need for lengthy manual conversions. This is one of the steps required to close the gap between architecture/BIM and city scale models.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample.gif "An example of the created LoD envelopes based on an input file")

The software is able to extract multiple different LoD (Level of Detail) shells from an IFC-model. The actual LoDs it is able to extract is dependent on the accuracy and validity of the input model. The tool utilizes three different extraction methods that can be used on progressively more accurate models. Lower detail shells (LoD 0.0 & 1.0) can be extracted only based on the vertices present in a model. Middle level detail shells (Lod 0.2 w/o , 1.2, 1.3, 2.2) can be extracted based on the model’s roofing structures. High level detail shells (Lod 0.2 w/ footprint and 3.2) can be extracted based on the model’s objects that are part of the building envelope. This final extraction step only functions on well-constructed models, but yields an accurate result that allows for overhang and underpasses. These features are often only present in models that are made manually.

Below you can see a speed comparison between the software and manual processing. Note that the software creates the shells for LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 while the manual processing only creates LoD 2.2. This example is sped up 5 times.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample2.gif "Speed comparison of the software making LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 vs making LoD 2.2 by hand.")

This program is part of the [CHECK project](https://chekdbp.eu/). Any suggestions, additions or changes that are made by other users will also be utilized by this project.  

## Envelope generation

### Lower detail envelopes

The lower level LoD shells are created based on a subset of vertices of the model. This subset is collected from the IfcWall, IfcWallStandardCase, IfcRoof, IfcSlab and IfcWindow objects. A set of incrementally rotated bounding boxes is created to encapsulate these vertices. The bounding box with the smallest area is converted to the LoD 1.0 envelope. The ground surface of the bounding box is isolated and converted to the LoD 0.0 envelope.

### Middle level shell

Middle level detail shell (Lod 0.2, 1.2, 1.3, 2.2) can be extracted primarily based on the model’s roofing structures. These roofing structures are isolated in two steps. The first step is a coarse isolation which is done via column voxelization. The second step is a filtering of the isolated objects via a column like ray casting process. This will result in a collection of roofing surfaces.

To construct the LoD 0.2 and 1.2 shell these surfaces (And their vertices) are all projected to the xy plane and merged in one single surface. This surface can be converted to the LoD 0.2 shell. To get the LoD 1.2 shell this surface is extrude in a positive z-direction to the max height of the original model.

To construct the LoD 1.3 shell all the filtered roofing surfaces are also projected to the xy plane, but not merged. Instead every surface is translated back in the z-direction to the z-max of their original surface shape. These translated shapes are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 1.3 shell.

To construct the LoD 2.2 shell all the filtered roofing surfaces are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 2.2 shell.

### High level shells

The high level shells are extracted via a voxelizations system that is refined by ray casting. The voxilization process functions is inspired by the room growing process described by [Vaart et al., (2022)](#1). The starting point is a voxel that is outside of the building. Every object that intersects with the growing shape is stored. The surfaces of each found object are then filtered with the help of a ray casting system which casts rays from points on each surface to the center of the exterior voxels. The found objects are merged into the LoD3.2 shell

### voxel shells (WIP)

Additionally the tool will also be able to export a voxel shell based on the input model. This will be stored as LoD5.0. LoD5.0 has only been partially implemented in the current tool and does not yet output desired results.

## Input file requirements

The algorithms in this tool have been developed to work with models created by people who may not be BIM experts. As a result, the tool has been developed to accept unconventional input files. However, To enlarge the chances of success, there are currently some requirements that should be followed whenever possible. The list with requirements is divided over the three different levels of processing that were mentioned before. The requirements per level build on top of the ones of lower levels. So for middle level shells the mentioned requirements + the lower detail requirements are recommended. The requirements within one level are listed in order of importance.

Lower detail shells (LoD 0.0 & 1.0):

* Valid IFC4 or IFC2x3 file
* Valid units
* Correctly classified objects* (recommended)
* No or limited use of IfcBuildingElementProxy objects (recommended)
* Site and building structurally separated (recommended)

Middle level shells (Lod 0.2, 1.2, 1.3, 2.2)

* Correctly modelled and watertight roofing structure
* No or limited use of triangulated objects (recommended)

High level envelopes (LoD 3.2)

* Correctly modelled and watertight building exterior

Performance of the tool is depending on the complexity of the input model. The tool has proven to function well on simple models in the current state but can struggle on more complex shapes.

*Default behavior of the tool is using the room bounding objects presented by [Vaart et al., (2022)](#1) as space dividing objects. These are IfcWall, IfcCurtainWall, IfcWallStandardCase, IfcRoof, IfcSlab, IfcWindow, IfcColumn, IfcBeam, IfcDoor, IfcCovering, IfcMember and IfcPlate. If default settings are used it is recommended to have these object correctly classified in the file. Additionally added types would require the additional type to be correctly classified as well.

## How to use

The tool can be used directly with the executables located in the Pre_Build folder. Make sure that you run the correct executable for the IFC version of the model. An IFC4 file will not be processed by the IFC2x3 version of the tool. When the .exe is started it will ask the required questions for processing. This can also be bypassed by inputting a JSON settings file instead of an IFC file. It is also possible to start the tool from the console or call it with other software directly. The only input allowed when doing this is a file path to the IFC file or the settings JSON.

## How to Build

If it is desired to compile the code locally the following libraries are required:

* [IfcOpenShell](http://ifcopenshell.org/)
* [Nlohmann](https://github.com/nlohmann/json)
* [CJT](https://github.com/jaspervdv/CJT)
* [Boost](https://www.boost.org/)
* [OpenCASCADE](https://dev.opencascade.org/)

In main.cpp there is a vector called "sourcePathArray". In this vector filepaths are hard coded, this is mostly for quick testing. If this array is not empty the tool will directly process the files at the filepaths found in this array at executable start. If the array is emptied the program will allow the user to submit one, or multiple filepaths when the executable is started.

In helper.h the first line is "#define USE_IFC4". keeping this line will build an executable that is able to process IFC4. If the line is commented out or removed the build executable will be able to process IFC2x3 files.`

Please note that CJT is developed in tandem with the IFcEnvExtractor. So possible version mismatches may occur due to CJT being updated later than the IFcEnvExtractor.

## GUI

IfcEnvelopeExtractor can now also be operated via an GUI. This GUI enables the user to easily change the settings. The goal of the GUI is making the tool more accessible for non-expert users. The GUI is a python based front that automatically generates the Settings JSON and calls the suitable extractor executable.

The GUI can be accessed via two routes:

* In the Pre_Build folder the Ext_GUI.exe will start the GUI. Make sure this .exe is always in the same folder as the Extractor.exe files.
* In the GUI_code the the python file will also start the GUI. This would require the user to have python installed.

<p align="center" width="100%">
    <img src="https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/GUI_example.JPG" alt= “” width="300">
</p>
Currently the GUI is still in an initial development state and does not expose all the settings that are available. Additionally it does also not supply the user with clear error messages if the tool fails.

## Settings JSON

The settings json has a very simple structure. An example can be found below:

```json
{
    "Filepaths": {
        "Input" : 
        [
            "path to IFC file",
            "Potential path to other IFC file"
        ],
        "Output" : "path to export folder"
    },
    "voxelSize" : {
        "xy" : 1
    },
    "Footprint elevation" : 0.15,
    "Output report" : 1,
    "LoD output" : [0.0, 0.2, 1.0, 1.2, 1.3, 2.2, 3.2],
    "Default div": 1,
    "Ignore proxy" : 1,
    "Div objects": ["IfcFlowTerminal"]
}
```

The json has mandatory and optional inputs. If the mandatory inputs are missing the process will not execute properly. If optional inputs are missing a default value will be used.

Mandatory:

* The "Filepaths""Input" is an array including 1 to ∞ input paths representing all the files constructing a single building.
* The "Filepaths""Output" is a string representing the output **folder** not the desired file. The tool will automatically generate a file name based on the input file.

Optional:

* "voxelSize""xy" is a double that will be used for the extraction process. A value between 0.5 and 1 often suffices for normal buildings. Default value = 0.5.
* "Footprint elevation" is a double that will be the level at which a horizontal section will be taken of the building. This section is used to create the footprint from. Default value = 0.
* "Output report" is an int/bool (either 0 or 1) to set the tool to output a report file or not. 0 = no, 1 = yes. Default value = true.
* "LoD output" is an array including the desired LoD output. The options are 0.0, 0.2, 1.0, 1.2, 1.3, 2.2, 3.2 and 5.0 (for a voxel shape). Default value: 0.0, 0.2, 1.0, 1.2, 1.3, 2.2, 3.2.
* "Default div" is an int/bool (either 0 or 1) to set if the default space bounding objects are used. Default value = true.
* "Ignore proxy" is an int/bool (either 0 or 1) tell the tool to use IfcBuildingElementProxy as a space dividing object class. 0 = no, 1 = yes. Default value: yes
* "Div objects" is an array including the additional space dividing objects (the Ifc types). Default value = emptry

more options will be added in the future

## References

<a id="1"></a> 
van der Vaart, J. A. J. (2022, June 6). Automatic Building Feature Detection and Reconstruction in IFC models. TU Delft Repositories. Retrieved March 30, 2023, from https://repository.tudelft.nl/islandora/object/uuid:db6edbfc-5310-47db-b2c7-3d8e2b62de0f 