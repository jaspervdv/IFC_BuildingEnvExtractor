# IfcEnvelopeExtractor

The IfcEnvelopeExtractor enables users to automatically extract the building shell of an IFC-model and convert it to a CityJSON model. Automating this process allows designs to be quickly and easily analyzed on a city scale without the need for lengthy manual conversions. This is one of the steps required to close the gap between architecture/BIM and city scale models.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample.gif "An example of the created LoD envelopes based on an input file")

The software is able to extract multiple different LoD (Level of Detail) shells from an IFC-model. The actual LoDs it is able to extract is dependent on the accuracy and validity of the input model.

Current possible output shells:

* Lod0.0 (exterior only)
* Lod0.2 (exterior roof outline, footprint & interior storeys) (WIP)
* LoD1.0 (exterior only)
* LoD1.2 (exterior only)
* LoD1.3 (exterior only)
* LoD2.2 (exterior only)
* LoD3.2 (exterior only) (WIP)
* LoD5.0 (exterior only)

For the extraction of the exterior shell the tool utilizes three different extraction methods that can be used on progressively more accurate models. Lower detail shells (LoD 0.0 & 1.0) can be extracted only based on the vertices present in a model. Middle level detail shells (Lod 0.2 w/o footprint, 1.2, 1.3, 2.2) can be extracted based on the model’s roofing structures. High level detail shells (Lod 0.2 w/ footprint and 3.2) can be extracted based on the model’s objects that are part of the building envelope. This final extraction step only functions on well-constructed models, but yields an accurate result that allows for overhang and underpasses. These features are often only present in models that are made manually.

The extraction of interior shells relies more heavily on the source IFC file and does not scale like the exterior shells. For all interior export (Storeys, apartments/areas and rooms/spaces) a high quality input model is required with additional extra "special" semantic information to help the tool function.

Below you can see a speed comparison between the software and manual processing of the exterior shell. Note that in this comparison the software creates the exterior shells for LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 while the manual processing only creates LoD 2.2. This example is sped up 5 times.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample2.gif "Speed comparison of the software making LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 vs making LoD 2.2 by hand.")

This program is part of the [CHECK project](https://chekdbp.eu/). Any suggestions, additions or changes that are made by other users could also be utilized by this project.  

## Table of Content

* [How to Execute](#how-to-execute)
* [How to Build](#how-to-build)
* [GUI](#gui)
* [Input file requirements](#input-file-requirements)
* [Output file structure](#output-file-structure)
* [Outer shell generation methods](#outer-shell-generation-methods)
  * [Lower detail shells](#lower-detail-shells)
  * [Middle level shells](#middle-level-shells)
  * [High level shells](#high-level-shells)
  * [Voxel shells](#voxel-shells-wip)
* [Inner shell generation methods](#inner-shell-generation-methods-experimentalwip)
  * [Storey extraction](#storey-extraction)
  * [Room/space extraction](#roomspace-extraction)
  * [Apartment/area extraction](#apartmentarea-extraction)
* [Settings JSON](#settings-json)
* [References](#references)

## How to Execute

The tool can be used directly with the executables located in the Pre_Build folder. The folder should contain three executables:

* Ifc_Envelope_Extractor_ifc2x3.exe
* Ifc_Envelope_Extractor_ifc4.exe
* Ext_GUI.exe

The _Ifc_Envelope_Extractor_ifc2x3.exe_ or _Ifc_Envelope_Extractor_ifc4.exe_ can be called with a path to a settings JSON file. This file is used to give the tool the needed information related to the IFC model ([more info](#settings-json)). This enables the .exe to be easily called by other applications. It is important to make sure that the correct executable for the IFC version of the model is used. An IFC4 file will not be processed by the IFC2x3 version of the tool.

If a more direct (human) user friendly approach is desired both the aforementioned .exe can be called with the help of the _Ext_GUI.exe_ ([more info](#gui)).

## How to Build

If it is desired to compile the code locally the following libraries are required:

* [IfcOpenShell](http://ifcopenshell.org/)
* [Nlohmann](https://github.com/nlohmann/json)
* [CJT](https://github.com/jaspervdv/CJT)
* [Boost](https://www.boost.org/)
* [OpenCASCADE](https://dev.opencascade.org/)

To set the IFC version of the tool a single line has to be commented out (or in). In _helper.h_ the first line is "#define USE_IFC4". keeping this line will build an executable that is able to process IFC4. If the line is commented out or removed the build executable will be able to process IFC2x3 files.`

Please note that CJT is developed in tandem with the IFcEnvExtractor. So possible version mismatches may occur due to CJT being updated later than the IFcEnvExtractor.

## GUI

IfcEnvelopeExtractor can now also be operated via an GUI. This GUI enables the user to easily change the settings. The goal of the GUI is making the tool more accessible for non-expert users. The GUI is a python based front that automatically generates the Settings JSON and calls the suitable extractor executable.

The GUI can be accessed via two routes:

* In the Pre_Build folder the _Ext_GUI.exe_ will start the GUI. Make sure this .exe is always in the same folder as the Extractor.exe files.
* In the GUI_code folder the python file will also start the GUI. This would require the user to have python installed on their system.

<p align="center" width="100%">
    <img src="https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/GUI_example.JPG" alt= “” width="300">
</p>

## Input file requirements

The algorithms in this tool have been developed to work with models created by people who may not be BIM experts. As a result, the tool can accept somewhat unconventional input files. However, To enlarge the chances of success, there are currently some requirements that should be followed whenever possible. The list with requirements for exterior extraction is divided over the three different complexity levels ([more info](#outer-shell-generation-methods)). The requirements per level build on top of the ones of lower levels. This means that for middle level shell extraction, the middle detail requirements + the lower detail requirements are recommended. The requirements within one level are listed in order of importance.

These levels only apply to LoD models that are build WITHOUT interiors. If interior generation is enabled the requirements from all levels (low, middle and high) and the additional interior shell requirements have to be met.

Lower detail shells (LoD 0.0, 1.0 & voxel shells):

* Valid IFC4 or IFC2x3 file
* Valid units
* Correctly classified objects* (recommended)
* No or limited use of _IfcBuildingElementProxy_ objects (recommended)
* Site and building structurally separated (recommended)

Middle level shells (Lod 0.2, 1.2, 1.3, 2.2):

* Correctly modelled and watertight roofing structure
* No or limited use of triangulated objects (recommended)

High level envelopes (LoD 3.2):

* Correctly modelled and watertight building exterior

Envelopes with interior data:

* Correct _IfcBuildingStorey_ use for storey creation
* Correctly grouped objects in their respective storeys. Recommended to use NO objects that span multiple storeys.
* Correct IfcSpace use for space and apartment creation (both semantically and geometrically)**

Performance of the tool is depending on the complexity of the input model. The tool has proven to function well on simple models in the current state but can struggle on more complex shapes.

*Default behavior of the tool is using the room bounding objects presented by [Vaart et al., (2022)](#1) as space dividing objects. These objects are _IfcWall, IfcCurtainWall, IfcWallStandardCase, IfcRoof, IfcSlab, IfcWindow, IfcColumn, IfcBeam, IfcDoor, IfcCovering, IfcMember_ and _IfcPlate_. If default settings are used it is recommended to have these object correctly classified in the file. Additionally added types would require the additional type to be correctly classified as well.

**Note that software like Autodesk Revit natively has some issues that can prevent it from doing this properly,

## Output file structure

The output files are CityJSON format compliant but might follow a different structure than is usually encountered. Every object that represents a building is split up in 2 building parts: The "Outer Shell" and the "Inner Shell". The "Outer Shell" follows the regular structure that is often seen in established city scale models. It is populated with the different exterior LoD shells. The "Inner Shell" is, depending on the process settings and input file quality, split further over storeys, apartments and rooms.

Below the structure of the file can be seen.

```bash
CityJSON File
└── Building Object*
    ├── Outer Shell
    └── Inner Shell*
        └── Storey**
            └── Apartment/Area* **
                └── Space/Room**
```

*Object that does not directly store any geometry.

**Depending on the quality of the input, the nature of the input and the user settings these objects can occur many times.

## Outer shell generation methods

### Lower detail shells

The lower level LoD shells are created based on a subset of vertices of the model. This subset is collected from the _IfcWall, IfcWallStandardCase, IfcRoof, IfcSlab_ and _IfcWindow_ objects. A set of incrementally rotated bounding boxes is created to encapsulate these vertices. The bounding box with the smallest area is converted to the LoD 1.0 envelope. The ground surface of the bounding box is isolated and converted to the LoD 0.0 envelope.

### Middle level shells

Middle level detail shell (Lod 0.2  w/o footprint, 1.2, 1.3, 2.2) can be extracted primarily based on the model’s roofing structures. These roofing structures are isolated in two steps. The first step is a coarse isolation which is done via column voxelization. The second step is a filtering of the isolated objects via a column-like ray casting process. This will result in a collection of roofing surfaces.

To construct the LoD 0.2 and 1.2 shell these surfaces (And their vertices) are all projected to the xy plane and merged in one single surface. This surface can be converted to the LoD 0.2 shell. To get the LoD 1.2 shell this surface is extruded in a positive z-direction to the max z-height of the original model.

To construct the LoD 1.3 shell all the filtered roofing surfaces are also projected to the xy plane, but not merged. Instead every surface is translated back in the z-direction to the z-max of their original surface shape. These translated shapes are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 1.3 shell.

To construct the LoD 2.2 shell all the filtered roofing surfaces are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 2.2 shell.

### High level shells (WIP)

The high level shells (Lod 0.2 w/ footprint and 3.2) are extracted via a voxelization system that is refined by ray casting. The voxelization process is inspired by the room growing process described by [Vaart et al., (2022)](#1). The starting point is a voxel that is outside of the building. Every object that intersects with the growing shape is stored. The surfaces of each found object are then filtered with the help of a ray casting system which casts rays from points on each surface to the center of the exterior voxels. The found objects are merged into the LoD3.2 shell when done in 3D. This process can also be executed in a 2D section at the ground floor level to create the footprints.

At this moment in time this extraction method is slow and does not return solids.

### voxel shells

Additionally the tool will also be able to export a shell based on the voxelgrid that is used in the other processes. This will be stored as LoD5.0. Note that this is a voxelgrid that is used primairily for filtering and raycasting purposes. Possibly this will follow different rules than are anticipated.

## Inner shell generation Methods (Experimental/WIP)

**THIS EXTRACTION IS NOT YET PROPERLY IMPLEMENTED; THIS SECTION WILL NOT BE ACCURATE**.

Inner shell creation relies heavily on both geometric and semantic data in an IFC file. This means that for (somewhat) reliable interior extraction.

### Storey extraction

For storey extraction the elevations and related objects of _IfcBuildingStorey_ objects are used. The related objects are split horizontally 5 cm below the storey's stored elevation. The shapes that are created are merged into the horizontal sections representing the storeys.

For simple buildings the related object detection can be disabled. The tool will now evaluate all the objects in the model, and not only the related objects to the _IfcBuildingStorey_. This improves the extraction on simple buildings that have modelling inaccuracies.

For more complex buildings a bool can be added related to the _IfcBuildingStorey_ object that signifies if the tool should evaluate it or not. This could be useful when the model includes _IfcBuildingStorey_ objects that are used in a non-standard way

### Room/space extraction

Method still in development

### Apartment/area extraction

Method still in development

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
        "Output" : "path to export (City)JSON file"
    },
    "voxelSize" : {
        "xy" : 1
    },
    "Footprint elevation": -0.15,
    "Output report": 1,
    "Default div": 1,
    "Ignore Proxy": 1,
    "Generate interior": 1,
    "Generate footprint": 1,
    "Generate roof outline": 1,
    "LoD output": [
        0.0,
        0.2,
        1.0,
        1.2,
        1.3,
        2.2
    ]
}
```

The json has mandatory and optional inputs. If the mandatory inputs are missing the process will not execute properly. If optional inputs are missing a default value will be used.

Mandatory:

* The "Filepaths" "Input" is an array including 1 to ∞ input paths representing all the files constructing a single building.
* The "Filepaths" "Output" is a string representing the output filepath.

Optional:

* "voxelSize" "xy" is a double that will be used for the extraction process. A value between 0.5 and 1 often suffices for normal buildings. Default value = 0.5.
* "Footprint elevation" is a double that will be the level at which a horizontal section will be taken of the building. This section is used to create the footprint from. Default value = 0.
* "Output report" is an int/bool (either 0 or 1) to set the tool to output a report file or not. 0 = no, 1 = yes. Default value = true.
* "Default div" is an int/bool (either 0 or 1) to set if the default space bounding objects are used. Default value = true.
* "Ignore proxy" is an int/bool (either 0 or 1) to tell the tool to use IfcBuildingElementProxy as a space dividing object class. 0 = no, 1 = yes. Default value: yes
* "Div objects" is an array including the additional space dividing objects (the Ifc types). Default value = empty
* "Generate interior" is an int/bool (ether 0 or 1) to enable interior shapes to be stored to the exported file.
* "Generate footprint" is an int/bool (ether 0 or 1) to enable the footprint export for LoD0.2. If off the roof outline will be placed at footprint level
* "Generate roof outline" is an int/bool (ether 0 or 1) to enable the roof outline export for LoD0.2
* "LoD output" is an array including the desired LoD output. The options are 0.0, 0.2, 1.0, 1.2, 1.3, 2.2, 3.2 and 5.0 (for a voxel shape). Default value: 0.0, 0.2, 1.0, 1.2, 1.3, 2.2, 3.2.

more options will be added in the future.

## References

<a id="1"></a> 
van der Vaart, J. A. J. (2022, June 6). Automatic Building Feature Detection and Reconstruction in IFC models. TU Delft Repositories. Retrieved March 30, 2023, from https://repository.tudelft.nl/islandora/object/uuid:db6edbfc-5310-47db-b2c7-3d8e2b62de0f 