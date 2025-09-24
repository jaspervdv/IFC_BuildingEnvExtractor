# IfcEnvelopeExtractor

<!-- markdownlint-disable MD033 -->
<!-- markdownlint-disable MD034 -->

The IfcEnvelopeExtractor enables users to automatically abstract BIM models (IFC) to format compliant GIS models (CityJSON, STEP, and OBJ). Automating this process allows designs to be quickly and easily place the building BIM model into a GIS environment without the need of a lengthy manual conversion or without the need of surveying data. This can be used to use GIS analysis methods without compromise but also to update GIS environment models faster and with more detail (such as overhang and facade/roof openings). The converter adheres geometrically to a selection of LoD from the LoD framework developed by the TU Delft [(Biljecki et al., 2016)](#2). The software can create CityJSON models with overhang (LoD3/3.2) and interior spaces and/or storeys (LoD0.2, 0.3, 1.2, 2.2, 3.2). The capabilities of the application has been expanded by experimental LoD that fall outside of this framework that utilize the data of BIM.

The development of this conversion process is one of the steps required to close the gap between architecture/BIM and  GIS/city scale models.

**Development of this tool is funded up until the end of December 2025. After this date new funding has to be found if further development is desired.**

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample.gif "An example of the created LoD envelopes based on an input file")

The software is able to extract multiple different LoD (Level of Detail) shells from an IFC-model. The actual LoDs it is able to extract is dependent on the accuracy and validity of the input model.

Current possible output shells (*non-standard LoD in Italic type*):

* Lod0.0 - exterior only
* Lod0.2 - exterior roof outline, footprint, interior stories and rooms
* LoD0.3 - exterior roof structure and interior stories
* *LoD0.4 - exterior roof structure (LoDa.0, roof structure)*
* LoD1.0 - exterior only
* LoD1.2 - exterior and rooms
* LoD1.3 - exterior only
* LoD2.2 - exterior and rooms
* LoD3.2 - exterior and rooms (WIP)
* *LoD5.0 - exterior and interior rooms (voxelized representation)*
* *LoDb.0 - exterior only (footprint extruded upwards to roof structure but retaining overhang from roofs)*
* *LoDc.1 - exterior only (extruded LoD0.2 surfaces)*
* *LoDc.2 - exterior only (WIP) (extruded LoD0.2 surfaces with LoD0.4 roofs)*
* *LoDd.1 - exterior only (WIP) (extruded LoD0.3 internal surfaces)*
* *LoDd.2 - exterior only (planned) (extruded LoD0.3 internal surfaces with LoD0.4 roofs)*
* *LoDe.0 - all relevant geometry (WIP) (full 1:1 conversion of geometry)*
* *LoDe.1 - exterior unrefined LoD3.2*

Current supported IFC versions:

* IFC2x3
* IFC4
* IFC4x3 (Partial material support)

A video summary covering the tool and GUI can be found [here](https://www.youtube.com/watch?v=3bJyBj61a-Y)

More information about the input requirement can be found at the [input requirements paragraph](#input-file-requirements).

Below you can see a speed comparison between the software and manual processing of the exterior shell. Note that in this comparison the software creates the exterior shells for LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 while the manual processing only creates LoD 2.2. This example is sped up 5 times.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample2.gif "Speed comparison of the software making LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 vs making LoD 2.2 by hand.")

This program is part of the [CHEK project](https://chekdbp.eu/). Any suggestions, additions or changes that are made by other users could also be utilized by this project.  

## Table of Content

* [How to execute](#how-to-execute)
* [How to build](#how-to-build)
* [GUI](#gui)
* [Input file requirements](#input-file-requirements)
* [Output file structure](#output-file-structure)
* [Outer shell generation methods](#outer-shell-generation-methods)
  * [Lower detail shells](#lower-detail-shells)
  * [Middle level shells](#middle-level-shells)
  * [High level shells](#high-level-shells-wip)
  * [Voxel shells](#voxel-shells)
* [Inner shell generation methods](#inner-shell-generation-methods)
  * [Storey extraction](#storey-extraction)
  * [Room/space extraction](#roomspace-extraction)
  * [Apartment/area extraction](#apartmentarea-extraction)
* [Configuration JSON](#configuration-json)
* [Report JSON](#report-json)
* [Additional stored attributes](#additional-stored-attributes)
  * [Georeferencing](#georeferencing)
  * [Voxel summary values](#voxel-summary-values)
* [References](#references)

## How to execute

The tool can be used directly with the executables located in the release tab. For both Windows and Linux those are:

* Ifc_Envelope_Extractor_ifc2x3(.exe)
* Ifc_Envelope_Extractor_ifc4(.exe)
* Ifc_Envelope_Extractor_ifc4x3(.exe)
* Ext_GUI.exe (windows only)

The *Ifc_Envelope_Extractor_ifc2x3*, *Ifc_Envelope_Extractor_ifc4* or *Ifc_Envelope_Extractor_ifc4x3* application can be called with a path to a configuration JSON file. This config file is used to supply the tool the needed information related to the input IFC model and the desired output ([more info](#configuration-json)). It is important to make sure that the correct executable for the IFC version of the model is used. An IFC4 file will not be processed by the IFC2x3 version of the tool.

If a more direct (human) user friendly approach is desired on windows, the extractors can be configured with the help of the *Ext_GUI.exe* ([more info](#gui)).

## How to build

If it is desired to compile the code locally the following libraries are required:

* [IfcOpenShell](http://ifcopenshell.org/)
* [Nlohmann](https://github.com/nlohmann/json)
* [CJT](https://github.com/jaspervdv/CJT)
* [Boost](https://www.boost.org/)
* [OpenCASCADE](https://dev.opencascade.org/)

To set the IFC version a single line has to be changed. In *helper.h* the first line is "#define *IfcVersion*". The *IfcVersion* can be changed to the supported IFC versions preceded by USE_. The currently supported versions are: *USE_IFC2x3*, *USE_IFC4* and *USE_IFC4x3*. Each version creates an executable that can ONLY process IFC files of the version that the executable was created for.

```c++
#define USE_IFC4x3 //<- change this line to change supported ifc version

#ifdef USE_IFC2x3
#define IfcSchema Ifc2x3
#define buildVersion "IFC2X3"
#define SCHEMA_VERSIONS (2x3)
#define SCHEMA_SEQ (2x3)

#elif defined(USE_IFC4)
#define IfcSchema Ifc4
#define buildVersion "IFC4"
#define SCHEMA_VERSIONS (4)
#define SCHEMA_SEQ (4)

#elif defined(USE_IFC4x3)
#define IfcSchema Ifc4x3
#define buildVersion "IFC4X3"
#define SCHEMA_VERSIONS (4x3)
#define SCHEMA_SEQ (4x3)

#else
#error "No IFC version defined"
#endif // USE_IFC
```

Please note that CJT is developed in tandem with the IFcEnvExtractor. So possible version mismatches may occur due to CJT being updated at a slightly different time compared to the IFcEnvExtractor. If this is encountered feel free to open a new issue and I will resolve it.

## GUI

IfcEnvelopeExtractor can now also be operated via an GUI. This GUI enables the user to easily access a subset of the settings. The goal of the GUI is making the tool more accessible for non-expert users. The GUI is a python based front that automatically generates the configuration JSON and calls the suitable extractor executable.

The GUI can be accessed via two routes:

* In the Pre_Build folder the *Ext_GUI.exe* will start the GUI. Make sure this .exe is always in the same folder as the *Extractor.exe* files.
* In the GUI_code folder the python file will also start the GUI. This would require the user to have python installed on their system.

<p align="center" width="100%">
    <img src="https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/GUI_example.JPG" alt= “” width="300">
</p>

Only a subset of the settings is available from the GUI, if more advanced settings are required a configuration JSON file has to be created. This can be initialized by using the generate button instead of the run button. This will generate a configuration JSON representing the settings set in the GUI and place it in the same folder as the GUI. This configuration JSON can then further be edited. For more info related to the settings see [here](#configuration-json). This section also explains the settings that can be accessed via the GUI.

## Input file requirements

The algorithms in this tool have been developed to work with models created by people who may not be BIM experts. As a result, the tool can accept somewhat unconventional input files. However, To enlarge the chances of success, there are currently some requirements that should be followed whenever possible. This list of requirements can be grouped depending on the geometric reliance of the methods that are applied.

Low geometric dependent abstraction (LoD 0.0, 1.0 & voxel shells):

* Valid IFC4x3, IFC4 or IFC2x3 file
* Valid units
* Correctly IFC class use<sup>1</sup>
* No or limited use of *IfcBuildingElementProxy* objects<sup>2</sup>
* Site modelled with correct IFC classes<sup>3</sup>

Middle geometric dependent abstraction (LoD 0.2, 0.3, 1.2, 1.3, 2.2 exterior):

* Correctly modelled and watertight roofing structure
* No or limited use of curtain wall objects in the roofing structure

(Very) High geometric dependent abstraction (LoD0.2 and 0.3 storeys and LoD3.2 exterior):

* Correctly modelled and watertight building exterior
* No or limited use of curtain wall objects
* Correct *IfcBuildingStorey* use for storey creation<sup>4</sup>

Interior geometric dependent abstraction (LoD0.2, 1.2, 2.2 and 3.2 spaces):

* *IfcSpace* objects present in the model
* Correct IfcSpace use for space and apartment creation (both semantically and geometrically)<sup>5</sup>

Performance of the tool is depending on the complexity of the input model. The tool has proven to function well on simple models in the current state but can struggle on more complex shapes.

<sup>1</sup> Default behavior of the tool is using the room bounding objects presented by [Vaart et al., (2022)](#1) as space dividing objects. These objects are _IfcWall, IfcCurtainWall, IfcWallStandardCase, IfcRoof, IfcSlab, IfcWindow, IfcColumn, IfcBeam, IfcDoor, IfcCovering, IfcMember_ and _IfcPlate_. If default settings are used it is recommended to have these object correctly classified in the file. Additionally added types would require the additional type to be correctly classified as well.

<sup>2</sup> If *IfcBuildingElementProxy* are used make sure that the objects utilizing this class are either all interior or exterior. If not this can severely hamper the performance of the tool.

<sup>3</sup> If the site is modelled with IFC classes that are reserved for the modelling of a building the tool can incorrectly consider these objects as part of the building.

<sup>4</sup> LoD0.2 requires all the storeys to be present at the correct level. It does not work well with half storeys. LoD0.3 requires all the the storeys to be present at the correct level and have the objects placed at these storeys correctly related. It does support half storeys.

<sup>5</sup> Note that IFC output from software like Autodesk Revit constructively fails to correctly set up the relations between spaces and areas which can cause issues during processing.

## Output file structure

The output files are CityJSON format compliant but might follow a different structure than is usually encountered. Every object that represents a building is split up in 2 building parts: The "Outer Shell" and the "Inner Shell". The "Outer Shell" follows the regular structure that is often seen in established city scale models. It is populated with the different exterior LoD shells. The "Inner Shell" is, depending on the process settings and input file quality, split further over storeys, apartments and rooms.

Below the structure of the file can be seen.

```bash
CityJSON File
└── Building Object*
    ├── Outer Shell
    └── Inner Shell*
        └── Storey**
            └── Space/Room**
```

*Object that does not directly store any geometry.

**Depending on the quality of the input, the nature of the input and the user settings these objects can occur many times.

## Outer shell generation methods

### Low geometric dependent abstraction

Low geometric dependent abstraction shells (L0D0.0 and 1.0 exterior) are created based on a subset of vertices of the model. A set of incrementally rotated bounding boxes is created to encapsulate these vertices. The bounding box with the smallest area is converted to the LoD 1.0 envelope. The ground surface of the bounding box is isolated and converted to the LoD 0.0 envelope.

### Middle geometric dependent abstraction

Middle geometric dependent abstraction shells (Lod0.2, 0.3, and 0.4 roof and 1.2, 1.3, and 2.2 exterior) can be extracted primarily based on the model’s roofing structures. These roofing structures are isolated in two steps. The first step is a coarse isolation which is done via column voxelization. The second step is a filtering of the isolated objects via a ray casting process. This will result in a collection of roof surfaces.

To construct the LoD 0.2 roof and 1.2 exterior shell these roof surfaces are all projected to the xy plane and merged in one single surface. This merged surface can be converted to the LoD 0.2 roof. To get the LoD 1.2 exterior shell this surface is extruded in a positive z-direction to the max z-height of the original model.

To construct LoD0.4 roofs all the filtered roof surfaces are trimmed so that no roof surface overhangs over another surface. The LoD0.4 roof surfaces can be extruded downwards to the footprint height and merged to create the LoD2.2 exterior.

To construct the LoD0.3 roofs all the filtered roof surfaces are grouped and flattened. These flattened surfaces are trimmed so that no flattened roof surface overhangs another flattened surface. The LoD0.3 roof surfaces can be extruded downwards to the footprint height and merged to create the LoD1.3 exterior.

### High geometric dependent abstraction

high geometric dependent abstraction shells (Lod0.2 and 0.3 storeys and 0.2, 0.3, and 0.4 footprint) are based on the building's horizontal envelope at certain intervals (the storey elevations).

To construct the LoD0.2 storey an horizontal buffered section is made across the entire model at each of the storey elevations. The resulting shape (per storey) is merged and stored as LoD0.2.

To construct the LoD0.3 storey a horizontal buffered section is made across al the storey's related objects. These resulting shapes are classified as either interior or exterior. The shapes of each group respectively is merged into a simplified representation and stored as LoD0.3.

The footprint geometry of LoD0.2, 0.3, and 0.4 are the same. This geometry is created by making an LoD0.2 storey at the footprint height. The resulting shape will selectively have its inner voids eliminated (such as elevator and staircase shafts).

### Very high geometric dependent abstraction (WIP)

The high level shells (Lod3.2) are extracted via a voxelization system that is refined by ray casting. The voxelization process is inspired by the room growing process described by [Vaart et al., (2022)](#1). The starting point is a voxel that is outside of the building. Every object that intersects with the growing shape is stored. The surfaces of each found object are then filtered with the help of a ray casting system which casts rays from points on each surface to the center of the exterior voxels. The found objects are merged into the LoD3.2 shell.

### Interior geometric dependent abstraction

Interior geometric dependent abstraction (LoD0.2, 1.2, 2.2 and 3.2 spaces) are extracted based on the *IfcSpace* geometry. This choice was made due to time constraints. If this project is funded in the future we hope to be able to change this logic to rely on the actual tangible geometry of the model since *IfcSpace* geometry can be unreliable.

To construct the LoD 0.2 and 1.2 space representation the ceiling surfaces are all projected to the xy plane and merged in one single surface. This merged surface can be converted to the LoD 0.2 representation. To get the LoD 1.2 representation this surface is extruded in a positive z-direction to the max z-height of the original space.

The ceiling surfaces can be extruded downwards to the footprint height and merged to create the LoD2.2 space representation.

The LoD3.2 space representation is a 1:1 conversion of the *IfcSpace* geometry as is.

Due to the heavy reliance on the *IfcSpace* objects there is, aside from LoD5.0 export, no space export if the model has no *IfcSpace* are objects present. The LoD5.0 spaces will be abele to approximate the space's shape but they will only have generic semantic data stored.

### voxel shells

Additionally the tool will also be able to export a shell based on the voxel grid that is used in the other processes. This will be stored as LoD5.0. Note that this is a voxel grid that is used primarily for filtering and ray-casting purposes. Possibly this will follow different rules than are anticipated.

## Inner shell generation methods

Inner shell creation relies heavily on both geometric and semantic data in an IFC file. This means that for (somewhat) reliable interior extraction the model has to be constructed in an accurate way.

### Apartment/area extraction

*Method still in development.*

## Configuration JSON

The configuration json has a very simple structure. An example can be found below:

```json
{
    "Filepaths": {
        "Input" : 
        [
            "path to IFC file",
            "Potential path to other IFC file"
        ],
        "Output" : "path to export (City)JSON file",
        "Report" : "path to export report JSON file"
    },
    "LoD output": [
        5.0,
        0.0,
        0.2,
        0.3,
        1.0,
        1.2,
        1.3,
        2.2,
        3.2
    ],
    "Voxel":{
        "Size": 1,
        "Store values" : 0,
        "Logic" : 3,
        "Coarse filter": true
    },
    "IFC": {
        "Rotation angle" : 90,
        "Default div": true,
        "Ignore proxy": true,
        "Div objects" : [],
        "Ignore voids" : 0,
        "Simplify geometry" : true,
        "Ignore simplification" : [],
        "Correct placement" : true
    },
    "JSON" : {
        "Footprint elevation": 1,
        "Footprint based" : 0,
        "Horizontal section offset": 0,
        "Generate footprint": 1,
        "Generate roof outline": 1,
        "Generate interior": 0,
        "Generate exterior": 1,
        "Generate site": 0,
        "Georeference" : 1,
        "Merge semantic objects": 1,
    },
    "Output format" : {
        "STEP file" : 1,
        "OBJ file" : 1
    },
    "Tolerances" : {
      "Spatial tolerance" : 1e-6,
      "Angular tolerance" : 1e-4,
      "Area tolerance" : 1e-4
    },
    "Generate report": 1,
    "Threads": 12
}
```

The json has mandatory and optional inputs. If the mandatory inputs are missing the process will not execute properly. If the optional inputs are missing a default value will be used or the tool will attempt to pick a suitable value. Some variables are available in the configuration JSON file only ( :ballot_box_with_check: ) while others are available in both the configuration JSON and the GUI ( :white_check_mark: ). The mentioned default values are selected if the entry is missing from the configuration JSON. For certain cases, such as the "IFC" "Rotation angle" and the "Threads" options it is only possible to trigger the default behavior by omitting them from the configuration JSON.

Mandatory:

* :white_check_mark: "Filepaths" "Input" :white_check_mark:
  * Array filled with string, size 1 to ∞
  * All required paths representing all the IFC files constructing a single building.
* :white_check_mark: "Filepaths" "Output" :white_check_mark:
  * String
  * The output CityJSON filepath. The folder structure is required to be existing. The file name should end with .json or .city.json
* :white_check_mark: "LoD output" :white_check_mark:
  * Array filled with floats/double and/or string values, size 1 to ∞
  * The desired LoD output. The options are 0.0, 0.2, 0.3, 0.4, 1.0, 1.2, 1.3, 2.2, 3.2, 5.0 (for a voxel shape), b.0, c.1, c.2, d.1, d.2, e.0 and e.1.

Optional:

* :ballot_box_with_check: "Filepaths" "Report" :ballot_box_with_check:
  * String
  * The output report JSON filepath. Folder structure is required to be existing. The file name should end with .json.
  * Default value = The "Filepaths" "Output" entry path with "_report" added to the file name.
* :white_check_mark: "Voxel" "Size" :white_check_mark:
  * Float/double
  * The x, y and z dimension of the voxels that will be used for the extraction process. A value between 0.5 and 1 often suffices for normal buildings.
  * Default value = 0.5
* :white_check_mark: "Voxel" "Store values" :white_check_mark:
  * Boolean
  * Toggles the computation of general shell summary values based on the voxelized shape and stores this as semantic attributes.
  * Default value = false
* :ballot_box_with_check: "Voxel" "Logic" :ballot_box_with_check:
  * Integer (either 2 or 3)
  * Toggles the voxel intersection logic; 2 = 2D/plane intersection, 3 = 3D/solid intersection.
  * Default value = 3
* :ballot_box_with_check: "Voxel" "Coarse filter" :ballot_box_with_check:
  * Boolean
  * Toggles the use of the voxels to coarsely filter the objects
  * Default value = true
* :ballot_box_with_check: "IFC" "Rotation angle" :ballot_box_with_check:
  * Float/Double
  * Sets the angle for a custom IFC object rotation around the Z-axis during processing. The value in degrees will be used as rotation angle.
  * Default value = rotation that gives smallest bounding box
* :white_check_mark: "IFC" "Default div" :white_check_mark:
  * Boolean
  * Toggles the use of the default space bounding objects.
  * Default value = true
* :white_check_mark: "IFC" "Ignore proxy" :white_check_mark:
  * Boolean
  * Toggles the use of the IfcBuildingElementProxy objects.
  * Default value = yes
* :white_check_mark: "IFC" "Div objects" :white_check_mark:
  * Array filled with string, size 0 to ∞
  * Adds custom space bounding objects to the to be utilized object pool.
  * Default value = empty
* :white_check_mark: "IFC" "Ignore voids" :white_check_mark:
  * int
  * Toggles the use of void objects on the IFC objects. If 2: voids are not applied, if 1: voids are only applied for void objects that are not filled with other objects, if 0: all void objects are applied. GUI allows the choice between 0 and 2. 1 is only available from the ConfigJSON. If voids are not applied processing speed will improve, but accuracy is reduced if there are voids present in the to be evaluated objects.
  * Default value = false
* :ballot_box_with_check: "IFC" "Simplify geometry" :ballot_box_with_check:
  * Boolean
  * Toggles the use of oriented bounding box simplification of *IfcWindow* and *IfcDoor* objects.
  * Default value = true
* :ballot_box_with_check: "IFC" "Ignore simplification" :ballot_box_with_check:
  * Array filled with string, size 0 to ∞
  * Allows to omit certain *IfcDoor* and *IfcWindow* objects from the oriented bounding box simplification process based on the GUID. These omitted objects are used as modelled in the IFC model.
  * Default value = empty
* :ballot_box_with_check: "IFC" "Correct placement"
  * Boolean
  * Toggles if the local placement of the model should be corrected.
* :white_check_mark: "JSON" "Footprint elevation" :white_check_mark:
  * Float/double
  * Sets the level at which a horizontal section will be taken of the building. This section is used to create the footprint.
  * Default value = 0
* :white_check_mark: "JSON" "Footprint based" :white_check_mark:
  * Boolean
  * Toggles footprint based shape creation for LoD1.2, 1.3, and 2.2. This is done by trimming the roofing structures on which LoD1.2, 1.3, and 2.2 are based so that they do not overhang over the footprint shape before extruding them downwards to create the solid representation.
  * Default value = false
* :ballot_box_with_check: "JSON" "Horizontal section offset" :ballot_box_with_check:
  * Float/double
  * Sets how much the footprint and storey sections should be offset from the found/submitted elevation.
  * Default value = 0
* :white_check_mark: "JSON" "Generate footprint" :white_check_mark:
  * Boolean
  * Toggles the export of the footprint for LoD0.2, 0.3, and 0.4. If false the roof outline will be placed at footprint level for LoD0.2.
  * Default value = false
* :white_check_mark: "JSON" "Generate interior" :white_check_mark:
  * Boolean
  * Toggles IfcSpaces to be exported as interior spaces.
  * Default value = false
* :white_check_mark: "JSON" "Generate exterior" :white_check_mark:
  * Boolean
  * Toggles building's exterior generation.
  * Default value = true
* :ballot_box_with_check: "JSON" "Generate site" :ballot_box_with_check:
  * Boolean
  * Toggles building's site copying process.
  * Default value = false
* :white_check_mark: "JSON" "Generate roof outline" :white_check_mark:
  * Boolean
  * Toggles the roof outline to be exported for LoD0.2.
  * Default value = true
* :ballot_box_with_check: "JSON" "Georeference" :ballot_box_with_check:
  * Boolean
  * Toggles (attempted) georeferencing of the output JSON file.
  * Default value = true
* :ballot_box_with_check: "JSON" "Merge semantic objects" :ballot_box_with_check:
  * Boolean
  * Toggles semantic objects to be merged if they have identical attributes.
  * Default value = true
* :white_check_mark: "Output format" "STEP file" :white_check_mark:
  * Boolean
  * Toggles the copying of the output shaped to a .step file.
  * Default value = false
* :white_check_mark: "Output format" "OBJ file" :white_check_mark:
  * Boolean
  * Toggles the copying of the output shaped to a .obj file.
  * Default value = false
* :ballot_box_with_check: "Output report" :ballot_box_with_check:
  * Boolean
  * Toggles the output of a report file, see [this section](#report-json) for more info.
  * Default value = true
* :ballot_box_with_check: "Threads" :ballot_box_with_check:
  * Integer (>0)
  * Sets the maximum allowed threads to be used.
  * Default value = hardware_concurrency (according to std::threads) * 3 - 2

More options will be added in the future.

## Report JSON

**THE REPORT JSON IS STILL BEING DEVELOPED SO CERTAIN ELEMENTS CAN BE MISSING OR PARTIALLY IMPLEMENTED**.

The report JSON documents all the processes that have been executed by the application. The report JSON can be split in three different sections, the input settings, duration and errors.

The input settings section of the json show which settings have been used to generate the related CityJSON file. These settings use the same variable and object names as used in the configuration JSON. Thus, the report JSON will reflect the configuration JSON but extended with the used default settings, if there were default settings present.

The duration section shows the computing time for each part of the process. This is timed by the *chrono* library.

The errors section shows the issues that the tool encountered during processing. This section will also include warnings. If the process was terminated in an unexpected way possibly this section will document the reason for the termination (WIP).

## Additional stored attributes

### Georeferencing

The CityJSON output can be properly georeferenced if the input IFC file has a proper georeferencing. For IFC4 the method described [here](https://www.buildingsmart.org/wp-content/uploads/2020/02/User-Guide-for-Geo-referencing-in-IFC-v2.0.pdf) is adhered to. For IFC2x3 a custom method is apply that is used by the [ifcgref](https://github.com/tudelft3d/ifcgref) tool. If these methods are not followed the output JSON is either not georeferenced or incorrectly georeferenced.

### Voxel summary values

The EnvExtractor allows the voxel shell to be used for the approximation of a set of building output variables. Current computed values are the:

* Shell area
* Basement shell area
* Building shell area
* Footprint area
* Facade opening area
* Shell volume
* Basement shell volume
* Building shell volume

These attributes are prefaced with "Env_ex V" to highlight that these values are approximated utilizing the voxel grid. The accuracy of these variables are heavily dependent on the size of the utilized voxels and the shape of the buildings. If the proper voxel size is selected the volume approximations and the footprint area can be very accurate. However, the shell area computations area extremely unreliable regardless of the input variables.

## References

<a id="2"></a>
Biljecki, F., Ledoux, H., & Stoter, J. (2016). An improved LOD specification for 3D building models. Computers, environment and urban systems, 59, 25-37.

<a id="1"></a>
van der Vaart, J. A. J. (2022, June 6). Automatic Building Feature Detection and Reconstruction in IFC models. TU Delft Repositories. Retrieved March 30, 2023, from https://repository.tudelft.nl/islandora/object/uuid:db6edbfc-5310-47db-b2c7-3d8e2b62de0f
