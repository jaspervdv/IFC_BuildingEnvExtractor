# IfcEnvelopeExtractor

<!-- markdownlint-disable MD033 -->
<!-- markdownlint-disable MD034 -->

The IfcEnvelopeExtractor enables users to automatically extract the building shell of an IFC-model and convert it to a CityJSON model. Automating this process allows designs to be quickly and easily analyzed on a city scale without the need for lengthy manual conversions. This is one of the steps required to close the gap between architecture/BIM and city scale models.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample.gif "An example of the created LoD envelopes based on an input file")

The software is able to extract multiple different LoD (Level of Detail) shells from an IFC-model. The actual LoDs it is able to extract is dependent on the accuracy and validity of the input model.

Current possible output shells:

* Lod0.0 - exterior only
* Lod0.2 - exterior roof outline, footprint, interior stories and rooms
* LoD0.3 - exterior roof structure and interior stories (WIP)
* LoD1.0 - exterior only
* LoD1.2 - exterior and rooms
* LoD1.3 - exterior only
* LoD2.2 - exterior and rooms
* LoD3.2 - exterior and rooms (WIP)
* LoD5.0 - exterior and interior rooms

Current supported IFC versions:

* IFC2x3
* IFC4
* IFC4x3

For the extraction of the exterior shell the tool utilizes three different extraction methods that can be used on progressively more accurate models. Lower detail shells (LoD0.0 & 1.0) can be extracted based on only the vertices present in a model. Middle level detail shells (Lod0.2, 0.3, 1.2, 1.3, 2.2) can be extracted based on the model’s roofing structures. High level detail shells (Lod3.2) can be extracted based on the model’s objects that are part of the building envelope. This final extraction step only functions on well-constructed models, but yields an accurate result that allows for overhang and underpasses. These features are currently in practice only present in models that are made manually.

The extraction of interior shells relies more heavily on the source IFC file and does not scale like the exterior shells. For all interior export (Storeys, apartments/areas and rooms/spaces) a high quality input model is required with additional extra "special" semantic information to help the tool function.

More information about the input requirement can be found at the input requirements paragraph ([here](#input-file-requirements)).

Below you can see a speed comparison between the software and manual processing of the exterior shell. Note that in this comparison the software creates the exterior shells for LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 while the manual processing only creates LoD 2.2. This example is sped up 5 times.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample2.gif "Speed comparison of the software making LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 vs making LoD 2.2 by hand.")

This program is part of the [CHEK project](https://chekdbp.eu/). Any suggestions, additions or changes that are made by other users could also be utilized by this project.  

**Development of this tool will continue until the end of September 2025, after this current funding will end.**

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

The _Ifc_Envelope_Extractor_ifc2x3_, _Ifc_Envelope_Extractor_ifc4_ or _Ifc_Envelope_Extractor_ifc4x3_ application can be called with a path to a configuration JSON file. This config file is used to supply the tool the needed information that is related to the IFC model ([more info](#configuration-json)). This enables the extractor to be easily called by other applications. It is important to make sure that the correct executable for the IFC version of the model is used. An IFC4 file will not be processed by the IFC2x3 version of the tool.

If a more direct (human) user friendly approach is desired on windows, the extractors can be configured with the help of the _Ext_GUI.exe_ ([more info](#gui)).

## How to build

If it is desired to compile the code locally the following libraries are required:

* [IfcOpenShell](http://ifcopenshell.org/)
* [Nlohmann](https://github.com/nlohmann/json)
* [CJT](https://github.com/jaspervdv/CJT)
* [Boost](https://www.boost.org/)
* [OpenCASCADE](https://dev.opencascade.org/)

To set the IFC version of the tool a single line has to be changed. In _helper.h_ the first lines are "#define _IfcVersion_". The _IfcVersion_ can be changed to the supported IFC versions preceded by USE_. The currently supported versions are: USE_IFC2x3, USE_IFC4 and USE_IFC4x3. Each version creates an executable that can ONLY process IFC files of the version that the executable was created for.

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

Please note that CJT is developed in tandem with the IFcEnvExtractor. So possible version mismatches may occur due to CJT being updated at a slightly different time compared to the IFcEnvExtractor.

## GUI

IfcEnvelopeExtractor can now also be operated via an GUI. This GUI enables the user to easily change the settings. The goal of the GUI is making the tool more accessible for non-expert users. The GUI is a python based front that automatically generates the configuration JSON and calls the suitable extractor executable.

The GUI can be accessed via two routes:

* In the Pre_Build folder the _Ext_GUI.exe_ will start the GUI. Make sure this .exe is always in the same folder as the Extractor.exe files.
* In the GUI_code folder the python file will also start the GUI. This would require the user to have python installed on their system.

<p align="center" width="100%">
    <img src="https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/GUI_example.JPG" alt= “” width="300">
</p>

Only a subset of the settings is available from the GUI, if more advanced settings are required a configuration JSON file has to be created. This can be initialized by using the generate button instead of the run button. This will generate a configuration JSON representing the settings set in the GUI and place it in the same folder as the GUI. This configuration JSON can then further be edited. For more info related to the settings see [here](#configuration-json). This section also explains the settings that can be accessed via the GUI.

## Input file requirements

The algorithms in this tool have been developed to work with models created by people who may not be BIM experts. As a result, the tool can accept somewhat unconventional input files. However, To enlarge the chances of success, there are currently some requirements that should be followed whenever possible. The list with requirements for exterior extraction is divided over the three different complexity levels ([more info](#outer-shell-generation-methods)). The requirements per level build on top of the ones of lower levels. This means that for middle level shell extraction, the middle detail requirements + the lower detail requirements are recommended. The requirements within one level are listed in order of importance.

These levels only apply to LoD models that are build WITHOUT interiors. If interior generation is enabled the requirements from all levels (low, middle and high) and the additional interior shell requirements have to be met.

Lower detail shells (LoD 0.0, 1.0 & voxel shells):

* Valid IFC4x3, IFC4 or IFC2x3 file
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
* No or limited use of "half storeys"
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
            └── Space/Room**
```

*Object that does not directly store any geometry.

**Depending on the quality of the input, the nature of the input and the user settings these objects can occur many times.

## Outer shell generation methods

### Lower detail shells

The lower level LoD shells are created based on a subset of vertices of the model. This subset is collected from the _IfcWall, IfcWallStandardCase, IfcRoof, IfcSlab_ and _IfcWindow_ objects. A set of incrementally rotated bounding boxes is created to encapsulate these vertices. The bounding box with the smallest area is converted to the LoD 1.0 envelope. The ground surface of the bounding box is isolated and converted to the LoD 0.0 envelope.

### Middle level shells

Middle level detail shell (Lod0.2, 1.2, 1.3, 2.2) can be extracted primarily based on the model’s roofing structures. These roofing structures are isolated in two steps. The first step is a coarse isolation which is done via column voxelization. The second step is a filtering of the isolated objects via a column-like ray casting process. This will result in a collection of roofing surfaces.

To construct the LoD 0.2 and 1.2 shell these surfaces (And their vertices) are all projected to the xy plane and merged in one single surface. This surface can be converted to the LoD 0.2 shell. To get the LoD 1.2 shell this surface is extruded in a positive z-direction to the max z-height of the original model.

To construct the LoD 1.3 shell all the filtered roofing surfaces are also projected to the xy plane, but not merged. Instead every surface is translated back in the z-direction to the z-max of their original surface shape. These translated shapes are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 1.3 shell.

To construct the LoD 2.2 shell all the filtered roofing surfaces are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 2.2 shell.

### High level shells (WIP)

The high level shells (Lod3.2) are extracted via a voxelization system that is refined by ray casting. The voxelization process is inspired by the room growing process described by [Vaart et al., (2022)](#1). The starting point is a voxel that is outside of the building. Every object that intersects with the growing shape is stored. The surfaces of each found object are then filtered with the help of a ray casting system which casts rays from points on each surface to the center of the exterior voxels. The found objects are merged into the LoD3.2 shell.

At this moment in time this extraction method does not yet return solids.

### voxel shells

Additionally the tool will also be able to export a shell based on the voxel grid that is used in the other processes. This will be stored as LoD5.0. Note that this is a voxel grid that is used primarily for filtering and ray-casting purposes. Possibly this will follow different rules than are anticipated.

## Inner shell generation methods

**THIS EXTRACTION IS NOT YET PROPERLY IMPLEMENTED; THIS SECTION WILL NOT BE COMPLETELY ACCURATE**.

Inner shell creation relies heavily on both geometric and semantic data in an IFC file. This means that for (somewhat) reliable interior extraction the model has to be constructed in an accurate way.

### Storey extraction

For storey extraction the elevations and related objects of _IfcBuildingStorey_ objects are used. The related objects are split horizontally 50 cm above the storey's stored elevation. their outlines are projected on a horizontal plane and used to split this plane. The resulting outer wires of the split shapes are collected and converted into surfaces.

### Room/space extraction

For space extraction the _IfcSpace_ Objects are processed according to the LoD framework. For LoD 0.2 the top surfaces of the space objects are isolated, projected to the room's lowest z-height and merged into a single surface per room (if possible).

The isolated room top surfaces are also used for LoD1.3 and 2.2. For LoD2.2 these surfaces are kept at their original height but extruded downwards to the room's lowest z-height to create solids. And for LoD1.3 these surfaces are first flattened and extruded downwards.

The extraction of LoD3.2 space shapes is a 1:1 conversion of the _IfcSpace_ geometry.

LoD5.0 space shells are created by using the voxel grid but instead of growing from a point that is presumed to be exterior it is started from points that are presumed to be interior. The resulting shape is than compared to the pool of _IfcSpace_ objects in the model to find its correct space name and other semantic data.

Due to the heavy reliance of the _IfcSpace_ objects there is, aside from LoD5.0 export, no space export if the model has no _IfcSpace_ objects present. The LoD5.0 spaces will be abele to approximate the space's shape but they will only have generic semantic data stored.

### Apartment/area extraction

Method still in development

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
        "Logic" : 3
    },
    "IFC": {
        "Rotation angle" : 90,
        "Default div": true,
        "Ignore proxy": true,
        "Div objects" : [],
        "Simplify geometry" : 0
    },
    "JSON" : {
        "Footprint elevation": 1,
        "Footprint based" : 0,
        "Horizontal section offset": 0,
        "Generate footprint": 1,
        "Generate roof outline": 1,
        "Generate interior": 1,
        "Generate exterior": 1,
        "Georeference" : 1,
        "Merge semantic objects": 1,
    },
    "Generate report": 1,
    "Threads": 12
}
```

The json has mandatory and optional inputs. If the mandatory inputs are missing the process will not execute properly. If optional inputs are missing a default value will be used. Variables are available in the configuration JSON file only ( :ballot_box_with_check: ) or in both the configuration JSON and the GUI ( :white_check_mark: ). The mentioned default values are selected if the entry is missing from the configuration JSON. For certain cases, such as the "IFC" "Rotation angle" and the "Threads" options it is only possible to trigger the default behavior by not adding them to the configuration JSON.

Mandatory:

* :white_check_mark: "Filepaths" "Input" :white_check_mark:
  * Array filled with string, size 1 to ∞
  * All required paths representing all the IFC files constructing a single building.
* :white_check_mark: "Filepaths" "Output" :white_check_mark:
  * String
  * The output CityJSON filepath. Folder structure is required to be existing. The file name should end with .json or .city.json/
* :white_check_mark: "LoD output" :white_check_mark:
  * Array filled with floats/double, size 1 to ∞
  * the desired LoD output. The options are 0.0, 0.2, 0.3, 1.0, 1.2, 1.3, 2.2, 3.2 and 5.0 (for a voxel shape).

Optional:

* :ballot_box_with_check: "Filepaths" "Report" :ballot_box_with_check:
  * String
  * The output report JSON filepath. Folder structure is required to be existing. The file name should end with .json.
  * Default value = "Filepaths" "Output" path with "_report" added to the file name.
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
  * Toggle the voxel intersection logic; 2 = 2D/plane intersection, 3 = 3D/solid intersection.
  * Default value = 3
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
  * Default value: yes
* :white_check_mark: "IFC" "Div objects" :white_check_mark:
  * Array filled with string, size 0 to ∞
  * Adds more custom space bounding objects to the processing.
  * Default value = empty
* :white_check_mark: "IFC" "Simplify geometry" :white_check_mark:
  * int
  * Toggles the use of void objects on the IFC objects. If 2: voids are not applied, if 1: voids are only applied for void objects that are not filled with other objects, if 0: all void objects are applied. GUI allows the choice between 0 and 2. 1 is only available from the ConfigJSON. If voids are not applied processing speed will improve, but accuracy is reduced if there are voids present in the to be evaluated objects. Buggy former behavior was ignoring voids.
  * Default value = false
* :white_check_mark: "JSON" "Footprint elevation" :white_check_mark:
  * Float/double
  * Sets the level at which a horizontal section will be taken of the building. This section is used to create the footprint.
  * Default value = 0
* :white_check_mark: "JSON" "Footprint based" :white_check_mark:
  * Boolean
  * Toggles footprint based shape creation for LoD1.2, 1.3, and, 2.2.
  * Default value = false
* :ballot_box_with_check: "JSON" "Horizontal section offset" :ballot_box_with_check:
  * Float/double
  * Sets how much the footprint and storey sections should be offset from the found/submitted elevation.
  * Default value = 0
* :white_check_mark: "JSON" "Generate footprint" :white_check_mark:
  * Boolean
  * Toggles the export of the footprint for LoD0.2. If false the roof outline will be placed at footprint level.
  * Default value = false
* :white_check_mark: "JSON" "Generate interior" :white_check_mark:
  * Boolean
  * Toggles IfcSpaces to be exported as interior spaces.
  * Default value = false
* :ballot_box_with_check: "JSON" "Generate exterior" :ballot_box_with_check:
  * Boolean
  * Toggles building's exterior generation
  * Default value = true
* :white_check_mark: "JSON" "Generate roof outline" :white_check_mark:
  * Boolean
  * Toggles the roof outline to be exported for LoD0.2
  * Default value = true
* :ballot_box_with_check: "JSON" "Georeference" :ballot_box_with_check:
  * Boolean
  * Toggles (attempted) georeferencing of the output JSON file.
  * Default value = true
* :ballot_box_with_check: "JSON" "Merge semantic objects" :ballot_box_with_check:
  * Boolean
  * Toggles semantic objects to be merged if they have identical attributes.
  * Default value = true
* :white_check_mark: "Output report" :white_check_mark:
  * Boolean
  * Toggles the output of a report file, see [this section](#report-json) for more info.
  * Default value = true
* :ballot_box_with_check: "Threads" :ballot_box_with_check:
  * Integer (>0)
  * Sets the maximum allowed threads to be used.
  * Default value = hardware_concurrency - 2 according to std::threads

More options will be added in the future.

## Report JSON

**THE REPORT JSON IS STILL BEING DEVELOPED SO CERTAIN ELEMENTS CAN BE MISSING OR PARTIALLY IMPLEMENTED**.

The report JSON documents all the processes that have been executed by the application. The report JSON can be split in three different sections, the input settings, duration and errors.

The input settings section of the json show which settings have been used to generate the related CityJSON file. These settings use the same variable and object names as used in the configuration JSON. Thus, the report JSON will reflect the configuration JSON but extended with the used default settings, if there were default settings present.

the duration section shows the computing time for each part of the process. This is clocked by the chrono library.

The errors section shows the issues that the tool encountered during processing. This section will also include warnings. If the process was terminated in an unexpected way possibly this section will document the reason for the termination.

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

<a id="1"></a>
van der Vaart, J. A. J. (2022, June 6). Automatic Building Feature Detection and Reconstruction in IFC models. TU Delft Repositories. Retrieved March 30, 2023, from https://repository.tudelft.nl/islandora/object/uuid:db6edbfc-5310-47db-b2c7-3d8e2b62de0f
