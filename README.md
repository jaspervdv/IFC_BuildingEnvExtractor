# IfcEnvelopeExtractor

The IfcEnvelopeExtractor enables users to automatically extract the building envelope of an IFC-model and convert it to a CityJSON model. Automating this process allows designs to be quickly and easily analyzed on a city scale without the need for lengthy manual conversions. This is one of the steps required to close the gap between architecture/BIM and city scale models.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample.gif "An example of the created LoD envelopes based on an input file")

The software is able to extract multiple different LoD (Level of Detail) envelopes from an IFC-model. The actual LoDs it is able to extract is dependent on the accuracy and validity of the input model. The tool utilizes three different extraction methods that can be used on progressively more accurate models. Lower detail envelopes (LoD 0.0 & 1.0) can be extracted only based on the vertices present in a model. Middle level detail envelopes (Lod 0.2, 1.2, 1.3, 2.2) can be extracted based on the model’s roofing structures. High level detail envelopes (Lod 3.2) can be extracted based on the model’s objects that are part of the building envelope. This final extraction step only functions on well-constructed models, but yields an accurate result that allows for overhang and underpasses. These features are often only present in models that are made manually.

Below you can see a speed comparison between the software and manual processing. Note that the software creates the envelopes for LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 while the manual processing only creates LoD 2.2. This example is speed up to 5 times speed.

![Output of the IfcEnvelopeExtractor](https://raw.githubusercontent.com/jaspervdv/IFC_BuildingEnvExtractor/master/Images/EnvExtractorExample2.gif "Speed comparison of the software making LoD 0.0, 0.2, 1.0, 1.2, 2.2 and 3.2 vs making LoD 2.2 by hand.")

This program is part of the [CHECK project](https://chekdbp.eu/). Any suggestions, additions or changes that are made by other users will also be utilized by this project.  

## Envelope generation

### Lower detail envelopes

The lower level LoD envelopes are created based on a subset of vertices of the model. This subset is collected from the IfcWall, IfcWallStandardCase, IfcRoof, IfcSlab and IfcWindow objects. A set of incrementally rotated bounding boxes is created to encapsulate these vertices. The bounding box with the smallest area is converted to the LoD 1.0 envelope. The ground surface of the bounding box is isolated and converted to the LoD 0.0 envelope.

### Middle level envelopes

Middle level detail envelopes (Lod 0.2, 1.2, 1.3, 2.2) can be extracted based on the model’s roofing structures. These roofing structures are isolated in two steps. The first step is a coarse isolation is done via a column voxelization. The second step is a filtering of the isolated objects via a column like ray casting process. This will result in a collection of roofing surfaces.

To construct the LoD 0.2 and 1.2 envelopes these surfaces (And their vertices) are all projected to the xy plane and merged in one single surface. This surface can be converted to the LoD 0.2 envelope. To get the LoD 1.2 envelope this surface is extrude in a positive z-direction to the max height of the original model.

To construct the LoD 1.3 envelope all the filtered roofing surfaces are also projected to the xy plane, but not merged. Instead every surface is translated in the z-direction to the z-max of their original surface shape. These translated shapes are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 1.3 envelope.

To construct the LoD 2.2 envelope all the filtered roofing surfaces are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 2.2 envelope.

### High level envelopes

The high level envelopes are extracted via a boolean processing of the objects that make up the building envelope. These objects are first isolated via a voxelization process. Ths voxilization process functions is inspired by the room growing process described by [Vaart et al., (2022)](#1). The starting point is a voxel that is outside of the building. Every object that intersects with the growing shape is stored.

The resulting list of objects is used to boolean split a bounding box encompassing the entire voxel grid. From the resulting shape the outer most solid is extracted. From this solid the shells representing the building shapes are isolated and converted to the LoD 3.2 envelopes.

## Input file requirements

The algorithms in this tool have been developed to work with models created by people who may not be BIM experts. As a result, the tool has been developed to accept unconventional input files. However, To enlarge the chances of success, there are currently some requirements that should be followed whenever possible. The list with requirements is divided over the three different levels of processing that were mentioned before. The requirements per level build on top of the ones of lower levels. So for middle level envelopes the mentioned requirements + the lower detail requirements are recommended. The requirements within one level are listed in order of importance.

Lower detail envelopes (LoD 0.0 & 1.0):

* Valid IFC4 or IFC2x3 file
* Valid units
* Correctly classified objects* (recommended)
* No or limited use of IfcBuildingElementProxy objects (recommended)
* Site and building structurally separated (recommended)

Middle level envelopes (Lod 0.2, 1.2, 1.3, 2.2)

* Correctly modelled and watertight roofing structure
* No or limited use of triangulated objects (recommended)

High level envelopes (LoD 3.2)

* Correctly modelled and watertight building exterior

*The types of object that are utilized by the program and thus have to be correctly classified are: IfcWall, IfcWallStandardCase, IfcRoof, IfcSlab and IfcWindow for lower and middle detail envelopes. For higher level envelopes this list is extended with IfcColumn, IfcBeam, IfcDoor, IfcCovering and IfcPlate.

## How to run
The tool can be used directly with the executables located in the Pre_Build folder. Make sure that you run the correct executable for the IFC version of the model. An IFC4 file will not be processed by the IFC2x3 version of the tool. It is also possible to start the tool from the console or call it with other software. This is however still a fairly new and untested feature, more detail on how this works will come soon<sup>tm</sup>.

If it is desired to compile the code locally the following libraries are required:

* [IfcOpenShell](http://ifcopenshell.org/)
* [Nlohmann](https://github.com/nlohmann/json)
* [CJT](https://github.com/jaspervdv/CJT)
* [Boost](https://www.boost.org/)
* [OpenCASCADE](https://dev.opencascade.org/)

In main.cpp there is a vector called "sourcePathArray". In this vector filepaths are hard coded, this is mostly for quick testing. If this array is not empty the tool will directly process the files at the filepaths found in this array at executable start. If the array is emptied the program will allow the user to submit one, or multiple filepaths when the executable is started.

In helper.h the first line is "#define USE_IFC4". keeping this line will build an executable that is able to process IFC4. If the line is commented out or removed the build executable will be able to process IFC2x3 files.`

Please note that CJT is developed in tandem with the IFcEnvExtractor. So possible version mismatches may occur due to CJT being updated later than the IFcEnvExtractor.

## References
<a id="1"></a> 
van der Vaart, J. A. J. (2022, June 6). Automatic Building Feature Detection and Reconstruction in IFC models. TU Delft Repositories. Retrieved March 30, 2023, from https://repository.tudelft.nl/islandora/object/uuid:db6edbfc-5310-47db-b2c7-3d8e2b62de0f 