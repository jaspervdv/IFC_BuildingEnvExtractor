# IfcEnvelopeExtractor

The IfcEnvelopeExtractor enables users to automatically extract the building envelope of an IFC- model and convert it to a CityJSON model.
Automating this process allows designs to be easily analyzed on a city scale without the need for manual conversion.
This is one of the steps required to close the gap between architecture/BIM and city scale models.

The software is able to extract multiple different LoD (Level of Detail) envelopes from an IFC-model.
The actual LoDs it is able to extract is dependent on the accuracy and validity of the input model.
The tool utilizes three different extraction methods that can be used on progressively more accurate models.
Lower detail envelopes (LoD 0.0 & 1.0) can be extracted only based on the vertices present in a model.
Middle level detail envelopes (Lod 0.2, 1.2, 1.3, 2.2) can be extracted based on the model’s roofing structures.
High level detail envelopes (Lod 3.2) can be extracted based on the model’s objects that are part of the building envelope.
This final extraction step only functions on well-constructed models, but yields an accurate result that allows for overhang and underpasses.
These features are often only present in models that are made manually.

## Envelope generation

### Lower detail LoD (LoD 0.0 & 1.0)

The lower level LoD envelopes are created based on a subset of vertices of the model.
This subset is collected from the IfcWall, IfcWallStandardCase, IfcRoof, IfcSlab and IfcWindow objects.
A set of incrementally rotated bounding boxes is created to encapsulate these vertices.
The bounding box with the smallest area is converted to the LoD 1.0 envelope.
The ground surface of the bounding box is isolated and converted to the LoD 0.0 envelope.

### Middle level envelopes

Middle level detail envelopes (Lod 0.2, 1.2, 1.3, 2.2) can be extracted based on the model’s roofing structures.
These roofing structures are isolated in two steps.
The first step is a coarse isolation is done via a column voxelization.
The second step is a filtering of the isolated objects via a column like ray casting process.
This will result in a collection of roofing surfaces.

To construct the LoD 0.2 and 1.2 envelopes these surfaces (And their vertices) are all projected to the xy plane and merged in one single surface. This surface can be converted to the LoD 0.2 envelope. To get the LoD 1.2 envelope this surface is extrude in a positive z-direction to the max height of the original model.

To construct the LoD 1.3 envelope all the filtered roofing surfaces are also projected to the xy plane, but not merged. Instead every surface is translated in the z-direction to the z-max of their original surface shape. These translated shapes are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 1.3 envelope.

To construct the LoD 2.2 envelope all the filtered roofing surfaces are extruded in a negative x-direction towards the xy plane. The resulting solids are merged and converted to the LoD 2.2 envelope.

### High level envelopes

The high level envelopes are extracted via a boolean processing of the objects that make up the building envelope. These objects are first isolated via a voxelization process. Ths voxilization process functions is inspired by the room growing process described by [Vaart et al., (2022)](#1) The starting point is a voxel that is outside of the building. Every object that intersects with the growing shape is stored.

The resulting list of objects is used to boolean split a bounding box encompassing the entire voxel grid. From the resulting shape the outer most solid is extracted. From this solid the shells representing the building shapes are isolated and converted to the LoD 3.2 envelopes.

## References
<a id="1"></a> 
van der Vaart, J. A. J. (2022, June 6). Automatic Building Feature Detection and Reconstruction in IFC models. TU Delft Repositories. Retrieved March 30, 2023, from https://repository.tudelft.nl/islandora/object/uuid:db6edbfc-5310-47db-b2c7-3d8e2b62de0f 