#include "helper.h"
#include "roomProcessor.h"

#include <unordered_set>
#include <string>
#include <vector>


class IOManager {
private:

	// if true no comminucation is pushed to console
	bool isSilent_ = false;

	// if programm is instructed by a json file = true
	bool isJsonInput_ = false;

	std::vector<std::string> inputPathList_ = {};
	std::string outputFolderPath_ = "";

	// sets which LoD envelopes are attampted to be created
	bool make00_ = true;
	bool make02_ = true;
	bool make10_ = true;
	bool make12_ = true;
	bool make13_ = true;
	bool make22_ = true;
	bool make32_ = true;

	bool writeReport_ = true;

	// variables set the deviding objects
	bool useDefaultDiv_ = true;
	bool useProxy_ = false;

	double voxelSize_ = 0.5;

	double footprintElevation_ = 0;

	// how many proxy objects are present in the input
	int proxyCount_ = 0;

	helperCluster hCluster_;

	CJT::CityCollection cityCollection_;

	// time summary for the output
	double timeInternalizing_ = -1;
	double timeInternalizingIsS_ = false;

	double timeLoD00_ = -1;
	bool timeLoD00IsS_ = false;
	double timeLoD02_ = -1;
	bool timeLoD02IsS_ = false;
	double timeLoD10_ = -1;
	bool timeLoD10IsS_ = false;
	double timeLoD12_ = -1;
	bool timeLoD12IsS_ = false;
	double timeLoD13_ = -1;
	bool timeLoD13IsS_ = false;
	double timeLoD22_ = -1;
	bool timeLoD22IsS_ = false;
	double timeLoD32_ = -1;
	bool timeLoD32IsS_ = false;

	double timeProcess = -1;
	double timeTotal = -1;

	std::vector<std::string> ErrorList_;

	// question askers
	bool yesNoQuestion();
	int numQuestion(int n, bool lower = true);

	bool getTargetPathList();
	bool getOutputPathList();
	std::string getFileName(const std::string& stringPath);

	bool getUseDefaultSettings();

	bool getDesiredLoD();

	bool getBoudingRules();

	bool getVoxelSize();

	bool getFootprintElev();

	// attempts to ask the user for settings
	bool getUserValues();

	// attempts to get the settings from json file
	bool getJSONValues();

	// checks if the string has the extension that is supplied
	bool hasExtension(const std::vector<std::string>& stringList, const std::string& ext);
	bool hasExtension(const std::string& string, const std::string& ext);

	bool isValidPath(const std::vector<std::string>& path);
	bool isValidPath(const std::string& path);

	// console outputs the settings that are utilized
	void printSummary();

	std::string getLoDEnabled();

	nlohmann::json settingsToJSON();


public:
	bool init(const std::vector<std::string>& inputPathList, bool silent = false);

	bool run();

	bool write();

	// temp data

	double voxelSize() { return voxelSize_; }
	bool makeReport() { return writeReport_; }

	std::string getOutputPath() { return outputFolderPath_; }

	helperCluster helpCluster() { return hCluster_; }

	bool makeLoD00() { return make00_; }
	bool makeLoD02() { return make02_; }
	bool makeLoD10() { return make10_; }
	bool makeLoD12() { return make12_; }
	bool makeLoD13() { return make13_; }
	bool makeLoD22() { return make22_; }
	bool makeLoD32() { return make32_; }

	std::unordered_set<std::string> divObjects_ = { // Only used for output purposes
		"IFCSLAB",
		"IFCROOF",
		"IFCWALL",
		"IFCWALLSTANDARDCASE",
		"IFCCOVERING",
		"IFCCOLUMN",
		"IFCBEAM",
		"IFCCURTAINWALL",
		"IFCPLATE",
		"IFCMEMBER",
		"IFCDOOR",
		"IFCWINDOW"
	};

	std::unordered_set<std::string> addDivObjects_ = {
	};

	std::unordered_set<std::string> DevObjectsOptions_ = {
		"IFC2DCOMPOSITECURVE",
		"IFCACTIONREQUEST",
		"IFCACTOR",
		"IFCACTORROLE",
		"IFCACTUATORTYPE",
		"IFCADDRESS",
		"IFCAIRTERMINALBOXTYPE",
		"IFCAIRTERMINALTYPE",
		"IFCAIRTOAIRHEATRECOVERYTYPE",
		"IFCALARMTYPE",
		"IFCANGULARDIMENSION",
		"IFCANNOTATION",
		"IFCANNOTATIONCURVEOCCURRENCE",
		"IFCANNOTATIONFILLAREA",
		"IFCANNOTATIONFILLAREAOCCURRENCE",
		"IFCANNOTATIONOCCURRENCE",
		"IFCANNOTATIONSURFACE",
		"IFCANNOTATIONSURFACEOCCURRENCE",
		"IFCANNOTATIONSYMBOLOCCURRENCE",
		"IFCANNOTATIONTEXTOCCURRENCE",
		"IFCAPPLICATION",
		"IFCAPPLIEDVALUE",
		"IFCAPPLIEDVALUERELATIONSHIP",
		"IFCAPPROVAL",
		"IFCAPPROVALACTORRELATIONSHIP",
		"IFCAPPROVALPROPERTYRELATIONSHIP",
		"IFCAPPROVALRELATIONSHIP",
		"IFCARBITRARYCLOSEDPROFILEDEF",
		"IFCARBITRARYOPENPROFILEDEF",
		"IFCARBITRARYPROFILEDEFWITHVOIDS",
		"IFCASSET",
		"IFCASYMMETRICISHAPEPROFILEDEF",
		"IFCAXIS1PLACEMENT",
		"IFCAXIS2PLACEMENT2D",
		"IFCAXIS2PLACEMENT3D",
		"IFCBSPLINECURVE",
		"IFCBEAM",
		"IFCBEAMTYPE",
		"IFCBEZIERCURVE",
		"IFCBLOBTEXTURE",
		"IFCBLOCK",
		"IFCBOILERTYPE",
		"IFCBOOLEANCLIPPINGRESULT",
		"IFCBOOLEANRESULT",
		"IFCBOUNDARYCONDITION",
		"IFCBOUNDARYEDGECONDITION",
		"IFCBOUNDARYFACECONDITION",
		"IFCBOUNDARYNODECONDITION",
		"IFCBOUNDARYNODECONDITIONWARPING",
		"IFCBOUNDEDCURVE",
		"IFCBOUNDEDSURFACE",
		"IFCBOUNDINGBOX",
		"IFCBOXEDHALFSPACE",
		"IFCBUILDING",
		"IFCBUILDINGELEMENT",
		"IFCBUILDINGELEMENTCOMPONENT",
		"IFCBUILDINGELEMENTPART",
		"IFCBUILDINGELEMENTPROXY",
		"IFCBUILDINGELEMENTPROXYTYPE",
		"IFCBUILDINGELEMENTTYPE",
		"IFCBUILDINGSTOREY",
		"IFCCSHAPEPROFILEDEF",
		"IFCCABLECARRIERFITTINGTYPE",
		"IFCCABLECARRIERSEGMENTTYPE",
		"IFCCABLESEGMENTTYPE",
		"IFCCALENDARDATE",
		"IFCCARTESIANPOINT",
		"IFCCARTESIANTRANSFORMATIONOPERATOR",
		"IFCCARTESIANTRANSFORMATIONOPERATOR2D",
		"IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM",
		"IFCCARTESIANTRANSFORMATIONOPERATOR3D",
		"IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM",
		"IFCCENTERLINEPROFILEDEF",
		"IFCCHAMFEREDGEFEATURE",
		"IFCCHILLERTYPE",
		"IFCCIRCLE",
		"IFCCIRCLEHOLLOWPROFILEDEF",
		"IFCCIRCLEPROFILEDEF",
		"IFCCLASSIFICATION",
		"IFCCLASSIFICATIONITEM",
		"IFCCLASSIFICATIONITEMRELATIONSHIP",
		"IFCCLASSIFICATIONNOTATION",
		"IFCCLASSIFICATIONNOTATIONFACET",
		"IFCCLASSIFICATIONREFERENCE",
		"IFCCLOSEDSHELL",
		"IFCCOILTYPE",
		"IFCCOLOURRGB",
		"IFCCOLOURSPECIFICATION",
		"IFCCOLUMN",
		"IFCCOLUMNTYPE",
		"IFCCOMPLEXPROPERTY",
		"IFCCOMPOSITECURVE",
		"IFCCOMPOSITECURVESEGMENT",
		"IFCCOMPOSITEPROFILEDEF",
		"IFCCOMPRESSORTYPE",
		"IFCCONDENSERTYPE",
		"IFCCONDITION",
		"IFCCONDITIONCRITERION",
		"IFCCONIC",
		"IFCCONNECTEDFACESET",
		"IFCCONNECTIONCURVEGEOMETRY",
		"IFCCONNECTIONGEOMETRY",
		"IFCCONNECTIONPOINTECCENTRICITY",
		"IFCCONNECTIONPOINTGEOMETRY",
		"IFCCONNECTIONPORTGEOMETRY",
		"IFCCONNECTIONSURFACEGEOMETRY",
		"IFCCONSTRAINT",
		"IFCCONSTRAINTAGGREGATIONRELATIONSHIP",
		"IFCCONSTRAINTCLASSIFICATIONRELATIONSHIP",
		"IFCCONSTRAINTRELATIONSHIP",
		"IFCCONSTRUCTIONEQUIPMENTRESOURCE",
		"IFCCONSTRUCTIONMATERIALRESOURCE",
		"IFCCONSTRUCTIONPRODUCTRESOURCE",
		"IFCCONSTRUCTIONRESOURCE",
		"IFCCONTEXTDEPENDENTUNIT",
		"IFCCONTROL",
		"IFCCONTROLLERTYPE",
		"IFCCONVERSIONBASEDUNIT",
		"IFCCOOLEDBEAMTYPE",
		"IFCCOOLINGTOWERTYPE",
		"IFCCOORDINATEDUNIVERSALTIMEOFFSET",
		"IFCCOSTITEM",
		"IFCCOSTSCHEDULE",
		"IFCCOSTVALUE",
		"IFCCOVERING",
		"IFCCOVERINGTYPE",
		"IFCCRANERAILASHAPEPROFILEDEF",
		"IFCCRANERAILFSHAPEPROFILEDEF",
		"IFCCREWRESOURCE",
		"IFCCSGPRIMITIVE3D",
		"IFCCSGSOLID",
		"IFCCURRENCYRELATIONSHIP",
		"IFCCURTAINWALL",
		"IFCCURTAINWALLTYPE",
		"IFCCURVE",
		"IFCCURVEBOUNDEDPLANE",
		"IFCCURVESTYLE",
		"IFCCURVESTYLEFONT",
		"IFCCURVESTYLEFONTANDSCALING",
		"IFCCURVESTYLEFONTPATTERN",
		"IFCDAMPERTYPE",
		"IFCDATEANDTIME",
		"IFCDEFINEDSYMBOL",
		"IFCDERIVEDPROFILEDEF",
		"IFCDERIVEDUNIT",
		"IFCDERIVEDUNITELEMENT",
		"IFCDIAMETERDIMENSION",
		"IFCDIMENSIONCALLOUTRELATIONSHIP",
		"IFCDIMENSIONCURVE",
		"IFCDIMENSIONCURVEDIRECTEDCALLOUT",
		"IFCDIMENSIONCURVETERMINATOR",
		"IFCDIMENSIONPAIR",
		"IFCDIMENSIONALEXPONENTS",
		"IFCDIRECTION",
		"IFCDISCRETEACCESSORY",
		"IFCDISCRETEACCESSORYTYPE",
		"IFCDISTRIBUTIONCHAMBERELEMENT",
		"IFCDISTRIBUTIONCHAMBERELEMENTTYPE",
		"IFCDISTRIBUTIONCONTROLELEMENT",
		"IFCDISTRIBUTIONCONTROLELEMENTTYPE",
		"IFCDISTRIBUTIONELEMENT",
		"IFCDISTRIBUTIONELEMENTTYPE",
		"IFCDISTRIBUTIONFLOWELEMENT",
		"IFCDISTRIBUTIONFLOWELEMENTTYPE",
		"IFCDISTRIBUTIONPORT",
		"IFCDOCUMENTELECTRONICFORMAT",
		"IFCDOCUMENTINFORMATION",
		"IFCDOCUMENTINFORMATIONRELATIONSHIP",
		"IFCDOCUMENTREFERENCE",
		"IFCDOOR",
		"IFCDOORLININGPROPERTIES",
		"IFCDOORPANELPROPERTIES",
		"IFCDOORSTYLE",
		"IFCDRAUGHTINGCALLOUT",
		"IFCDRAUGHTINGCALLOUTRELATIONSHIP",
		"IFCDRAUGHTINGPREDEFINEDCOLOUR",
		"IFCDRAUGHTINGPREDEFINEDCURVEFONT",
		"IFCDRAUGHTINGPREDEFINEDTEXTFONT",
		"IFCDUCTFITTINGTYPE",
		"IFCDUCTSEGMENTTYPE",
		"IFCDUCTSILENCERTYPE",
		"IFCEDGE",
		"IFCEDGECURVE",
		"IFCEDGEFEATURE",
		"IFCEDGELOOP",
		"IFCELECTRICAPPLIANCETYPE",
		"IFCELECTRICDISTRIBUTIONPOINT",
		"IFCELECTRICFLOWSTORAGEDEVICETYPE",
		"IFCELECTRICGENERATORTYPE",
		"IFCELECTRICHEATERTYPE",
		"IFCELECTRICMOTORTYPE",
		"IFCELECTRICTIMECONTROLTYPE",
		"IFCELECTRICALBASEPROPERTIES",
		"IFCELECTRICALCIRCUIT",
		"IFCELECTRICALELEMENT",
		"IFCELEMENT",
		"IFCELEMENTASSEMBLY",
		"IFCELEMENTCOMPONENT",
		"IFCELEMENTCOMPONENTTYPE",
		"IFCELEMENTQUANTITY",
		"IFCELEMENTTYPE",
		"IFCELEMENTARYSURFACE",
		"IFCELLIPSE",
		"IFCELLIPSEPROFILEDEF",
		"IFCENERGYCONVERSIONDEVICE",
		"IFCENERGYCONVERSIONDEVICETYPE",
		"IFCENERGYPROPERTIES",
		"IFCENVIRONMENTALIMPACTVALUE",
		"IFCEQUIPMENTELEMENT",
		"IFCEQUIPMENTSTANDARD",
		"IFCEVAPORATIVECOOLERTYPE",
		"IFCEVAPORATORTYPE",
		"IFCEXTENDEDMATERIALPROPERTIES",
		"IFCEXTERNALREFERENCE",
		"IFCEXTERNALLYDEFINEDHATCHSTYLE",
		"IFCEXTERNALLYDEFINEDSURFACESTYLE",
		"IFCEXTERNALLYDEFINEDSYMBOL",
		"IFCEXTERNALLYDEFINEDTEXTFONT",
		"IFCEXTRUDEDAREASOLID",
		"IFCFACE",
		"IFCFACEBASEDSURFACEMODEL",
		"IFCFACEBOUND",
		"IFCFACEOUTERBOUND",
		"IFCFACESURFACE",
		"IFCFACETEDBREP",
		"IFCFACETEDBREPWITHVOIDS",
		"IFCFAILURECONNECTIONCONDITION",
		"IFCFANTYPE",
		"IFCFASTENER",
		"IFCFASTENERTYPE",
		"IFCFEATUREELEMENT",
		"IFCFEATUREELEMENTADDITION",
		"IFCFEATUREELEMENTSUBTRACTION",
		"IFCFILLAREASTYLE",
		"IFCFILLAREASTYLEHATCHING",
		"IFCFILLAREASTYLETILESYMBOLWITHSTYLE",
		"IFCFILLAREASTYLETILES",
		"IFCFILTERTYPE",
		"IFCFIRESUPPRESSIONTERMINALTYPE",
		"IFCFLOWCONTROLLER",
		"IFCFLOWCONTROLLERTYPE",
		"IFCFLOWFITTING",
		"IFCFLOWFITTINGTYPE",
		"IFCFLOWINSTRUMENTTYPE",
		"IFCFLOWMETERTYPE",
		"IFCFLOWMOVINGDEVICE",
		"IFCFLOWMOVINGDEVICETYPE",
		"IFCFLOWSEGMENT",
		"IFCFLOWSEGMENTTYPE",
		"IFCFLOWSTORAGEDEVICE",
		"IFCFLOWSTORAGEDEVICETYPE",
		"IFCFLOWTERMINAL",
		"IFCFLOWTERMINALTYPE",
		"IFCFLOWTREATMENTDEVICE",
		"IFCFLOWTREATMENTDEVICETYPE",
		"IFCFLUIDFLOWPROPERTIES",
		"IFCFOOTING",
		"IFCFUELPROPERTIES",
		"IFCFURNISHINGELEMENT",
		"IFCFURNISHINGELEMENTTYPE",
		"IFCFURNITURESTANDARD",
		"IFCFURNITURETYPE",
		"IFCGASTERMINALTYPE",
		"IFCGENERALMATERIALPROPERTIES",
		"IFCGENERALPROFILEPROPERTIES",
		"IFCGEOMETRICCURVESET",
		"IFCGEOMETRICREPRESENTATIONCONTEXT",
		"IFCGEOMETRICREPRESENTATIONITEM",
		"IFCGEOMETRICREPRESENTATIONSUBCONTEXT",
		"IFCGEOMETRICSET",
		"IFCGRID",
		"IFCGRIDAXIS",
		"IFCGRIDPLACEMENT",
		"IFCGROUP",
		"IFCHALFSPACESOLID",
		"IFCHEATEXCHANGERTYPE",
		"IFCHYGROMATERIALPROPERTIES",
		"IFCSHAPEPROFILEDEF",
		"IFCIMAGE",
		"IFCINVENTORY",
		"IFCIRREGULARTIMESERIES",
		"IFCIRREGULARTIMESERIESVALUE",
		"IFCJUNCTIONBOXTYPE",
		"IFCLSHAPEPROFILEDEF",
		"IFCLABORRESOURCE",
		"IFCLAMPTYPE",
		"IFCLIBRARYINFORMATION",
		"IFCLIBRARYREFERENCE",
		"IFCLIGHTDISTRIBUTIONDATA",
		"IFCLIGHTFIXTURETYPE",
		"IFCLIGHTINTENSITYDISTRIBUTION",
		"IFCLIGHTSOURCE",
		"IFCLIGHTSOURCEAMBIENT",
		"IFCLIGHTSOURCEDIRECTIONAL",
		"IFCLIGHTSOURCEGONIOMETRIC",
		"IFCLIGHTSOURCEPOSITIONAL",
		"IFCLIGHTSOURCESPOT",
		"IFCLINE",
		"IFCLINEARDIMENSION",
		"IFCLOCALPLACEMENT",
		"IFCLOCALTIME",
		"IFCLOOP",
		"IFCMANIFOLDSOLIDBREP",
		"IFCMAPPEDITEM",
		"IFCMATERIAL",
		"IFCMATERIALCLASSIFICATIONRELATIONSHIP",
		"IFCMATERIALDEFINITIONREPRESENTATION",
		"IFCMATERIALLAYER",
		"IFCMATERIALLAYERSET",
		"IFCMATERIALLAYERSETUSAGE",
		"IFCMATERIALLIST",
		"IFCMATERIALPROPERTIES",
		"IFCMEASUREWITHUNIT",
		"IFCMECHANICALCONCRETEMATERIALPROPERTIES",
		"IFCMECHANICALFASTENER",
		"IFCMECHANICALFASTENERTYPE",
		"IFCMECHANICALMATERIALPROPERTIES",
		"IFCMECHANICALSTEELMATERIALPROPERTIES",
		"IFCMEMBER",
		"IFCMEMBERTYPE",
		"IFCMETRIC",
		"IFCMONETARYUNIT",
		"IFCMOTORCONNECTIONTYPE",
		"IFCMOVE",
		"IFCNAMEDUNIT",
		"IFCOBJECT",
		"IFCOBJECTDEFINITION",
		"IFCOBJECTPLACEMENT",
		"IFCOBJECTIVE",
		"IFCOCCUPANT",
		"IFCOFFSETCURVE2D",
		"IFCOFFSETCURVE3D",
		"IFCONEDIRECTIONREPEATFACTOR",
		"IFCOPENSH",
		"IFCOPENINGELEMENT",
		"IFCOPTICALMATERIALPROPERTIES",
		"IFCORDERACTION",
		"IFCORGANIZATION",
		"IFCORGANIZATIONRELATIONSHIP",
		"IFCORIENTEDEDGE",
		"IFCOUTLETTYPE",
		"IFCOWNERHISTORY",
		"IFCPARAMETERIZEDPROFILEDEF",
		"IFCPATH",
		"IFCPERFORMANCEHISTORY",
		"IFCPERMEABLECOVERINGPROPERTIES",
		"IFCPERMIT",
		"IFCPERSON",
		"IFCPERSONANDORGANIZATION",
		"IFCPHYSICALCOMPLEXQUANTITY",
		"IFCPHYSICALQUANTITY",
		"IFCPHYSICALSIMPLEQUANTITY",
		"IFCPILE",
		"IFCPIPEFITTINGTYPE",
		"IFCPIPESEGMENTTYPE",
		"IFCPIXELTEXTURE",
		"IFCPLACEMENT",
		"IFCPLANARBOX",
		"IFCPLANAREXTENT",
		"IFCPLANE",
		"IFCPLATE",
		"IFCPLATETYPE",
		"IFCPOINT",
		"IFCPOINTONCURVE",
		"IFCPOINTONSURFACE",
		"IFCPOLYLOOP",
		"IFCPOLYGONALBOUNDEDHALFSPACE",
		"IFCPOLYLINE",
		"IFCPORT",
		"IFCPOSTALADDRESS",
		"IFCPREDEFINEDCOLOUR",
		"IFCPREDEFINEDCURVEFONT",
		"IFCPREDEFINEDDIMENSIONSYMBOL",
		"IFCPREDEFINEDITEM",
		"IFCPREDEFINEDPOINTMARKERSYMBOL",
		"IFCPREDEFINEDSYMBOL",
		"IFCPREDEFINEDTERMINATORSYMBOL",
		"IFCPREDEFINEDTEXTFONT",
		"IFCPRESENTATIONLAYERASSIGNMENT",
		"IFCPRESENTATIONLAYERWITHSTYLE",
		"IFCPRESENTATIONSTYLE",
		"IFCPRESENTATIONSTYLEASSIGNMENT",
		"IFCPROCEDURE",
		"IFCPROCESS",
		"IFCPRODUCT",
		"IFCPRODUCTDEFINITIONSHAPE",
		"IFCPRODUCTREPRESENTATION",
		"IFCPRODUCTSOFCOMBUSTIONPROPERTIES",
		"IFCPROFILEDEF",
		"IFCPROFILEPROPERTIES",
		"IFCPROJECT",
		"IFCPROJECTORDER",
		"IFCPROJECTORDERRECORD",
		"IFCPROJECTIONCURVE",
		"IFCPROJECTIONELEMENT",
		"IFCPROPERTY",
		"IFCPROPERTYBOUNDEDVALUE",
		"IFCPROPERTYCONSTRAINTRELATIONSHIP",
		"IFCPROPERTYDEFINITION",
		"IFCPROPERTYDEPENDENCYRELATIONSHIP",
		"IFCPROPERTYENUMERATEDVALUE",
		"IFCPROPERTYENUMERATION",
		"IFCPROPERTYLISTVALUE",
		"IFCPROPERTYREFERENCEVALUE",
		"IFCPROPERTYSET",
		"IFCPROPERTYSETDEFINITION",
		"IFCPROPERTYSINGLEVALUE",
		"IFCPROPERTYTABLEVALUE",
		"IFCPROTECTIVEDEVICETYPE",
		"IFCPROXY",
		"IFCPUMPTYPE",
		"IFCQUANTITYAREA",
		"IFCQUANTITYCOUNT",
		"IFCQUANTITYLENGTH",
		"IFCQUANTITYTIME",
		"IFCQUANTITYVOLUME",
		"IFCQUANTITYWEIGHT",
		"IFCRADIUSDIMENSION",
		"IFCRAILING",
		"IFCRAILINGTYPE",
		"IFCRAMP",
		"IFCRAMPFLIGHT",
		"IFCRAMPFLIGHTTYPE",
		"IFCSTRUCTURALACTION",
		"IFCSTRUCTURALACTIVITY",
		"IFCSTRUCTURALANALYSISMODEL",
		"IFCSTRUCTURALCONNECTION",
		"IFCSTRUCTURALCONNECTIONCONDITION",
		"IFCSTRUCTURALCURVECONNECTION",
		"IFCSTRUCTURALCURVEMEMBER",
		"IFCSTRUCTURALCURVEMEMBERVARYING",
		"IFCSTRUCTURALITEM",
		"IFCSTRUCTURALLINEARACTION",
		"IFCSTRUCTURALLINEARACTIONVARYING",
		"IFCSTRUCTURALLOAD",
		"IFCSTRUCTURALLOADGROUP",
		"IFCSTRUCTURALLOADLINEARFORCE",
		"IFCSTRUCTURALLOADPLANARFORCE",
		"IFCSTRUCTURALLOADSINGLEDISPLACEMENT",
		"IFCSTRUCTURALLOADSINGLEDISPLACEMENTDISTORTION",
		"IFCSTRUCTURALLOADSINGLEFORCE",
		"IFCSTRUCTURALLOADSINGLEFORCEWARPING",
		"IFCSTRUCTURALLOADTEMPERATURE",
		"IFCSTRUCTURALMEMBER",
		"IFCSTRUCTURALPLANARACTION",
		"IFCSTRUCTURALPOINTACTION",
		"IFCSTRUCTURALPOINTCONNECTION",
		"IFCSTRUCTURALPOINTREACTION",
		"IFCSTRUCTURALREACTION",
		"IFCSTRUCTURALRESULTGROUP",
		"IFCSTRUCTURALSTEELPROFILEPROPERTIES",
		"IFCSTRUCTURALSURFACECONNECTION",
		"IFCSTRUCTURALSURFACEMEMBER",
		"IFCSTRUCTURALSURFACEMEMBERVARYING",
		"IFCSTRUCTUREDDIMENSIONCALLOUT",
		"IFCSUBCONTRACTRESOURCE",
		"IFCSUBEDGE",
		"IFCSURFACE",
		"IFCSURFACECURVESWEPTAREASOLID",
		"IFCSURFACEOFLINEAREXTRUSION",
		"IFCSURFACEOFREVOLUTION",
		"IFCSURFACESTYLE",
		"IFCSURFACESTYLELIGHTING",
		"IFCSURFACESTYLEREFRACTION",
		"IFCSURFACESTYLERENDERING",
		"IFCSURFACESTYLESHADING",
		"IFCSURFACESTYLEWITHTEXTURES",
		"IFCSWEPTAREASOLID",
		"IFCSWITCHINGDEVICETYPE",
		"IFCSYMBOLSTYLE",
		"IFCSYMBOLSTYLESELECT",
		"IFCSYSTEM",
		"IFCSYSTEMFURNITUREELEMENTTYPE",
		"IFCTSHAPEPROFILEDEF",
		"IFCTABLE",
		"IFCTABLEROW",
		"IFCTANKTYPE",
		"IFCTASK",
		"IFCTASKTIME",
		"IFCTAXONOMICCLASSIFICATION",
		"IFCTAXONOMICCLASSIFICATIONRELATIONSHIP",
		"IFCTEXTSTYLE",
		"IFCTHEATRE",
		"IFCTHEATRETYPE",
		"IFCTHERMALMATERIALPROPERTIES",
		"IFCTHERMOPHYSICALMATERIALPROPERTIES",
		"IFCTHERMOPHYSICALPROPERTYSET",
		"IFCTHERMOPHYSICALPROPERTYSETUSAGE",
		"IFCTHERMOPHYSICALSIMPLEPROPERTY",
		"IFCTIMEPERIOD",
		"IFCTIMESERIES",
		"IFCTIMESERIESGROUP",
		"IFCTIMESERIESREFERENCE",
		"IFCTIMESERIESSCHEDULE",
		"IFCTIMESERIESVALUE",
		"IFCTOPOLOGICALREPRESENTATIONITEM",
		"IFCTRANSFORMERRESOURCE",
		"IFCTRANSPORTELEMENT",
		"IFCTRANSPORTELEMENTTYPE",
		"IFCTRANSPORTELEMENTSTATICDEFLECTION",
		"IFCTRANSPORTELEMENTSTATICREACTION",
		"IFCTRAPEZIUMPROFILEDEF",
		"IFCTRIMMEDCURVE",
		"IFCTUBEBUNDLETYPE",
		"IFCTWIRLGENERATORTYPE",
		"IFCUNITASSIGNMENT",
		"IFCUNITARYCONTROLELEMENT",
		"IFCUNITARYCONTROLELEMENTTYPE",
		"IFCUNITARYEQUIPMENT",
		"IFCUNITARYEQUIPMENTTYPE",
		"IFCUNITARYPRODUCT",
		"IFCUNITARYPRODUCTTYPE",
		"IFCVALVE",
		"IFCVALVETYPE",
		"IFCVECTOR",
		"IFCVERTEX",
		"IFCVERTEXLOOP",
		"IFCVERTEXPOINT",
		"IFCVIBRATORYPE",
		"IFCVIRTUALELEMENT",
		"IFCVIRTUALELEMENTTYPE",
		"IFCVOIDINGFEATURE",
		"IFCVOIDINGFEATURETYPE",
		"IFCVOLUMEBEAM",
		"IFCVOLUMEBEAMTYPE",
		"IFCWALL",
		"IFCWALLSTANDARDCASE",
		"IFCWALLTYPE",
		"IFCWARPINGCONSTANTMEASURE",
		"IFCWARPINGMOMENTOFINERTIA_MEASURE",
		"IFCWINDOW",
		"IFCWINDOWLININGPROPERTIES",
		"IFCWINDOWPANELPROPERTIES",
		"IFCWINDOWSTYLE",
		"IFCZSHAPEPROFILEDEF"
	};
};