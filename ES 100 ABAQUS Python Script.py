# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
from abaqusConstants import *
import numpy as np
from odbAccess import *

os.chdir(r"C:/Users/19178/Documents/FINAL ABAQUS YEAA2")

# Length and Width of pouch
pouch_length_values = [0.3302, 0.2794, 0.2286, 0.1778]  # m
pouch_width_values  = [0.3048, 0.2540, 0.2032, 0.1524] # m


toppan_stress_values = []
foil_stress_values = []

for pouch_length in pouch_length_values:
    for pouch_width in pouch_width_values:
        Mdb()
        m = mdb.models['Model-1']

        # Variables
        seam_width = 0.00635 # m
        curve_radius = 0.01 # m
        angle = -20 # degrees
        density = 500
        material_thickness = 0.000105
        time_period = 0.2
        velocity = -3.235 # 21 inch drop
        mass = 185*(pouch_length*pouch_width)**1.76
        pouch_thickness = mass / (pouch_length*pouch_width*1000) # m 

        # Create 3d pouch
        m.ConstrainedSketch(name='__profile__', sheetSize=200.0)
        m.sketches['__profile__'].rectangle(point1=(-pouch_width/2, 
            -pouch_length/2), point2=(pouch_width/2, 
            pouch_length/2))
        m.Part(dimensionality=THREE_D, name='half pouch', type=
            DEFORMABLE_BODY)
        m.parts['half pouch'].BaseSolidExtrude(depth=pouch_thickness/2, 
            draftAngle=angle, sketch=m.sketches['__profile__'])
        del m.sketches['__profile__']

        # Create Instance
        m.rootAssembly.Instance(dependent=ON, name='half pouch-1', 
            part=m.parts['half pouch'])
        m.rootAssembly.Instance(dependent=ON, name='half pouch-2', 
            part=m.parts['half pouch'])

        # Rotate and Merge
        m.rootAssembly.rotate(angle=180.0, axisDirection=(0.0, 1.0, 
            0.0), axisPoint=(0.0, 0.0, 0.0), instanceList=('half pouch-2', ))
        m.rootAssembly.InstanceFromBooleanMerge(domain=GEOMETRY, 
            instances=(m.rootAssembly.instances['half pouch-1'], 
            m.rootAssembly.instances['half pouch-2']), name='pouch'
            , originalInstances=SUPPRESS)

        # Make Hollow
        m.parts['pouch'].RemoveCells(cellList=
            m.parts['pouch'].cells.getSequenceFromMask(('[#1 ]', ), 
            ))
        m.rootAssembly.regenerate()
            
        # Make Seams
        m.ConstrainedSketch(name='__profile__', sheetSize=1.0)
        m.sketches['__profile__'].rectangle(point1=(-pouch_width/2 - seam_width, 
            -pouch_length/2 - seam_width), point2=(pouch_width/2 + seam_width, 
            pouch_length/2 + seam_width))  
        sketch = m.sketches['__profile__']
        curve1 = sketch.geometry[3]  # Top horizontal line
        curve2 = sketch.geometry[2]  # Left vertical line
        near_point1 = (-pouch_width/2 - seam_width + 0.001, pouch_length/2 + seam_width - 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point1, nearPoint2=near_point1, radius=0.01)
        curve1 = sketch.geometry[2]  # Left vertical line
        curve2 = sketch.geometry[5]  # Bottom horizontal line
        near_point2 = (-pouch_width/2 - seam_width + 0.001, -pouch_length/2 - seam_width + 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point2, nearPoint2=near_point2, radius=0.01)
        curve1 = sketch.geometry[4]  # Right vertical line
        curve2 = sketch.geometry[5]  # Bottom horizontal line
        near_point3 = (pouch_width/2 + seam_width - 0.001, -pouch_length/2 - seam_width + 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point3, nearPoint2=near_point3, radius=0.01)
        curve1 = sketch.geometry[3]  # Top horizontal line
        curve2 = sketch.geometry[4]  # Right vertical line
        near_point4 = (pouch_width/2 + seam_width - 0.001, pouch_length/2 + seam_width - 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point4, nearPoint2=near_point4, radius=0.01)
        m.Part(dimensionality=THREE_D, name='seams', type=
            DEFORMABLE_BODY)
        m.parts['seams'].BaseShell(sketch=
            m.sketches['__profile__'])
        del m.sketches['__profile__']

        # Put Seams on Pouch
        m.rootAssembly.Instance(dependent=ON, name='seams-1', part=
            m.parts['seams'])
        m.rootAssembly.InstanceFromBooleanMerge(domain=GEOMETRY, 
            instances=(m.rootAssembly.instances['pouch-1'], 
            m.rootAssembly.instances['seams-1']), name=
            'pouch with seams', originalInstances=SUPPRESS)

        # Make Hollow
        m.parts['pouch with seams'].RemoveFaces(deleteCells=False, 
            faceList=
            m.parts['pouch with seams'].faces.getSequenceFromMask(
            mask=('[#2 ]', ), ))

        # Mesh
        m.parts['pouch with seams'].seedPart(deviationFactor=0.1, 
            minSizeFactor=0.1, size=0.01)
        m.parts['pouch with seams'].setMeshControls(algorithm=
            MEDIAL_AXIS, elemShape=QUAD, regions=
            m.parts['pouch with seams'].faces.getSequenceFromMask((
            '[#7ff ]', ), ))
        m.parts['pouch with seams'].generateMesh()

        # Create Intertia for Simulating Mass
        m.parts['pouch with seams'].Set(faces=
            m.parts['pouch with seams'].faces.getSequenceFromMask((
            '[#7ff ]', ), ), name='pouch')
        m.parts['pouch with seams'].engineeringFeatures.NonstructuralMass(
            distribution=MASS_PROPORTIONAL, magnitude=mass, name='Inertia-1', region=
            m.parts['pouch with seams'].sets['pouch'], units=
            TOTAL_MASS)

        # Create Plate
        m.ConstrainedSketch(name='__profile__', sheetSize=0.5)
        m.sketches['__profile__'].Line(point1=(-0.5, 0.0), point2=(
            0.5, 0.0))
        m.sketches['__profile__'].HorizontalConstraint(
            addUndoState=False, entity=
            m.sketches['__profile__'].geometry[2])
        m.Part(dimensionality=THREE_D, name='Plate', type=
            ANALYTIC_RIGID_SURFACE)
        m.parts['Plate'].AnalyticRigidSurfExtrude(depth=1,
            sketch=m.sketches['__profile__'])
        del m.sketches['__profile__']
        m.parts['Plate'].ReferencePoint(point=
            m.parts['Plate'].vertices[2])
        m.parts['Plate'].Surface(name='Surf-plate', side1Faces=
            m.parts['Plate'].faces.getSequenceFromMask(('[#1 ]',
            ), ))
        m.rootAssembly.Instance(dependent=ON, name='plate-1', part=
            m.parts['Plate'])

        # Move pouch up for drop, making sure it lands on short side
        if pouch_width > pouch_length:
        # Rotate the pouch so that its short dimension (originally the width along X) becomes aligned with Y
            m.rootAssembly.rotate(instanceList=('pouch with seams-1', ),
            angle=90.0,
            axisDirection=(0.0, 0.0, 1.0),
            axisPoint=(0.0, 0.0, 0.0))
            drop_offset = pouch_width/2 + seam_width + 0.0001
        else:
        # No rotation needed if the Y-directed dimension is already the short side
            drop_offset = pouch_length/2 + seam_width + 0.0001
        #Translate upward using the proper short side dimension
        m.rootAssembly.translate(instanceList=('pouch with seams-1', ),
        vector=(0.0, drop_offset, 0.0))

        # Boundary Conditions
        m.EncastreBC(createStepName='Initial', localCsys=None,
            name='BC-1', region=Region(referencePoints=(
            m.rootAssembly.instances['plate-1'].referencePoints[2],
            )))

        # Create Step
        m.ExplicitDynamicsStep(improvedDtMethod=ON, 
            name='Step-1', previous='Initial', scaleFactor=
            1.0)
        m.steps['Step-1'].setValues(improvedDtMethod=ON, 
            timePeriod=time_period)

        # Define Fluid Cavity
        m.FluidCavityProperty(bulkModulusTable=((2100000000.0, ), )
            , expansionTable=((1.0, ), ), fluidDensity=1000.0, name='IntProp-1', 
            useBulkModulus=True, useExpansion=True)
        m.rootAssembly.ReferencePoint(point=(0.0, pouch_length/2, 
            0.0))
        m.rootAssembly.regenerate()
        m.rootAssembly.Set(name='Ref Point', referencePoints=(
            m.rootAssembly.referencePoints[14], ))
        m.rootAssembly.Surface(name='Cavity', side2Faces=
            m.rootAssembly.instances['pouch with seams-1'].faces.getSequenceFromMask(
            ('[#7fe ]', ), ))
        m.FluidCavity(cavityPoint=
            m.rootAssembly.sets['Ref Point'], cavitySurface=
            m.rootAssembly.surfaces['Cavity'], createStepName=
            'Initial', interactionProperty='IntProp-1', name='Int-1')

        # Initialize Temperature
        m.Temperature(createStepName='Step-1', 
            crossSectionDistribution=CONSTANT_THROUGH_THICKNESS, distributionType=
            UNIFORM, magnitudes=(0.0, ), name='Predefined Field-1', region=
            m.rootAssembly.sets['Ref Point'])

        # Define Contact 
        m.ContactProperty('IntProp-2')
        m.interactionProperties['IntProp-2'].NormalBehavior(
            allowSeparation=ON, constraintEnforcementMethod=DEFAULT, 
            pressureOverclosure=HARD)
        m.interactionProperties['IntProp-2'].TangentialBehavior(
            formulation=FRICTIONLESS)
        m.ContactExp(createStepName='Step-1', name='Int-2')
        m.interactions['Int-2'].includedPairs.setValuesInStep(
            stepName='Step-1', useAllstar=ON)
        m.interactions['Int-2'].contactPropertyAssignments.appendInStep(
            assignments=((GLOBAL, SELF, 'IntProp-2'), ), stepName='Step-1')

        # Velocity and Gravity
        m.rootAssembly.Set(faces=
            m.rootAssembly.instances['pouch with seams-1'].faces.getSequenceFromMask(
            ('[#5ef ]', ), ), name='Pouch', referencePoints=(
            m.rootAssembly.referencePoints[14], ))
        m.Velocity(distributionType=MAGNITUDE, field='', name=
            'Predefined Field-3', omega=0.0, region=
            m.rootAssembly.sets['Pouch'], velocity2=velocity)
        m.Gravity(comp2=-9.81, createStepName='Step-1', 
            distributionType=UNIFORM, field='', name='Load-1')

        # Material Properties, Create and Assign Section
        m.Material(name='Toppan')
        m.materials['Toppan'].Density(table=((density, ), ))
        m.materials['Toppan'].Damping(alpha=1.0)
        m.materials['Toppan'].Elastic(table=((1516000000.0, 0.3), 
            ))
        m.materials['Toppan'].Plastic(table=((31000000, 0.0), (
            33600000.0, 0.01), (34900000.0, 0.0242)))
        m.HomogeneousShellSection(idealization=NO_IDEALIZATION, 
            integrationRule=SIMPSON, material='Toppan', name='Section-1', 
            nodalThicknessField='', numIntPts=5, poissonDefinition=DEFAULT, 
            preIntegrate=OFF, temperature=GRADIENT, thickness=material_thickness, thicknessField='', 
            thicknessModulus=None, thicknessType=UNIFORM, useDensity=OFF)
        m.parts['pouch with seams'].SectionAssignment(offset=0.0, 
            offsetField='', offsetType=MIDDLE_SURFACE, region=
            m.parts['pouch with seams'].sets['pouch'], sectionName=
            'Section-1', thicknessAssignment=FROM_SECTION)

        # Request Max Mises Field Output
        m.fieldOutputRequests['F-Output-1'].setValues(variables=(
            'S', 'MISES', 'MISESMAX', 'E', 'U'))

        # Job 
        METERS_TO_INCHES = 39.3701
        length_in_inches = round(pouch_length * METERS_TO_INCHES)
        width_in_inches = round(pouch_width * METERS_TO_INCHES)
        job_name = 'Toppan_'+str(int(length_in_inches))+'x'+str(int(width_in_inches))+'_pouch'
        m.rootAssembly.regenerate()
        mdb.Job(activateLoadBalancing=False, atTime=None, contactPrint=OFF, 
            description='', echoPrint=OFF, explicitPrecision=DOUBLE_PLUS_PACK, historyPrint=OFF, 
            memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF, 
            multiprocessingMode=DEFAULT, name=job_name, nodalOutputPrecision=SINGLE, 
            numCpus=8, numDomains=8, parallelizationMethodExplicit=DOMAIN, queue=None, 
            resultsFormat=ODB, scratch='', type=ANALYSIS, userSubroutine='', waitHours=
            0, waitMinutes=0)
        mdb.jobs[job_name].submit(consistencyChecking=OFF)
        mdb.jobs[job_name].waitForCompletion()

        # Post Processing
        odb_path = job_name+'.odb'
        odb = openOdb(path=odb_path)

        step = odb.steps['Step-1']

        max_mises_stress = 0.0 

        for frame in step.frames:
            stress_field = frame.fieldOutputs['S']
            # Get the Mises stress invariant
            mises_field = stress_field.getScalarField(invariant=MISES)
            # Loop over all stress values
            for stress_value in mises_field.values:
                if stress_value.data > max_mises_stress:
                    max_mises_stress = stress_value.data

        toppan_stress_values.append([length_in_inches,width_in_inches,max_mises_stress])

        odb.close()

np.savetxt('Toppan_Max_Stress_Values',toppan_stress_values)

# FOIL POUCH NOW
for pouch_length in pouch_length_values:
    for pouch_width in pouch_width_values:
        Mdb()
        m = mdb.models['Model-1']

        # Variables
        seam_width = 0.00635 # m
        curve_radius = 0.01 # m
        angle = -20 # degrees
        density = 500
        material_thickness = 0.000105
        time_period = 0.2
        velocity = -3.235 # 21 inch drop
        mass = 185*(pouch_length*pouch_width)**1.76
        pouch_thickness = mass / (pouch_length*pouch_width*1000) # m 

        # Create 3d pouch
        m.ConstrainedSketch(name='__profile__', sheetSize=200.0)
        m.sketches['__profile__'].rectangle(point1=(-pouch_width/2, 
            -pouch_length/2), point2=(pouch_width/2, 
            pouch_length/2))
        m.Part(dimensionality=THREE_D, name='half pouch', type=
            DEFORMABLE_BODY)
        m.parts['half pouch'].BaseSolidExtrude(depth=pouch_thickness/2, 
            draftAngle=angle, sketch=m.sketches['__profile__'])
        del m.sketches['__profile__']

        # Create Instance
        m.rootAssembly.Instance(dependent=ON, name='half pouch-1', 
            part=m.parts['half pouch'])
        m.rootAssembly.Instance(dependent=ON, name='half pouch-2', 
            part=m.parts['half pouch'])

        # Rotate and Merge
        m.rootAssembly.rotate(angle=180.0, axisDirection=(0.0, 1.0, 
            0.0), axisPoint=(0.0, 0.0, 0.0), instanceList=('half pouch-2', ))
        m.rootAssembly.InstanceFromBooleanMerge(domain=GEOMETRY, 
            instances=(m.rootAssembly.instances['half pouch-1'], 
            m.rootAssembly.instances['half pouch-2']), name='pouch'
            , originalInstances=SUPPRESS)

        # Make Hollow
        m.parts['pouch'].RemoveCells(cellList=
            m.parts['pouch'].cells.getSequenceFromMask(('[#1 ]', ), 
            ))
        m.rootAssembly.regenerate()
            
        # Make Seams
        m.ConstrainedSketch(name='__profile__', sheetSize=1.0)
        m.sketches['__profile__'].rectangle(point1=(-pouch_width/2 - seam_width, 
            -pouch_length/2 - seam_width), point2=(pouch_width/2 + seam_width, 
            pouch_length/2 + seam_width))  
        sketch = m.sketches['__profile__']
        curve1 = sketch.geometry[3]  # Top horizontal line
        curve2 = sketch.geometry[2]  # Left vertical line
        near_point1 = (-pouch_width/2 - seam_width + 0.001, pouch_length/2 + seam_width - 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point1, nearPoint2=near_point1, radius=0.01)
        curve1 = sketch.geometry[2]  # Left vertical line
        curve2 = sketch.geometry[5]  # Bottom horizontal line
        near_point2 = (-pouch_width/2 - seam_width + 0.001, -pouch_length/2 - seam_width + 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point2, nearPoint2=near_point2, radius=0.01)
        curve1 = sketch.geometry[4]  # Right vertical line
        curve2 = sketch.geometry[5]  # Bottom horizontal line
        near_point3 = (pouch_width/2 + seam_width - 0.001, -pouch_length/2 - seam_width + 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point3, nearPoint2=near_point3, radius=0.01)
        curve1 = sketch.geometry[3]  # Top horizontal line
        curve2 = sketch.geometry[4]  # Right vertical line
        near_point4 = (pouch_width/2 + seam_width - 0.001, pouch_length/2 + seam_width - 0.001)
        sketch.FilletByRadius(curve1=curve1, curve2=curve2, nearPoint1=near_point4, nearPoint2=near_point4, radius=0.01)
        m.Part(dimensionality=THREE_D, name='seams', type=
            DEFORMABLE_BODY)
        m.parts['seams'].BaseShell(sketch=
            m.sketches['__profile__'])
        del m.sketches['__profile__']

        # Put Seams on Pouch
        m.rootAssembly.Instance(dependent=ON, name='seams-1', part=
            m.parts['seams'])
        m.rootAssembly.InstanceFromBooleanMerge(domain=GEOMETRY, 
            instances=(m.rootAssembly.instances['pouch-1'], 
            m.rootAssembly.instances['seams-1']), name=
            'pouch with seams', originalInstances=SUPPRESS)

        # Make Hollow
        m.parts['pouch with seams'].RemoveFaces(deleteCells=False, 
            faceList=
            m.parts['pouch with seams'].faces.getSequenceFromMask(
            mask=('[#2 ]', ), ))

        # Mesh
        m.parts['pouch with seams'].seedPart(deviationFactor=0.1, 
            minSizeFactor=0.1, size=0.01)
        m.parts['pouch with seams'].setMeshControls(algorithm=
            MEDIAL_AXIS, elemShape=QUAD, regions=
            m.parts['pouch with seams'].faces.getSequenceFromMask((
            '[#7ff ]', ), ))
        m.parts['pouch with seams'].generateMesh()

        # Create Intertia for Simulating Mass
        m.parts['pouch with seams'].Set(faces=
            m.parts['pouch with seams'].faces.getSequenceFromMask((
            '[#7ff ]', ), ), name='pouch')
        mdb.models['Model-1'].parts['pouch with seams'].engineeringFeatures.NonstructuralMass(
            distribution=MASS_PROPORTIONAL, magnitude=mass, name='Inertia-1', region=
            mdb.models['Model-1'].parts['pouch with seams'].sets['pouch'], units=
            TOTAL_MASS)

        # Create Plate
        m.ConstrainedSketch(name='__profile__', sheetSize=0.5)
        m.sketches['__profile__'].Line(point1=(-0.5, 0.0), point2=(
            0.5, 0.0))
        m.sketches['__profile__'].HorizontalConstraint(
            addUndoState=False, entity=
            m.sketches['__profile__'].geometry[2])
        m.Part(dimensionality=THREE_D, name='Plate', type=
            ANALYTIC_RIGID_SURFACE)
        m.parts['Plate'].AnalyticRigidSurfExtrude(depth=1,
            sketch=m.sketches['__profile__'])
        del m.sketches['__profile__']
        m.parts['Plate'].ReferencePoint(point=
            m.parts['Plate'].vertices[2])
        m.parts['Plate'].Surface(name='Surf-plate', side1Faces=
            m.parts['Plate'].faces.getSequenceFromMask(('[#1 ]',
            ), ))
        m.rootAssembly.Instance(dependent=ON, name='plate-1', part=
            m.parts['Plate'])

        # Move pouch up for drop, making sure it lands on short side
        if pouch_width > pouch_length:
        # Rotate the pouch so that its short dimension (originally the width along X) becomes aligned with Y
            m.rootAssembly.rotate(instanceList=('pouch with seams-1', ),
            angle=90.0,
            axisDirection=(0.0, 0.0, 1.0),
            axisPoint=(0.0, 0.0, 0.0))
            drop_offset = pouch_width/2 + seam_width + 0.0001
        else:
        # No rotation needed if the Y-directed dimension is already the short side
            drop_offset = pouch_length/2 + seam_width + 0.0001
        #Translate upward using the proper short side dimension
        m.rootAssembly.translate(instanceList=('pouch with seams-1', ),
        vector=(0.0, drop_offset, 0.0))

        # Boundary Conditions
        m.EncastreBC(createStepName='Initial', localCsys=None,
            name='BC-1', region=Region(referencePoints=(
            m.rootAssembly.instances['plate-1'].referencePoints[2],
            )))

        # Create Step
        m.ExplicitDynamicsStep(improvedDtMethod=ON, 
            name='Step-1', previous='Initial', scaleFactor=
            1.0)
        m.steps['Step-1'].setValues(improvedDtMethod=ON, 
            timePeriod=time_period)

        # Define Fluid Cavity
        m.FluidCavityProperty(bulkModulusTable=((2100000000.0, ), )
            , expansionTable=((1.0, ), ), fluidDensity=1000.0, name='IntProp-1', 
            useBulkModulus=True, useExpansion=True)
        m.rootAssembly.ReferencePoint(point=(0.0, pouch_length/2, 
            0.0))
        m.rootAssembly.regenerate()
        m.rootAssembly.Set(name='Ref Point', referencePoints=(
            m.rootAssembly.referencePoints[14], ))
        m.rootAssembly.Surface(name='Cavity', side2Faces=
            m.rootAssembly.instances['pouch with seams-1'].faces.getSequenceFromMask(
            ('[#7fe ]', ), ))
        m.FluidCavity(cavityPoint=
            m.rootAssembly.sets['Ref Point'], cavitySurface=
            m.rootAssembly.surfaces['Cavity'], createStepName=
            'Initial', interactionProperty='IntProp-1', name='Int-1')

        # Initialize Temperature
        m.Temperature(createStepName='Step-1', 
            crossSectionDistribution=CONSTANT_THROUGH_THICKNESS, distributionType=
            UNIFORM, magnitudes=(0.0, ), name='Predefined Field-1', region=
            m.rootAssembly.sets['Ref Point'])

        # Define Contact 
        m.ContactProperty('IntProp-2')
        m.interactionProperties['IntProp-2'].NormalBehavior(
            allowSeparation=ON, constraintEnforcementMethod=DEFAULT, 
            pressureOverclosure=HARD)
        m.interactionProperties['IntProp-2'].TangentialBehavior(
            formulation=FRICTIONLESS)
        m.ContactExp(createStepName='Step-1', name='Int-2')
        m.interactions['Int-2'].includedPairs.setValuesInStep(
            stepName='Step-1', useAllstar=ON)
        m.interactions['Int-2'].contactPropertyAssignments.appendInStep(
            assignments=((GLOBAL, SELF, 'IntProp-2'), ), stepName='Step-1')

        # Velocity and Gravity
        m.rootAssembly.Set(faces=
            m.rootAssembly.instances['pouch with seams-1'].faces.getSequenceFromMask(
            ('[#5ef ]', ), ), name='Pouch', referencePoints=(
            m.rootAssembly.referencePoints[14], ))
        m.Velocity(distributionType=MAGNITUDE, field='', name=
            'Predefined Field-3', omega=0.0, region=
            m.rootAssembly.sets['Pouch'], velocity2=velocity)
        m.Gravity(comp2=-9.81, createStepName='Step-1', 
            distributionType=UNIFORM, field='', name='Load-1')

        # Material Properties, Create and Assign Section
        m.Material(name='Foil')
        m.materials['Foil'].Density(table=((density, ), ))
        m.materials['Foil'].Damping(alpha=1.0)
        m.materials['Foil'].Elastic(table=((1534231000.0, 0.3), 
            ))
        m.materials['Foil'].Plastic(table=((23400000.0, 0.0), (
            35100000.0, 0.1), (56880000.0, 0.4), (64800000.0, 0.55)))
        m.HomogeneousShellSection(idealization=NO_IDEALIZATION, 
            integrationRule=SIMPSON, material='Foil', name='Section-1', 
            nodalThicknessField='', numIntPts=5, poissonDefinition=DEFAULT, 
            preIntegrate=OFF, temperature=GRADIENT, thickness=material_thickness, thicknessField='', 
            thicknessModulus=None, thicknessType=UNIFORM, useDensity=OFF)
        m.parts['pouch with seams'].SectionAssignment(offset=0.0, 
            offsetField='', offsetType=MIDDLE_SURFACE, region=
            m.parts['pouch with seams'].sets['pouch'], sectionName=
            'Section-1', thicknessAssignment=FROM_SECTION)

        # Request Max Mises Field Output
        m.fieldOutputRequests['F-Output-1'].setValues(variables=(
            'S', 'MISES', 'MISESMAX', 'E', 'U'))

        # Job 
        METERS_TO_INCHES = 39.3701
        length_in_inches = round(pouch_length * METERS_TO_INCHES)
        width_in_inches = round(pouch_width * METERS_TO_INCHES)
        job_name = 'Foil_'+str(int(length_in_inches))+'x'+str(int(width_in_inches))+'_pouch'
        m.rootAssembly.regenerate()
        mdb.Job(activateLoadBalancing=False, atTime=None, contactPrint=OFF, 
            description='', echoPrint=OFF, explicitPrecision=DOUBLE_PLUS_PACK, historyPrint=OFF, 
            memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF, 
            multiprocessingMode=DEFAULT, name=job_name, nodalOutputPrecision=SINGLE, 
            numCpus=8, numDomains=8, parallelizationMethodExplicit=DOMAIN, queue=None, 
            resultsFormat=ODB, scratch='', type=ANALYSIS, userSubroutine='', waitHours=
            0, waitMinutes=0)
        mdb.jobs[job_name].submit(consistencyChecking=OFF)
        mdb.jobs[job_name].waitForCompletion()

        # Post Processing
        odb_path = job_name+'.odb'
        odb = openOdb(path=odb_path)

        step = odb.steps['Step-1']

        max_mises_stress = 0.0 

        for frame in step.frames:
            stress_field = frame.fieldOutputs['S']
            # Get the Mises stress invariant
            mises_field = stress_field.getScalarField(invariant=MISES)
            # Loop over all stress values
            for stress_value in mises_field.values:
                if stress_value.data > max_mises_stress:
                    max_mises_stress = stress_value.data

        foil_stress_values.append([length_in_inches,width_in_inches,max_mises_stress])

        odb.close()

np.savetxt('Foil_Max_Stress_Values',foil_stress_values)

