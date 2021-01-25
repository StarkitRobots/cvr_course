# Vision    

All the detection relies on the usage of the Blob analysis from the OpenMV SDK

Classes:

Detectors:
    -Detector - basic visualization, placeholders for common methods

    -ColoredObjectDetector [inherited from Detector] - blob analysis,
        particular filtration methods
    
    -SurroundedObjectDetector [inherited from ColoredObjectDetector] - 
        takes surrounding pixels of the object into account to check if
        the detected object has proper background

Containers/other:
    -Vision - container for the detectors
    
    -VisionPostprocessing - mutual spatial relations of the detected objects
