# Hive observer firmware - beehive sensor with visual capabilities

This is a beehive sensor based on the sensebox_eye project. that aims to detect data that is related to the current activity and health of a beehive.

## Build
uses ESP-IDF v5.5.4

```
. /home/dings/.espressif/v5.5.4/esp-idf/export.sh #activate v5.5.4 environment
cd hive_observer_firmware

# use menuconfig to add wifi access point under "Example Connection Configuration"
# Add post-destination url to POST_URL in main.cpp

# build, flash, monitor
idf.py build
idf.py flash 
idf.py monitor
```

## Data
The sensebox will send recorded json-formatted data via POST-request to the specified POST_URL via wifi.
Example of the expected JSON:
```
{
    "timestamp":58784,
    "inferences":["crawling","bg","bg","flying"],
    "temperatures":[24.375000,24.750000]
}
```
timestamp:      the sensebox's uptime in milliseconds.
inferences:     list of elements of 3 classes: "bg", "crawling", "flying".
temperatures:   list of temperature values collected in sequence by all connected sensors.

## Bee Detection via Machine Vision on the ESP32s3
The machine vision component of this project uses a combination of classic image processing methods and a convolutional neural network to count bees of varying activity in an RGB combination of 3 concurrent frames.
The detailed approach is as follows:

1. Frame capture via the ESP-camera
    - capture 3 concurrent QVGA black-and-white frames using the ESP camera.
    - combine these 3 frames into the RGB-channels of a single RGB-frame.
    - in this RGB frame, saturation is an indicator for motion.
    - the RGB frame is passed to the `bee_vision::classify_frame()` method to get a list of inferences from the frame.
2. Frame preprocessing. selection of inference candidates
    - from the RGB frames, a saturation map is created with `max(r,g,b) - min(r,g,b})` for each pixel (`bee_vision::candidate_crops()`)
    - a 3x3 gaussian blur is applied to the saturation map. (`bee_vision::candidate_crops()`)
    - on the blurred saturation map, pixels that exceed a predefined saturation threshold are considered. (`bee_vision::candidate_crops()`)
    - on the threshold exceeding pixels, a connected-component labeling algorithm is run. (`bee_vision::candidate_crops()`)
    - of the resulting blobs, blobs with an area between 16 and 800px are selected as candidates. (`bee_vision::candidate_crops()`)
    - of each candidates, 32 x 32px crops are passed to inference. (`bee_vision::classify_frame()`)
3. Inference
    - the 32x32 crops are converted to tensors. (`bee_vision::classify_crop()`)
    - a pretrained convolutional neural network performs an inference on each crop in sequence.
    (`bee_vision::classify_crop()`)
    - each crop is classified as either flying, crawling or background based on the highest confidence returned.
    (`bee_vision::classify_crop()`)
    - the inferences of each crop are returned as a vector.
    (`bee_vision::classify_crop()`)

A modification may also allow for more info, such as location within the frame to be passed as a result from the inference. From this, more granular activity observations could be made. This is not currently implemented.