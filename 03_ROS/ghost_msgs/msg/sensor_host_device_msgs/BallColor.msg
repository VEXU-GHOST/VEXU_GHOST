# Classified ball colour from a colour sensor.
uint8 NONE=0      # nothing / too dark in front of the sensor
uint8 RED=1
uint8 BLUE=2
uint8 UNSURE=3    # something present but between the red/blue thresholds

string name       # source sensor name
uint8 color       # one of the values above
