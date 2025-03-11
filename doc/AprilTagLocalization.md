## Understanding Position Computation Quality

Notes:

**Tag size:** computed as the square root of the area of the tag in pixels. The area is computed as the number of pixels in the convex hull of the tag corners.

**Tag Axis to Center Angle:** the angle between the tag's axis and the vector from the tag's center to the camera. In radians.

**Distance:** the distance from the camera to the tag's center. Closely related to the tag's size.

**Lighting:** the quality of the lighting in the scene. The tag detection algorithm is sensitive to lighting conditions. (Less Important)

**Motion Blur:** the amount of motion blur in the image. The tag detection algorithm is sensitive to motion blur. (Less Important)

### Single Tag vs Multi Tag