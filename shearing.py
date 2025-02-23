import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
image = cv2.imread('your_image.jpg')  # Replace with your image path
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for correct display

# Get image dimensions
rows, cols = image.shape[:2]

# Define Shearing Matrix (Horizontal Shear)
shear_x = 0.5  # Shear factor along X-axis
shear_y = 0.0  # No Shear along Y-axis
shear_matrix = np.array([[1, shear_x, 0],
                         [shear_y, 1, 0]], dtype=np.float32)

# Apply Affine Transformation (Shearing)
sheared_image = cv2.warpAffine(image, shear_matrix, (cols + int(shear_x * rows), rows))

# Plot the Original and Sheared Images
plt.figure(figsize=(10,5))
plt.subplot(1, 2, 1)
plt.imshow(image)
plt.title("Original Image")
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(sheared_image)
plt.title("Sheared Image (X-axis)")
plt.axis('off')

plt.show()
