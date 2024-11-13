from PIL import Image
import numpy as np


def export_matrix_to_txt(matrix, filename):
    """
    Exports a NumPy boolean matrix to a text file.

    Args:
        matrix (np.ndarray): The boolean matrix to export.
        filename (str): The name of the output file.
    """
    # Convert the boolean matrix to integers (1 for True, 0 for False) and save as text
    np.savetxt(filename, matrix.astype(int), fmt="%d")


def import_matrix_from_txt(filename):
    """
    Imports a NumPy boolean matrix from a text file.

    Args:
        filename (str): The name of the input file.

    Returns:
        np.ndarray: The imported boolean matrix.
    """
    # Load the matrix and convert integers back to boolean
    matrix = np.loadtxt(filename, dtype=int)
    return matrix.astype(bool)

def image_to_boolean_matrix(image_path, crop_box=None, scale_size=(500, 500), threshold=128):
    """
    Converts a black-and-white image to a NumPy boolean matrix after cropping and downscaling.

    Args:
        image_path (str): Path to the image file.
        crop_box (tuple): The crop box as (left, upper, right, lower).
        scale_size (tuple): The resolution to scale the cropped area to (width, height).
        threshold (int): Grayscale threshold (0-255) to classify pixels as black or white.
                         Default is 128 (50%).

    Returns:
        np.ndarray: Boolean matrix where True represents black pixels and False represents white pixels.
    """
    # Open the image and convert it to grayscale
    image = Image.open(image_path).convert("L")  # "L" mode converts it to grayscale

    # Crop the image if a crop box is provided
    if crop_box:
        image = image.crop(crop_box)
        image.show(title="Cropped Area Preview")

    # Resize the image to the specified scale size
    image = image.resize(scale_size)

    # Convert image to a numpy array
    image_array = np.array(image)

    # Create a boolean matrix with True for black pixels and False for white
    boolean_matrix = image_array < threshold  # True for pixels below threshold, False otherwise

    return boolean_matrix



# Example usage
if __name__ == "__main__":
    # THis tool allow you to draw a map using paint and export it to a .txt. there is a function to import the .txt back to a numpy array.
    # the parameter scale size allow you to crop and scale the original picture ot make a smaller matrix.
    matrix = image_to_boolean_matrix("testData\mapWithBlackObstacle1.PNG")
    export_matrix_to_txt(matrix, "testData\mapWithBlackObstacle1.txt")
    print(str(matrix.shape))
