import matplotlib.image as img
import pandas as pd

from utils import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--inp", help="name of csv file specifying dataset, i.e. small_dataset.csv", action="store", required=True)
    parser.add_argument("--outp", help="name of txt file to write output to, i.e. small_dataset_test_list.txt", action="store", required=True)
    
    args = parser.parse_args()

    input_file = args.inp
    full_input_file = os.path.join(get_basepath(), "resources", input_file)

    output_file = args.outp
    full_output_file = os.path.join(get_basepath(), "resources", output_file)

    image_dir = os.path.join(get_basepath(), "data", "figures")
    test_list = []

    # get list of dataset
    dataset_df = pd.read_csv(full_input_file, header=None)

    dataset_set = set()

    # extract good files into dataset and add to list of pos names
    for i in range(0, len(dataset_df)):
        png_name, _ = tuple(dataset_df.iloc[i])
        dataset_set.add(png_name)

    

    with open(full_output_file, "w") as outfile:
        for image in os.listdir(image_dir):
            if (not image.endswith(".png")) or (image not in dataset_set):
                continue
            plt.imshow(img.imread(os.path.join(image_dir, image)))
            plt.show()
            description = input("Would you like to include this image in the test set? Y/N\n")
            if description.lower() == "y":
                outfile.write(image.strip(".png") + "\n")
            elif description.lower() == "c":
                break
