from PIL import Image
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-i', type=str)

args = parser.parse_args()

path = args.i
path_ = path[:path.rfind(".")] + ".png"

Image.open(path).save(path_)
