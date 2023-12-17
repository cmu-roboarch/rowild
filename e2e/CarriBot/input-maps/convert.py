#! /usr/bin/env python3

# MIT License
#
# Copyright (c) 2023 Carnegie Mellon University
#
# This file is part of RoWild.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from PIL import Image
import os, sys

def convert_image_to_text(image_path, output_file):
    with Image.open(image_path) as img:
        img = img.convert('RGB')
        width, height = img.size

        with open(output_file, 'w') as file:
            file.write(f'type octile\n')
            file.write(f'height {height}\n')
            file.write(f'width {width}\n')
            file.write(f'map\n')

            for y in range(height):
                for x in range(width):
                    r, g, b = img.getpixel((x, y))
                    grayscale = int((0.2989 * r + 0.5870 * g + 0.1140 * b))
                    num = 255 - grayscale
                    file.write(f"{'.' if num < 255//2 else '@'}")
                file.write('\n')

if len(sys.argv[1:]) != 1:
    print(f'Usage: {sys.argv[0]} path/to/image')
    exit(1)

filename = sys.argv[1]
assert os.path.isfile(filename), f'File {filename} does not exist'

convert_image_to_text(filename, os.path.splitext(filename)[0] + '.map')
