/*
 * MIT License
 *
 * Copyright (c) 2023 Carnegie Mellon University
 *
 * This file is part of RoWild.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "frame.h"
#include "glob.hpp"
#include <fstream>
#include <string>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// WARNING: Do not use the following functions beyond this
// scope; they are tailored for the srec kernel with
// ICL-NUIM inputset

static bool fileExists(std::string fileName) {
    // Check whether the file exists and there is only one file that matches
    // the file name (no globbing)

    auto f = glob::glob(fileName);
    return f.size() == 1;
}

static double **readOneChannelImage(std::string fileName, int *height,
                                    int *width, double scale, int downsample) {
    assert(fileExists(fileName));

    int bpp;
    unsigned char *image = stbi_load(fileName.c_str(), width, height, &bpp, 0);

    int buffHeight =
        static_cast<int>(ceil(static_cast<double>(*height) / downsample));
    int buffWidth =
        static_cast<int>(ceil(static_cast<double>(*width) / downsample));

    double **buffer = new double *[buffHeight];
    for (int i = 0; i < buffHeight; i++) {
        buffer[i] = new double[buffWidth];
    }

    int r = 0, c = 0;
    for (int i = 0; i < *height; i += downsample) {
        for (int j = 0; j < *width; j += downsample) {
            int idx = i * (*width) + j;
            buffer[r][c] = static_cast<double>(image[idx]) * scale;

            if (++c == buffWidth) {
                c = 0;
                r++;
            }
        }
    }

    assert(r == buffHeight);
    assert(c == 0);

    *height = buffHeight;
    *width = buffWidth;
    stbi_image_free(image);

    return buffer;
}

static double ***readThreeChannelImage(std::string fileName, int *height,
                                       int *width, double scale,
                                       int downsample) {
    assert(fileExists(fileName));

    int bpp;
    unsigned char *image = stbi_load(fileName.c_str(), width, height, &bpp, 0);
    assert(bpp == 3);

    int buffHeight =
        static_cast<int>(ceil(static_cast<double>(*height) / downsample));
    int buffWidth =
        static_cast<int>(ceil(static_cast<double>(*width) / downsample));

    double ***buffer = new double **[buffHeight];
    for (int i = 0; i < buffHeight; i++) {
        buffer[i] = new double *[buffWidth];
        for (int j = 0; j < buffWidth; j++) {
            buffer[i][j] = new double[bpp];
        }
    }

    int r = 0, c = 0;
    for (int i = 0; i < *height; i += downsample) {
        for (int j = 0; j < *width; j += downsample) {
            int idx = i * (*width) + j;
            idx *= bpp;
            for (int k = 0; k < bpp; k++) {
                assert(idx + k < (*height) * (*width) * bpp);
                buffer[r][c][k] = static_cast<double>(image[idx + k]) * scale;
            }

            if (++c == buffWidth) {
                c = 0;
                r++;
            }
        }
    }
    assert(r == buffHeight);
    assert(c == 0);

    *height = buffHeight;
    *width = buffWidth;
    stbi_image_free(image);

    return buffer;
}

static double ***readNormals(std::string pathPrefix, int height, int width,
                             int downsample) {
    double ***buffer = new double **[height];
    for (int i = 0; i < height; i++) {
        buffer[i] = new double *[width];
        for (int j = 0; j < width; j++) {
            buffer[i][j] = new double[3];
        }
    }

    for (int dim = 0; dim < 3; dim++) {
        std::string fileName =
            pathPrefix + "-dim" + std::to_string(dim + 1) + ".txt";
        assert(fileExists(fileName));
        std::ifstream nFile(fileName);
        assert(nFile.good());

        std::string token;
        int sR = 0;
        for (int r = 0; r < height; r++) {
            while (sR++ % downsample != 0) {
                std::getline(nFile, token);
            }
            std::getline(nFile, token);

            std::stringstream row(token);

            double temp;
            int sC = 0;
            for (int c = 0; c < width; c++) {
                while (sC++ % downsample != 0) {
                    row >> temp;
                }
                row >> temp;
                buffer[r][c][dim] = temp;
            }
        }
        nFile.close();
    }

    return buffer;
}

Frame::Frame(std::string path, int number, double depthScale, double colorScale,
             int downsample) {
    std::string dPath = path + "/depth/" + std::to_string(number) + ".png";
    int dMapHeight, dMapWidth;
    this->depthMap = readOneChannelImage(dPath, &dMapHeight, &dMapWidth,
                                         depthScale, downsample);

    std::string cPath = path + "/rgb/" + std::to_string(number) + ".png";
    int cMapHeight, cMapWidth;
    this->colorMap = readThreeChannelImage(cPath, &cMapHeight, &cMapWidth,
                                           colorScale, downsample);

    assert(dMapHeight == cMapHeight);
    assert(dMapWidth == cMapWidth);
    this->frameHeight = dMapHeight;
    this->frameWidth = cMapWidth;

    std::string nPath = path + "/normal/" + std::to_string(number);
    this->normalMap =
        readNormals(nPath, this->frameHeight, this->frameWidth, downsample);
}

Frame::~Frame() {
    for (int i = 0; i < this->frameHeight; i++)
        delete[] depthMap[i];
    delete[] depthMap;

    for (int i = 0; i < this->frameHeight; i++) {
        for (int j = 0; j < this->frameWidth; j++) {
            delete[] colorMap[i][j];
            delete[] normalMap[i][j];
        }
        delete[] colorMap[i];
        delete[] normalMap[i];
    }
    delete[] colorMap;
    delete[] normalMap;

    if (this->vertexMap) {
        for (int i = 0; i < this->frameHeight; i++) {
            for (int j = 0; j < this->frameWidth; j++) {
                delete[] vertexMap[i][j];
            }
            delete[] vertexMap[i];
        }
        delete[] vertexMap;
    }
}

void Frame::printBuffers(std::string header) const {
    printf("%s:\n", header.c_str());
    printf("depthMap:\n");
    for (int i = 0; i < this->frameHeight; i++) {
        for (int j = 0; j < this->frameWidth; j++) {
            printf("%.2f ", this->depthMap[i][j]);
        }
        printf("\n");
    }

    printf("\ncolorMap:\n");
    for (int i = 0; i < this->frameHeight; i++) {
        for (int j = 0; j < this->frameWidth; j++) {
            for (int k = 0; k < 3; k++) {
                printf("%.2f ", this->colorMap[i][j][k]);
            }
            printf("\n");
        }
    }

    printf("\nnormalMap:\n");
    for (int i = 0; i < this->frameHeight; i++) {
        for (int j = 0; j < this->frameWidth; j++) {
            for (int k = 0; k < 3; k++) {
                printf("%.2f ", this->normalMap[i][j][k]);
            }
            printf("\n");
        }
    }

    printf("\nvertexMap:\n");
    for (int i = 0; i < this->frameHeight; i++) {
        for (int j = 0; j < this->frameWidth; j++) {
            for (int k = 0; k < 3; k++) {
                printf("%.2f ", this->vertexMap[i][j][k]);
            }
            printf("\n");
        }
    }
    printf("\n");
}
