#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <algorithm>
#include <cmath>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline double uniform() {
    return rand() / (double)RAND_MAX;
}

static void random_direction(double v[3]) {
    double r1 = rand() / (double)RAND_MAX;
    double r2 = rand() / (double)RAND_MAX;
    double s  = sqrt(r2 * (1.0 - r2));
    v[0] = cos(2.0 * M_PI * r1) * 2.0 * s;
    v[1] = sin(2.0 * M_PI * r1) * 2.0 * s;
    v[2] = 1.0 - 2.0 * r2;
}

int main() {
    int W, H, C;
    int Wm, Hm, Cm;

    unsigned char* input = stbi_load("8733654151_b9422bb2ec_k.jpg",
                                     &W, &H, &C, STBI_rgb);
    unsigned char* model = stbi_load("redim.jpg",
                                     &Wm, &Hm, &Cm, STBI_rgb);

    if (!input || !model) {
        printf("Failed to load images.\n");
        return 1;
    }
    if (W * H != Wm * Hm) {
        printf("Input and model must have the same number of pixels.\n");
        return 1;
    }

    const int N = W * H;

    std::vector<double> I(N * 3);
    std::vector<double> M(N * 3);
    for (int i = 0; i < N * 3; i++) {
        I[i] = (double)input[i];
        M[i] = (double)model[i];
    }

    std::vector<std::pair<double, int> > projI(N);
    std::vector<std::pair<double, int> > projM(N);

    const int nbiter = 100;

    for (int iter = 0; iter < nbiter; iter++) {
        double v[3];
        random_direction(v);

        for (int i = 0; i < N; i++) {
            double di = I[3*i + 0]*v[0] + I[3*i + 1]*v[1] + I[3*i + 2]*v[2];
            double dm = M[3*i + 0]*v[0] + M[3*i + 1]*v[1] + M[3*i + 2]*v[2];
            projI[i] = std::pair<double, int>(di, i);
            projM[i] = std::pair<double, int>(dm, i);
        }

        std::sort(projI.begin(), projI.end());
        std::sort(projM.begin(), projM.end());

        for (int i = 0; i < N; i++) {
            int idx     = projI[i].second;
            double diff = projM[i].first - projI[i].first;
            I[3*idx + 0] += diff * v[0];
            I[3*idx + 1] += diff * v[1];
            I[3*idx + 2] += diff * v[2];
        }
    }

    std::vector<unsigned char> out(N * 3);
    for (int i = 0; i < N * 3; i++) {
        double x = I[i];
        if (x < 0.0)   x = 0.0;
        if (x > 255.0) x = 255.0;
        out[i] = (unsigned char)x;
    }

    stbi_write_png("image.png", W, H, 3, &out[0], 0);

    stbi_image_free(input);
    stbi_image_free(model);
    return 0;
}