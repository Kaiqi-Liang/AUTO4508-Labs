#ifndef IMAGE_H
#define IMAGE_H
#include "eyebot.h"

constexpr int IMAGE_SIZE = 128;
void read_pbm(const char *filename, BYTE **img);

#endif // IMAGE_H
